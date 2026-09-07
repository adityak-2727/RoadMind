"""carla_adapter.py - Phase 9 integration-foundation adapter.

Responsible ONLY for integration-level CARLA operations: connect, spawn one
ego vehicle, read its state, apply a control command, and clean up safely.
Deliberately contains NO perception, prediction, or planning logic - those
stay in MATLAB (this project's existing, validated K1/K2 pipeline), per the
Phase 9 task boundary. This module is a thin, isolated wrapper around the
standard `carla` Python client API so nothing else in the repository needs
to import `carla` directly.

Requires the `carla` package (CARLA's own Python API) to be installed and
a CARLA server already running and reachable - this module does not launch
CARLA itself. As of this Phase 9 pass, no CARLA installation exists on the
development machine this was written on (see docs/carla_integration.md for
the full, honest environment inspection); every function below therefore
also has to be considered UNTESTED against a live server. The code follows
CARLA's documented, standard client API exactly (client/world/actor model),
not a guessed API - see https://carla.readthedocs.io/en/latest/python_api/
for the reference this was written against.

Coordinate/unit conventions read FROM CARLA (see docs/carla_integration.md
for the full writeup, not just this summary):
  - CARLA: left-handed coordinate system, X forward / Y right / Z up,
    location in meters, rotation (pitch, yaw, roll) in DEGREES, velocity in
    m/s, angular velocity in deg/s.
  - This project's egoState (config/createEgoState.m): x, y in meters,
    yaw in RADIANS (right-handed, CCW-positive, matching every existing
    perception/prediction/planning formula's cos/sin/atan2 usage),
    velocity as a SCALAR longitudinal speed in m/s (not a vector).
  - The actual left-handed -> right-handed and degrees -> radians
    conversion is done on the MATLAB side (carla/matlab/carlaToProjectState.m),
    not here - this module returns CARLA's raw values unmodified (as a
    plain dict) so the conversion is in exactly one, clearly documented
    place, not duplicated.
"""

from __future__ import annotations

import sys
import time
from typing import Optional


class CarlaAdapterError(RuntimeError):
    """Raised for any CARLA integration failure (connection, spawn, etc.).

    Kept as a single, distinct exception type so MATLAB-side callers (via
    py.* calls) can catch integration failures specifically rather than
    letting an unrelated Python exception surface confusingly.
    """


class CarlaAdapter:
    """Thin wrapper around one CARLA client/world/ego-vehicle session.

    Usage:
        adapter = CarlaAdapter(host="localhost", port=2000, timeout=10.0)
        adapter.connect()
        adapter.spawn_ego_vehicle(blueprint_id="vehicle.tesla.model3", spawn_point_index=0)
        state = adapter.get_vehicle_state()
        adapter.apply_control(steer=0.0, throttle=0.0, brake=0.0)
        adapter.cleanup()
    """

    def __init__(self, host: str = "localhost", port: int = 2000, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._carla = None
        self._client = None
        self._world = None
        self._ego_vehicle = None

    # ------------------------------------------------------------------
    # Task 1/2: connection
    # ------------------------------------------------------------------
    def connect(self):
        """Import the carla package and connect to a running CARLA server.

        Raises CarlaAdapterError with a clear message if the `carla`
        package is missing or the server is unreachable - never lets a
        raw import/socket error surface as the failure mode.
        """
        try:
            import carla  # local import: only required once a real connection is attempted
        except ImportError as exc:
            raise CarlaAdapterError(
                "The 'carla' Python package is not installed. Install CARLA's "
                "Python API (matching your CARLA server version) before calling "
                "connect(). See docs/carla_integration.md."
            ) from exc

        self._carla = carla
        try:
            self._client = carla.Client(self.host, self.port)
            self._client.set_timeout(self.timeout)
            self._world = self._client.get_world()  # raises if the server isn't reachable
        except Exception as exc:  # noqa: BLE001 - deliberately broad: any transport failure means "not connected"
            self._client = None
            self._world = None
            raise CarlaAdapterError(
                f"Could not connect to a CARLA server at {self.host}:{self.port} "
                f"within {self.timeout}s. Is CarlaUE4 running and reachable? ({exc})"
            ) from exc

        return True

    def is_connected(self) -> bool:
        return self._world is not None

    # ------------------------------------------------------------------
    # Task 5: spawn one deterministic ego vehicle
    # ------------------------------------------------------------------
    def spawn_ego_vehicle(self, blueprint_id: str = "vehicle.tesla.model3", spawn_point_index: int = 0):
        """Spawns exactly one ego vehicle at a deterministic spawn point.

        spawn_point_index is 0-based, matching CARLA's own
        world.get_map().get_spawn_points() indexing (the MATLAB-side
        adapter converts from config/carlaConfig.m's 1-based
        spawnPointIndex before calling this).
        """
        if not self.is_connected():
            raise CarlaAdapterError("spawn_ego_vehicle() called before connect().")

        blueprint_library = self._world.get_blueprint_library()
        matches = blueprint_library.filter(blueprint_id)
        if len(matches) == 0:
            raise CarlaAdapterError(
                f"No blueprint matching '{blueprint_id}' found in this CARLA version's blueprint "
                "library. Check config/carlaConfig.m's egoBlueprint against the installed CARLA version."
            )
        blueprint = matches[0]

        spawn_points = self._world.get_map().get_spawn_points()
        if len(spawn_points) == 0:
            raise CarlaAdapterError("The current map has no defined spawn points.")
        if spawn_point_index < 0 or spawn_point_index >= len(spawn_points):
            raise CarlaAdapterError(
                f"spawn_point_index {spawn_point_index} is out of range for this map "
                f"(0..{len(spawn_points) - 1} available)."
            )
        spawn_point = spawn_points[spawn_point_index]

        actor = self._world.try_spawn_actor(blueprint, spawn_point)
        if actor is None:
            raise CarlaAdapterError(
                f"world.try_spawn_actor returned None at spawn point {spawn_point_index} "
                "(likely occupied/colliding). Try a different spawn_point_index."
            )
        self._ego_vehicle = actor
        return actor.id

    # ------------------------------------------------------------------
    # Task 6: read ego state
    # ------------------------------------------------------------------
    def get_vehicle_state(self) -> dict:
        """Returns the ego vehicle's raw CARLA-frame state as a plain dict.

        Deliberately returns CARLA's own units/conventions unconverted
        (see this module's docstring) - conversion into the project's
        egoState schema happens in carla/matlab/carlaToProjectState.m, in
        exactly one place.
        """
        if self._ego_vehicle is None:
            raise CarlaAdapterError("get_vehicle_state() called before spawn_ego_vehicle().")

        transform = self._ego_vehicle.get_transform()
        velocity = self._ego_vehicle.get_velocity()
        angular_velocity = self._ego_vehicle.get_angular_velocity()
        try:
            acceleration = self._ego_vehicle.get_acceleration()
            accel_dict = {"x": acceleration.x, "y": acceleration.y, "z": acceleration.z}
        except Exception:  # noqa: BLE001 - acceleration isn't available on every CARLA version/actor state
            accel_dict = None

        snapshot = self._world.get_snapshot()

        return {
            "actor_id": self._ego_vehicle.id,
            "location": {"x": transform.location.x, "y": transform.location.y, "z": transform.location.z},
            "rotation_deg": {"pitch": transform.rotation.pitch, "yaw": transform.rotation.yaw, "roll": transform.rotation.roll},
            "velocity_mps": {"x": velocity.x, "y": velocity.y, "z": velocity.z},
            "angular_velocity_deg_s": {"x": angular_velocity.x, "y": angular_velocity.y, "z": angular_velocity.z},
            "acceleration_mps2": accel_dict,
            "frame": snapshot.frame,
            "timestamp_s": snapshot.timestamp.elapsed_seconds,
        }

    # ------------------------------------------------------------------
    # Task 7: send control commands
    # ------------------------------------------------------------------
    def apply_control(self, steer: float = 0.0, throttle: float = 0.0, brake: float = 0.0):
        """Applies a raw CARLA VehicleControl to the ego vehicle.

        steer in [-1, 1], throttle in [0, 1], brake in [0, 1] - CARLA's own
        VehicleControl ranges. This is an integration-test control channel
        only (Phase 9 Task 7); it is not a substitute for the project's
        real vehicleController.m, which still owns the actual driving
        logic in later phases.
        """
        if self._ego_vehicle is None:
            raise CarlaAdapterError("apply_control() called before spawn_ego_vehicle().")

        steer = max(-1.0, min(1.0, float(steer)))
        throttle = max(0.0, min(1.0, float(throttle)))
        brake = max(0.0, min(1.0, float(brake)))

        control = self._carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
        self._ego_vehicle.apply_control(control)
        return {"steer": steer, "throttle": throttle, "brake": brake}

    # ------------------------------------------------------------------
    # Task 5/8/9: cleanup
    # ------------------------------------------------------------------
    def destroy_ego_vehicle(self):
        if self._ego_vehicle is not None:
            self._ego_vehicle.destroy()
            self._ego_vehicle = None

    def disconnect(self):
        """Destroys the ego actor if present and drops the client reference.

        CARLA's Python client has no explicit "close" call in the stable
        API - dropping the reference is the documented cleanup pattern.
        Safe to call multiple times.
        """
        self.destroy_ego_vehicle()
        self._client = None
        self._world = None

    def cleanup(self):
        """Alias for disconnect(), for callers that prefer this name."""
        self.disconnect()


def self_test(host: str = "localhost", port: int = 2000, timeout: float = 5.0) -> int:
    """Phase 9 integration self-test (Tasks 5-8), runnable standalone:
        python carla_adapter.py
    Exits 0 only if every step actually succeeded against a live CARLA
    server. Exits 1 (with a clear message, no fabricated success) if CARLA
    is not reachable in this environment - this is the expected, honestly
    reported outcome on a machine with no CARLA installation.
    """
    adapter = CarlaAdapter(host=host, port=port, timeout=timeout)
    try:
        adapter.connect()
    except CarlaAdapterError as exc:
        print(f"[carla_adapter self-test] SKIPPED - CARLA not reachable: {exc}")
        return 1

    try:
        actor_id = adapter.spawn_ego_vehicle()
        print(f"[carla_adapter self-test] Spawned ego vehicle, actor_id={actor_id}")

        state_before = adapter.get_vehicle_state()
        print(f"[carla_adapter self-test] State before control: {state_before}")

        adapter.apply_control(steer=0.0, throttle=0.3, brake=0.0)
        time.sleep(1.0)

        state_after = adapter.get_vehicle_state()
        print(f"[carla_adapter self-test] State after control: {state_after}")

        adapter.apply_control(steer=0.0, throttle=0.0, brake=1.0)
        time.sleep(0.5)

        print("[carla_adapter self-test] PASSED")
        return 0
    finally:
        adapter.cleanup()
        print("[carla_adapter self-test] Cleanup complete.")


if __name__ == "__main__":
    sys.exit(self_test())
