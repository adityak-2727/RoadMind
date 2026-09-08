"""carla_adapter.py - Phase 9 integration foundation + Phase 10 sensor
acquisition.

Responsible ONLY for integration-level CARLA operations: connect, spawn one
ego vehicle, attach RGB camera/LiDAR/radar sensors to it, read vehicle state
and sensor data, apply a control command, and clean up safely. Deliberately
contains NO perception fusion, tracking, prediction, or planning logic -
those stay in MATLAB (this project's existing, validated K1/K2 pipeline,
frozen). This module is a thin, isolated wrapper around the standard
`carla` Python client API so nothing else in the repository needs to import
`carla` directly - Phase 10 EXTENDS this same class rather than creating a
second connection layer.

Coordinate/unit conventions read FROM CARLA (see docs/carla_integration.md):
  - CARLA: left-handed coordinate system, X forward / Y right / Z up,
    location in meters, rotation (pitch, yaw, roll) in DEGREES, velocity in
    m/s.
  - CARLA radar detections (carla.RadarDetection): depth in meters, azimuth
    and altitude in RADIANS (NOT degrees - distinct from Transform.rotation,
    verified against CARLA's own python_api docs), velocity in m/s along
    the ray (CARLA convention: positive = moving away from the radar,
    negative = approaching).
  - This project's project-frame conversion (right-handed, y/yaw sign-
    flipped from CARLA's left-handed frame) is done entirely on the MATLAB
    side (carlaIntegration/matlab/carlaToProjectState.m for ego state;
    carlaIntegration/matlab/carlaCoordToProject.m, matching the same
    formula, for sensor data) - this module always returns CARLA's own raw
    values unconverted, so the transform lives in exactly one place per
    data type, not duplicated here.

Sensor images/point clouds/radar detections are delivered asynchronously by
CARLA (sensor.listen(callback), invoked on CARLA's own thread). Each sensor
here keeps only the SINGLE latest received frame (a bounded "queue" of
depth 1, per Phase 10's "latest-frame retrieval, avoid unbounded memory
growth" guidance) - MATLAB polls it via get_camera_frame()/
get_lidar_points()/get_radar_detections(), which return whatever is
currently the latest frame (or None if nothing has arrived yet).
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
    """Thin wrapper around one CARLA client/world/ego-vehicle/sensor session.

    Usage:
        adapter = CarlaAdapter(host="localhost", port=2000, timeout=10.0)
        adapter.connect()
        adapter.spawn_ego_vehicle(blueprint_id="vehicle.tesla.model3", spawn_point_index=0)
        adapter.attach_camera(width=800, height=600, fov=90.0)
        adapter.attach_lidar(channels=32, range_m=50.0, points_per_second=100000, rotation_frequency=10.0)
        adapter.attach_radar(horizontal_fov=30.0, vertical_fov=10.0, range_m=70.0, points_per_second=1500)
        frame = adapter.get_camera_frame()
        points = adapter.get_lidar_points()
        detections = adapter.get_radar_detections()
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

        self._camera = None
        self._camera_latest = None
        self._camera_fov = None

        self._lidar = None
        self._lidar_latest = None

        self._radar = None
        self._radar_latest = None

        self._collision_sensor = None
        self._collision_events = []  # accumulated for the vehicle's lifetime - Phase 14 ground-truth collision log

        self._other_actors = {}  # actor_id -> carla.Actor, for the Phase 10 validation scene / coordinate checks

    # ------------------------------------------------------------------
    # Connection
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

    def load_map(self, map_name: str):
        """Loads the named CARLA map if it is not already the active one.

        FOUND DURING PHASE 11.6 AUDIT: connect() (above, Phase 9,
        unmodified) never loads a specific map - it only attaches to
        whatever the server already has running, which defaults to
        Town10HD_Opt on a fresh launch. carlaConfig.m's/
        carlaIndianSceneConfig.m's mapName field was therefore dead
        configuration for any session that had not already had the right
        map loaded by some other means (this is exactly what happened
        during Phase 11.5 testing - Town03 was loaded once via a separate
        investigation script and simply stayed loaded for the rest of
        that server session, masking the gap). Confirmed live: a scene
        built against a cold server without this call produced 9 spawn
        failures out of 19 actors, because the hero scene's Town03-
        specific coordinates were being used to spawn actors into
        Town10HD_Opt's completely different geometry.

        This is additive - connect() itself is unchanged; callers that
        don't need a specific map (Phase 9/10/11/12's existing tests,
        which all run against whatever map is already loaded) are
        unaffected. client.load_world() is itself a somewhat slow,
        blocking call (full map teardown/reload), so this only calls it
        when the requested map is not already active.
        """
        if not self.is_connected():
            raise CarlaAdapterError("load_map() called before connect().")
        current_name = self._world.get_map().name  # e.g. "Carla/Maps/Town03"
        if current_name.endswith(f"/{map_name}") or current_name == map_name:
            return False  # already loaded, no reload needed

        # A full map teardown/reload takes noticeably longer than a normal
        # RPC call - the client's connect()-time timeout (config/
        # carlaConfig.m's timeoutSeconds, typically 10s) is too short and
        # was confirmed live to raise a timeout error here. Temporarily
        # extend it for this one blocking call, then restore.
        self._client.set_timeout(60.0)
        try:
            self._world = self._client.load_world(map_name)
        finally:
            self._client.set_timeout(self.timeout)
        return True

    # ------------------------------------------------------------------
    # Ego vehicle spawn / state / control (Phase 9, unchanged)
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

    def get_vehicle_state(self) -> dict:
        """Returns the ego vehicle's raw CARLA-frame state as a plain dict.

        Deliberately returns CARLA's own units/conventions unconverted -
        conversion into the project's egoState schema happens in
        carlaIntegration/matlab/carlaToProjectState.m, in exactly one place.
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

    def apply_control(self, steer: float = 0.0, throttle: float = 0.0, brake: float = 0.0):
        """Applies a raw CARLA VehicleControl to the ego vehicle.

        steer in [-1, 1], throttle in [0, 1], brake in [0, 1] - CARLA's own
        VehicleControl ranges. This is an integration-test control channel
        (Phase 9 Task 7); it is not a substitute for the project's real
        vehicleController.m, which still owns the actual driving logic.
        Never CARLA autopilot / Traffic Manager - this is the only control
        path used for the ego vehicle anywhere in this module.
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
    # Phase 10: RGB camera
    # ------------------------------------------------------------------
    def attach_camera(self, width: int = 800, height: int = 600, fov: float = 90.0,
                       mount_x: float = 1.5, mount_y: float = 0.0, mount_z: float = 2.0,
                       mount_pitch: float = 0.0, mount_yaw: float = 0.0, mount_roll: float = 0.0):
        """Spawns and attaches one sensor.camera.rgb to the ego vehicle.

        Mount position/rotation is in the VEHICLE's own local frame (CARLA
        convention for attach_to sensors: x=forward, y=right, z=up, meters;
        rotation in degrees) - e.g. the default (1.5, 0, 2.0) puts the
        camera 1.5m forward and 2m up from the vehicle origin, facing
        forward (yaw=0), a typical windshield-mount position.
        """
        if self._ego_vehicle is None:
            raise CarlaAdapterError("attach_camera() called before spawn_ego_vehicle().")

        bp_lib = self._world.get_blueprint_library()
        bp = bp_lib.find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(int(width)))
        bp.set_attribute('image_size_y', str(int(height)))
        bp.set_attribute('fov', str(float(fov)))

        transform = self._carla.Transform(
            self._carla.Location(x=mount_x, y=mount_y, z=mount_z),
            self._carla.Rotation(pitch=mount_pitch, yaw=mount_yaw, roll=mount_roll),
        )
        sensor = self._world.spawn_actor(bp, transform, attach_to=self._ego_vehicle)
        self._camera = sensor
        self._camera_latest = None
        self._camera_fov = float(fov)
        sensor.listen(self._on_camera_image)
        return sensor.id

    def _on_camera_image(self, image):
        self._camera_latest = image  # keep only the latest frame (bounded depth-1 "queue")

    def get_camera_frame(self) -> Optional[dict]:
        """Returns the latest camera frame as {rgb_bytes, width, height,
        fov, frame, timestamp_s}, or None if no frame has arrived yet.

        rgb_bytes is raw uint8 RGB (alpha channel dropped, BGRA->RGB
        reordered), row-major, width*height*3 bytes - reconstructed into an
        HxWx3 array on the MATLAB side.
        """
        if self._camera is None:
            raise CarlaAdapterError("get_camera_frame() called before attach_camera().")
        image = self._camera_latest
        if image is None:
            return None

        import numpy as np
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))  # CARLA delivers BGRA8
        rgb = arr[:, :, :3][:, :, ::-1]  # drop alpha, BGR -> RGB
        rgb = np.ascontiguousarray(rgb)

        return {
            "rgb_bytes": rgb.tobytes(),
            "width": image.width,
            "height": image.height,
            "fov": self._camera_fov,
            "frame": image.frame,
            "timestamp_s": image.timestamp,
        }

    # ------------------------------------------------------------------
    # Phase 10: LiDAR
    # ------------------------------------------------------------------
    def attach_lidar(self, channels: int = 32, range_m: float = 50.0, points_per_second: int = 100000,
                      rotation_frequency: float = 10.0, upper_fov: float = 10.0, lower_fov: float = -30.0,
                      mount_x: float = 0.0, mount_y: float = 0.0, mount_z: float = 2.2,
                      mount_pitch: float = 0.0, mount_yaw: float = 0.0, mount_roll: float = 0.0):
        """Spawns and attaches one sensor.lidar.ray_cast to the ego vehicle."""
        if self._ego_vehicle is None:
            raise CarlaAdapterError("attach_lidar() called before spawn_ego_vehicle().")

        bp_lib = self._world.get_blueprint_library()
        bp = bp_lib.find('sensor.lidar.ray_cast')
        bp.set_attribute('channels', str(int(channels)))
        bp.set_attribute('range', str(float(range_m)))
        bp.set_attribute('points_per_second', str(int(points_per_second)))
        bp.set_attribute('rotation_frequency', str(float(rotation_frequency)))
        bp.set_attribute('upper_fov', str(float(upper_fov)))
        bp.set_attribute('lower_fov', str(float(lower_fov)))

        transform = self._carla.Transform(
            self._carla.Location(x=mount_x, y=mount_y, z=mount_z),
            self._carla.Rotation(pitch=mount_pitch, yaw=mount_yaw, roll=mount_roll),
        )
        sensor = self._world.spawn_actor(bp, transform, attach_to=self._ego_vehicle)
        self._lidar = sensor
        self._lidar_latest = None
        sensor.listen(self._on_lidar_measurement)
        return sensor.id

    def _on_lidar_measurement(self, measurement):
        self._lidar_latest = measurement

    def get_lidar_points(self) -> Optional[dict]:
        """Returns the latest LiDAR sweep as {points_bytes, num_points,
        frame, timestamp_s}, or None if no sweep has arrived yet.

        points_bytes is a flat float32 buffer of [x, y, z, intensity]
        repeated num_points times (CARLA's own raw_data layout for
        sensor.lidar.ray_cast) - reconstructed into an Nx4 array on the
        MATLAB side. x/y/z are in the LIDAR SENSOR's own local frame
        (CARLA left-handed convention, meters); MATLAB applies the mount
        transform + world/project conversion.
        """
        if self._lidar is None:
            raise CarlaAdapterError("get_lidar_points() called before attach_lidar().")
        measurement = self._lidar_latest
        if measurement is None:
            return None

        import numpy as np
        arr = np.frombuffer(measurement.raw_data, dtype=np.float32)
        num_points = arr.shape[0] // 4

        return {
            "points_bytes": arr.tobytes(),
            "num_points": int(num_points),
            "frame": measurement.frame,
            "timestamp_s": measurement.timestamp,
        }

    # ------------------------------------------------------------------
    # Phase 10: Radar
    # ------------------------------------------------------------------
    def attach_radar(self, horizontal_fov: float = 30.0, vertical_fov: float = 10.0, range_m: float = 70.0,
                      points_per_second: int = 1500,
                      mount_x: float = 2.0, mount_y: float = 0.0, mount_z: float = 1.0,
                      mount_pitch: float = 0.0, mount_yaw: float = 0.0, mount_roll: float = 0.0):
        """Spawns and attaches one sensor.other.radar to the ego vehicle."""
        if self._ego_vehicle is None:
            raise CarlaAdapterError("attach_radar() called before spawn_ego_vehicle().")

        bp_lib = self._world.get_blueprint_library()
        bp = bp_lib.find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(float(horizontal_fov)))
        bp.set_attribute('vertical_fov', str(float(vertical_fov)))
        bp.set_attribute('range', str(float(range_m)))
        bp.set_attribute('points_per_second', str(int(points_per_second)))

        transform = self._carla.Transform(
            self._carla.Location(x=mount_x, y=mount_y, z=mount_z),
            self._carla.Rotation(pitch=mount_pitch, yaw=mount_yaw, roll=mount_roll),
        )
        sensor = self._world.spawn_actor(bp, transform, attach_to=self._ego_vehicle)
        self._radar = sensor
        self._radar_latest = None
        sensor.listen(self._on_radar_measurement)
        return sensor.id

    def _on_radar_measurement(self, measurement):
        self._radar_latest = measurement

    def get_radar_detections(self) -> Optional[dict]:
        """Returns the latest radar sweep as {detections_bytes,
        num_detections, frame, timestamp_s}, or None if no sweep has
        arrived yet.

        detections_bytes is a flat float32 buffer of
        [depth_m, azimuth_rad, altitude_rad, velocity_mps] repeated
        num_detections times - CARLA's own carla.RadarDetection fields,
        azimuth/altitude in RADIANS (verified against CARLA's python_api
        docs - distinct from Transform.rotation, which is degrees).
        velocity follows CARLA's own sign convention: positive = moving
        away from the radar along the ray, negative = approaching.
        """
        if self._radar is None:
            raise CarlaAdapterError("get_radar_detections() called before attach_radar().")
        measurement = self._radar_latest
        if measurement is None:
            return None

        import numpy as np
        rows = [(d.depth, d.azimuth, d.altitude, d.velocity) for d in measurement]
        arr = np.array(rows, dtype=np.float32) if rows else np.zeros((0, 4), dtype=np.float32)

        return {
            "detections_bytes": arr.tobytes(),
            "num_detections": int(arr.shape[0]),
            "frame": measurement.frame,
            "timestamp_s": measurement.timestamp,
        }

    # ------------------------------------------------------------------
    # Phase 14: sensor.other.collision - AUTHORITATIVE ground-truth
    # collision events from CARLA's own physics engine. Added because
    # every prior phase's "collision-free" claims relied entirely on
    # collisionCheck.m's own PREDICTED/geometric TTC evaluation of the
    # planner's candidate trajectories, never on an actual physics contact
    # event - correct for validating the planner's decision logic, but not
    # sufficient on its own for claiming the real CARLA vehicle never
    # physically touched anything. This sensor answers that second,
    # independent question.
    # ------------------------------------------------------------------
    def attach_collision_sensor(self):
        """Spawns and attaches one sensor.other.collision to the ego
        vehicle. Every collision event for the vehicle's remaining
        lifetime is appended to an internal list (fires once per contact,
        including ongoing scrapes - CARLA's own documented behavior),
        readable via get_collision_events()."""
        if self._ego_vehicle is None:
            raise CarlaAdapterError("attach_collision_sensor() called before spawn_ego_vehicle().")

        bp_lib = self._world.get_blueprint_library()
        bp = bp_lib.find('sensor.other.collision')
        transform = self._carla.Transform()  # collision sensor has no meaningful mount offset
        sensor = self._world.spawn_actor(bp, transform, attach_to=self._ego_vehicle)
        self._collision_sensor = sensor
        self._collision_events = []
        sensor.listen(self._on_collision)
        return sensor.id

    def _on_collision(self, event):
        # Phase 14.5 forensics: capture the FULL physical context at the
        # instant of impact, not just "something was hit". Positions and
        # velocities are read from the live actor handles inside the
        # callback, so they describe the moment of contact rather than
        # whenever MATLAB next happens to poll - which is what makes it
        # possible to tell repeated events from ONE continuous contact
        # apart from genuinely separate impacts (Phase 14.5 section 3).
        # MUST stay non-blocking. This runs on the sensor's own callback
        # thread, and a sustained contact fires it many times per second.
        # An earlier Phase 14.5 version queried the other actor's live
        # location/velocity/bounding box (and the ego's velocity) from
        # inside this callback; with hundreds of rapid events those calls
        # backed up and stalled the whole client - the maneuver hung with
        # no output for 6+ minutes. Only fields already carried BY the
        # event are read here; anything needing a world query is resolved
        # afterwards by resolve_actor_snapshot() below.
        other = event.other_actor
        impulse = event.normal_impulse
        ego_tf = event.transform  # ego transform at the moment of impact, already in the event

        self._collision_events.append({
            "frame": event.frame,
            "timestamp_s": event.timestamp,
            "other_actor_id": other.id if other is not None else -1,
            "other_actor_type": other.type_id if other is not None else "unknown",
            "impulse_magnitude": float((impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2) ** 0.5),
            "ego_x": float(ego_tf.location.x),
            "ego_y": float(ego_tf.location.y),
            "ego_yaw_deg": float(ego_tf.rotation.yaw),
        })

    def set_actor_hold(self, actor_id: int, hold: bool = True):
        """Brings a scripted NON-EGO vehicle to a real stop (or releases
        it), for the ego-proximity clamp in carlaIndianSceneTrafficStep.m.

        Why this exists (Phase 14.5, measured): the clamp previously just
        called set_target_velocity(0). That sets a target, it does NOT
        brake - a 5.5 m/s bus commanded to zero target velocity COASTS,
        and forensics showed it sliding into a stationary ego and coming
        to rest against it (998 collision events, mean impulse 50 - i.e.
        sustained resting contact rather than a single ram, with the ego
        wedged at a fixed position and -21.8 deg yaw for 16 seconds).
        Applying a real brake + hand brake actually arrests the vehicle
        within its own stopping distance instead. Never applied to the
        ego, and never CARLA autopilot - this is the same direct-control
        channel the scripted traffic already uses."""
        actor = self._other_actors.get(actor_id)
        if actor is None:
            actor = self._world.get_actor(int(actor_id))
        if actor is None:
            raise CarlaAdapterError(f"set_actor_hold: unknown actor_id {actor_id}.")
        if actor.type_id.startswith('walker.pedestrian'):
            speed = 0.0 if hold else 1.0
            control = self._carla.WalkerControl(
                direction=self._carla.Vector3D(x=1.0, y=0.0, z=0.0), speed=speed, jump=False)
            actor.apply_control(control)
            return
        if hold:
            actor.set_target_velocity(self._carla.Vector3D(x=0.0, y=0.0, z=0.0))
            actor.apply_control(self._carla.VehicleControl(
                throttle=0.0, steer=0.0, brake=1.0, hand_brake=True))
        else:
            actor.apply_control(self._carla.VehicleControl(
                throttle=0.0, steer=0.0, brake=0.0, hand_brake=False))

    def get_collision_count(self) -> int:
        """Cheap count of collision events so far - returns an int, not the
        whole list. get_collision_events() marshals every event across the
        Python/MATLAB boundary, so polling IT once per tick is O(n^2) in
        the number of events and was measured to crawl a run to a halt
        once the count reached the thousands (Phase 14.5). Use this for
        per-tick progress tracking and fetch the full list once, at the
        end."""
        if self._collision_sensor is None:
            raise CarlaAdapterError("get_collision_count() called before attach_collision_sensor().")
        return len(self._collision_events)

    def get_new_collision_events(self, since_index: int) -> list:
        """Phase 14.11: returns only events at index >= since_index (a plain
        Python list slice, O(k) in the number of NEW events, not O(n) in the
        total so far). Exists so a caller can log every event's actor id/
        impulse/pose as it happens without re-marshalling the whole,
        ever-growing list each tick - get_collision_events() does that, and
        Phase 14.5 measured polling it every tick to crawl a run to a halt
        once the count reached the thousands. Pass the count returned by the
        LAST call (or 0 initially) as since_index; the return value's length
        tells the caller the new current count."""
        if self._collision_sensor is None:
            raise CarlaAdapterError("get_new_collision_events() called before attach_collision_sensor().")
        return self._collision_events[int(since_index):]

    def resolve_actor_snapshot(self, actor_id: int) -> dict:
        """Looks up one actor's CURRENT location/extent by id, for
        post-run collision forensics. Called AFTER a run, never from
        inside a sensor callback (see _on_collision). Static scene
        objects - props, parked vehicles - have not moved, so this is
        their impact position too; for a moving actor it is only its
        end-of-run pose, and is labelled as such by the caller."""
        if not self.is_connected():
            raise CarlaAdapterError("resolve_actor_snapshot() called before connect().")
        actor = self._world.get_actor(int(actor_id))
        if actor is None:
            return {"found": False, "x": float('nan'), "y": float('nan'),
                    "extent_x": float('nan'), "extent_y": float('nan'), "type_id": "gone"}
        loc = actor.get_location()
        bb = getattr(actor, 'bounding_box', None)
        return {
            "found": True,
            "x": float(loc.x),
            "y": float(loc.y),
            "extent_x": float(bb.extent.x) if bb is not None else float('nan'),
            "extent_y": float(bb.extent.y) if bb is not None else float('nan'),
            "type_id": actor.type_id,
        }

    def get_collision_events(self) -> list:
        """Returns every collision event recorded since attach_collision_sensor()
        was called, as a list of dicts (frame, timestamp_s, other_actor_id,
        other_actor_type, impulse_magnitude). Empty list means zero
        physical contacts, not "sensor not attached" - raises if the
        sensor was never attached, so a caller cannot silently mistake
        "never checked" for "checked and clean"."""
        if self._collision_sensor is None:
            raise CarlaAdapterError("get_collision_events() called before attach_collision_sensor().")
        return list(self._collision_events)

    # ------------------------------------------------------------------
    # Phase 10: simulator-grounded actor/object metadata (NOT an
    # image-based detector - see this method's own docstring)
    # ------------------------------------------------------------------
    def get_nearby_actor_objects(self, range_m: float = 60.0) -> dict:
        """Returns ground-truth CARLA actor metadata for vehicles/walkers
        near the ego vehicle, as a flat float32 buffer, EXPLICITLY NOT a
        trained computer-vision detector output.

        This exists because Phase 10 requires "object/class identification
        information where available" without building a new detector; CARLA
        itself already knows the true class/position/velocity/bounding box
        of every actor it simulates (it has to, to render them), so this
        method reads that simulator-grounded ground truth directly via
        world.get_actors() - the same way this project's existing MATLAB
        scenarios (scenarios/*.m) already provide ground-truth agents for
        the synthetic perception pipeline to degrade. It is NOT derived
        from the RGB image, and callers (carlaIntegration/matlab/
        carlaActorObjectsToAgents.m) must not describe it as one.

        Each row: [actor_id, class_code, x, y, z, vx, vy, vz, yaw_deg,
        extent_x, extent_y, extent_z, distance_m]
        class_code is an integer index into a fixed CLASS_NAMES list (see
        carlaActorObjectsToAgents.m for the matching MATLAB-side table)
        rather than a string, to keep this a uniform numeric buffer like
        the other sensors.
        """
        if not self.is_connected() or self._ego_vehicle is None:
            raise CarlaAdapterError("get_nearby_actor_objects() called before spawn_ego_vehicle().")

        CLASS_NAMES = ["unknown", "car", "truck", "bus", "motorcycle", "bicycle", "pedestrian"]

        def classify(actor) -> int:
            # FIX (Phase 11.5, found during the Indian hero-scene audit): CARLA
            # vehicle blueprints expose their own 'base_type' attribute directly
            # (confirmed live: vehicle.bh.crossbike/diamondback.century/
            # gazelle.omafiets report base_type='bicycle';
            # vehicle.harley-davidson.low_rider/kawasaki.ninja/vespa.zx125/
            # yamaha.yzf report base_type='motorcycle') - this is authoritative
            # and must be checked BEFORE the wheel-count fallback. The previous
            # version checked num_wheels in (2,3) first, which routed every
            # 2-wheeled actor (bicycles included) to "motorcycle" and made the
            # "bicycle" entry in CLASS_NAMES above permanently unreachable dead
            # code - safety-relevant, since decision/behaviorDecision.m's
            # VULNERABLE_CLASSES and prediction/trajectoryPrediction.m's
            # irregularClasses both key on the class STRING "bicycle" for
            # vulnerable-road-user protection; a real bicycle silently
            # misclassified as "motorcycle" would receive neither.
            # num_wheels remains the fallback ONLY for a blueprint that reports
            # no base_type at all, never overriding a base_type that IS present.
            #
            # NO true auto-rickshaw/3-wheeler blueprint exists in CARLA 0.9.16's
            # stock vehicle library (confirmed by enumerating every
            # vehicle.* blueprint's base_type/number_of_wheels live - every
            # entry is 4-wheel car/truck/van/bus or 2-wheel bicycle/motorcycle,
            # nothing 3-wheeled). This is a real asset-library limitation, not
            # a classification bug - documented here rather than fabricating a
            # class the simulator cannot actually produce. See
            # config/carlaIndianSceneConfig.m for how the hero scene represents
            # an auto-rickshaw-equivalent using vehicle.vespa.zx125 (a small
            # scooter, the closest available visual/kinematic proxy),
            # explicitly labelled as a substitute, not a genuine match.
            type_id = actor.type_id
            if type_id.startswith('walker.pedestrian'):
                return CLASS_NAMES.index("pedestrian")
            if type_id.startswith('vehicle.'):
                attrs = actor.attributes
                base_type = attrs.get('base_type', '')
                if base_type == 'bicycle':
                    return CLASS_NAMES.index("bicycle")
                if base_type == 'motorcycle':
                    return CLASS_NAMES.index("motorcycle")
                if base_type == 'truck':
                    return CLASS_NAMES.index("truck")
                if base_type == 'bus' or base_type == 'Bus':
                    return CLASS_NAMES.index("bus")
                if base_type in ('car', 'van', ''):
                    num_wheels = int(attrs.get('number_of_wheels', 4))
                    if num_wheels in (2, 3) and base_type == '':
                        return CLASS_NAMES.index("motorcycle")  # fallback only when base_type is genuinely absent
                    return CLASS_NAMES.index("car")
                return CLASS_NAMES.index("car")  # any other populated base_type (e.g. an unseen future blueprint) - a car is the safer generic default over "unknown", which would drop it from VRU-adjacent handling entirely
            return CLASS_NAMES.index("unknown")

        ego_loc = self._ego_vehicle.get_location()
        rows = []
        for actor in self._world.get_actors():
            if actor.id == self._ego_vehicle.id:
                continue
            type_id = actor.type_id
            if not (type_id.startswith('vehicle.') or type_id.startswith('walker.pedestrian')):
                continue
            loc = actor.get_location()
            dist = loc.distance(ego_loc)
            if dist > range_m:
                continue
            vel = actor.get_velocity()
            transform = actor.get_transform()
            try:
                bbox = actor.bounding_box
                extent = (bbox.extent.x, bbox.extent.y, bbox.extent.z)
            except Exception:  # noqa: BLE001 - not every actor exposes a bounding box
                extent = (0.0, 0.0, 0.0)
            rows.append((
                float(actor.id), float(classify(actor)),
                loc.x, loc.y, loc.z,
                vel.x, vel.y, vel.z,
                transform.rotation.yaw,
                extent[0], extent[1], extent[2],
                dist,
            ))

        import numpy as np
        arr = np.array(rows, dtype=np.float32) if rows else np.zeros((0, 13), dtype=np.float32)
        snapshot = self._world.get_snapshot()
        return {
            "objects_bytes": arr.tobytes(),
            "num_objects": int(arr.shape[0]),
            "class_names": CLASS_NAMES,
            "frame": snapshot.frame,
            "timestamp_s": snapshot.timestamp.elapsed_seconds,
        }

    # ------------------------------------------------------------------
    # Phase 10: validation-scene / coordinate-verification helpers.
    # These spawn/control NON-EGO actors only, for testing sensor
    # coverage and the coordinate transform - never used to drive the
    # ego vehicle (ego is only ever driven via apply_control(), never
    # autopilot/Traffic Manager/a scripted trajectory).
    # ------------------------------------------------------------------
    def spawn_actor_relative_to_ego(self, blueprint_id: str, forward_m: float = 0.0,
                                     right_m: float = 0.0, up_m: float = 0.5,
                                     yaw_offset_deg: float = 0.0):
        """Spawns one non-ego actor at a position given relative to the
        ego vehicle's CURRENT transform (forward/right/up in meters, in
        the ego's own local frame; yaw_offset_deg added to the ego's
        current yaw). Used for coordinate-system verification (spawning
        at a known ahead/left/right offset and confirming the converted
        project-frame result) and the multi-actor validation scene.

        Returns the spawned actor's id, or None if try_spawn_actor
        failed (e.g. collision at that exact point). Tracked internally
        so destroy_other_actors()/disconnect() cleans it up - the same
        orphan-actor discipline as the ego vehicle and sensors.
        """
        if self._ego_vehicle is None:
            raise CarlaAdapterError("spawn_actor_relative_to_ego() called before spawn_ego_vehicle().")

        bp_lib = self._world.get_blueprint_library()
        matches = bp_lib.filter(blueprint_id)
        if len(matches) == 0:
            raise CarlaAdapterError(f"No blueprint matching '{blueprint_id}' found.")
        blueprint = matches[0]
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'false')

        ego_transform = self._ego_vehicle.get_transform()
        forward_vec = ego_transform.get_forward_vector()
        right_vec = ego_transform.get_right_vector()
        loc = ego_transform.location + forward_vec * forward_m + right_vec * right_m
        loc.z += up_m
        rotation = self._carla.Rotation(
            pitch=ego_transform.rotation.pitch,
            yaw=ego_transform.rotation.yaw + yaw_offset_deg,
            roll=ego_transform.rotation.roll,
        )
        transform = self._carla.Transform(loc, rotation)

        actor = self._world.try_spawn_actor(blueprint, transform)
        if actor is None:
            return None
        self._other_actors[actor.id] = actor
        return actor.id

    def set_actor_target_velocity(self, actor_id: int, vx: float, vy: float, vz: float = 0.0):
        """Sets a spawned non-ego actor's velocity - test-only determinism
        for coordinate-verification (Phase 10) and controlled Phase 12
        tracking/prediction demonstrations, not a substitute for organic
        physics-driven motion. Never applied to the ego vehicle.

        CARLA pedestrians (type_id 'walker.pedestrian.*') do NOT respond
        to Actor.set_target_velocity() the way vehicles do - walkers are
        driven by carla.WalkerControl (direction + speed), a different
        control path entirely (found live during Phase 12 development: a
        pedestrian given set_target_velocity never actually moved). This
        method detects a walker actor and issues the equivalent
        WalkerControl instead, so both actor kinds get real, verifiable
        motion from the same MATLAB-facing call.
        """
        actor = self._other_actors.get(actor_id)
        if actor is None:
            raise CarlaAdapterError(f"set_actor_target_velocity: unknown actor_id {actor_id} (not spawned via spawn_actor_relative_to_ego).")

        if actor.type_id.startswith('walker.pedestrian'):
            speed = (vx ** 2 + vy ** 2 + vz ** 2) ** 0.5
            if speed < 1e-6:
                direction = self._carla.Vector3D(x=1.0, y=0.0, z=0.0)
                speed = 0.0
            else:
                direction = self._carla.Vector3D(x=vx / speed, y=vy / speed, z=vz / speed)
            control = self._carla.WalkerControl(direction=direction, speed=speed, jump=False)
            actor.apply_control(control)
        else:
            actor.set_target_velocity(self._carla.Vector3D(x=vx, y=vy, z=vz))

    def set_actor_velocity_relative_to_ego(self, actor_id: int, forward_mps: float = 0.0,
                                            right_mps: float = 0.0, up_mps: float = 0.0):
        """Sets a spawned non-ego actor's velocity as components along the
        EGO's CURRENT forward/right axes (recomputed fresh from the ego's
        live transform every call - not a one-time snapshot), then
        forwards the resulting world-frame (vx, vy, vz) to
        set_actor_target_velocity() (so the walker/vehicle branching there
        is reused unchanged).

        Why this exists (Phase 13): scenario scripts staging a controlled
        conflict actor (crossing/merging/etc.) need to guarantee that
        actor's motion actually closes toward the ego's path. Picking a
        raw CARLA world-frame (vx, vy) by hand only does that if the
        picker already knows the ego's world heading at that spot - easy
        to get backwards or zero-out the very component that mattered
        (found live: a bicycle given world-frame vel=(-1.0, 0.0) at a
        road oriented ~parallel to world X turned out to have ZERO
        lateral closing component, so it passed the ego at a constant
        offset for the entire run without ever entering the planning
        corridor - not a perception/decision/planning defect, a scenario-
        authoring one). Expressing the intended motion directly in the
        ego's own forward/right axes - the SAME axes
        spawn_actor_relative_to_ego() already uses to place the actor -
        removes that class of mistake entirely.
        """
        if self._ego_vehicle is None:
            raise CarlaAdapterError("set_actor_velocity_relative_to_ego() called before spawn_ego_vehicle().")
        ego_transform = self._ego_vehicle.get_transform()
        forward_vec = ego_transform.get_forward_vector()
        right_vec = ego_transform.get_right_vector()
        vx = forward_vec.x * forward_mps + right_vec.x * right_mps
        vy = forward_vec.y * forward_mps + right_vec.y * right_mps
        self.set_actor_target_velocity(actor_id, vx, vy, up_mps)

    def get_actor_state(self, actor_id: int) -> dict:
        """Returns a non-ego test actor's raw CARLA-frame state, in the
        same shape as get_vehicle_state(), for coordinate-verification
        readback."""
        actor = self._other_actors.get(actor_id)
        if actor is None:
            raise CarlaAdapterError(f"get_actor_state: unknown actor_id {actor_id} (not spawned via spawn_actor_relative_to_ego).")
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        snapshot = self._world.get_snapshot()
        return {
            "actor_id": actor.id,
            "type_id": actor.type_id,
            "location": {"x": transform.location.x, "y": transform.location.y, "z": transform.location.z},
            "rotation_deg": {"pitch": transform.rotation.pitch, "yaw": transform.rotation.yaw, "roll": transform.rotation.roll},
            "velocity_mps": {"x": velocity.x, "y": velocity.y, "z": velocity.z},
            "frame": snapshot.frame,
            "timestamp_s": snapshot.timestamp.elapsed_seconds,
        }

    def set_spectator_transform(self, x: float, y: float, z: float, pitch_deg: float, yaw_deg: float, roll_deg: float) -> None:
        """EMERGENCY DEMO BRIDGE: points CARLA's free-fly spectator camera
        at an explicit world transform, so the evaluator's CARLA window
        shows the intersection instead of wherever the camera happened to
        default to. Read-only w.r.t. simulation state - moves only the
        spectator, never any actor."""
        if self._world is None:
            raise CarlaAdapterError("set_spectator_transform() called before connect().")
        transform = self._carla.Transform(
            self._carla.Location(x=x, y=y, z=z),
            self._carla.Rotation(pitch=pitch_deg, yaw=yaw_deg, roll=roll_deg),
        )
        self._world.get_spectator().set_transform(transform)

    # ------------------------------------------------------------------
    # Phase 11.5: absolute-world-transform spawning, for the Indian hero
    # scene. spawn_actor_relative_to_ego() (above) positions an actor
    # relative to the EGO's transform, which is exactly what Phase 10's
    # coordinate-verification tests needed. A fixed, reproducible hero
    # scene needs the opposite: actors (including the ego itself) placed
    # at known, deterministic WORLD coordinates resolved once against the
    # map's own road network (config/carlaIndianSceneConfig.m), the same
    # "live-verified, then hardcoded" convention config/carlaConfig.m
    # already uses for its map name/spawn point. These are new, additive
    # methods - spawn_actor_relative_to_ego() and spawn_ego_vehicle() are
    # unchanged.
    # ------------------------------------------------------------------
    def spawn_ego_vehicle_at_transform(self, blueprint_id: str, x: float, y: float, z: float, yaw_deg: float):
        """Spawns the ego vehicle at an explicit world transform, instead
        of by CARLA's own spawn_points() index (spawn_ego_vehicle()'s
        approach). Used for the hero scene, where the ego's approach
        point was resolved from the map's actual road waypoints (see
        config/carlaIndianSceneConfig.m) rather than picked from whatever
        spawn points the map happens to define."""
        if not self.is_connected():
            raise CarlaAdapterError("spawn_ego_vehicle_at_transform() called before connect().")
        if self._ego_vehicle is not None:
            raise CarlaAdapterError("Ego vehicle already spawned - call disconnect()/destroy_ego_vehicle() first.")

        blueprint_library = self._world.get_blueprint_library()
        matches = blueprint_library.filter(blueprint_id)
        if len(matches) == 0:
            raise CarlaAdapterError(f"No blueprint matching '{blueprint_id}' found.")
        blueprint = matches[0]

        transform = self._carla.Transform(
            self._carla.Location(x=x, y=y, z=z),
            self._carla.Rotation(pitch=0.0, yaw=yaw_deg, roll=0.0),
        )
        actor = self._world.try_spawn_actor(blueprint, transform)
        if actor is None:
            raise CarlaAdapterError(
                f"world.try_spawn_actor returned None at ({x:.2f},{y:.2f},{z:.2f}) - "
                "likely occupied/colliding at this exact point."
            )
        self._ego_vehicle = actor
        return actor.id

    def spawn_actor_at_transform(self, blueprint_id: str, x: float, y: float, z: float, yaw_deg: float):
        """Spawns one non-ego actor at an explicit world transform. Tracked
        in self._other_actors exactly like spawn_actor_relative_to_ego(),
        so destroy_other_actors()/disconnect() cleans it up identically -
        no separate bookkeeping or cleanup path needed. Returns the actor
        id, or None if the spawn point was occupied/colliding (never
        raises for that specific, expected case - callers building a
        scene with many actors need to handle individual placement
        failures gracefully, matching spawn_actor_relative_to_ego()'s
        contract)."""
        if not self.is_connected():
            raise CarlaAdapterError("spawn_actor_at_transform() called before connect().")

        blueprint_library = self._world.get_blueprint_library()
        matches = blueprint_library.filter(blueprint_id)
        if len(matches) == 0:
            raise CarlaAdapterError(f"No blueprint matching '{blueprint_id}' found.")
        blueprint = matches[0]
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'false')

        transform = self._carla.Transform(
            self._carla.Location(x=x, y=y, z=z),
            self._carla.Rotation(pitch=0.0, yaw=yaw_deg, roll=0.0),
        )
        actor = self._world.try_spawn_actor(blueprint, transform)
        if actor is None:
            return None
        self._other_actors[actor.id] = actor
        return actor.id

    def capture_snapshot(self, x: float, y: float, z: float, yaw_deg: float, pitch_deg: float = -30.0,
                          width: int = 1280, height: int = 800, fov: float = 90.0, timeout_s: float = 5.0):
        """Captures ONE RGB frame from a camera at an explicit world
        transform (e.g. an elevated bird's-eye view over the hero scene),
        independent of the single ego-mounted camera slot
        attach_camera()/get_camera_frame() manage. Spawns a temporary
        camera, waits (blocking, up to timeout_s) for exactly one frame
        via its own listen() callback, destroys the camera, and returns
        the same {rgb_bytes, width, height, ...} shape get_camera_frame()
        returns - Phase 11.5 evidence capture only, not part of the
        production sensor path.
        """
        if not self.is_connected():
            raise CarlaAdapterError("capture_snapshot() called before connect().")

        bp_lib = self._world.get_blueprint_library()
        bp = bp_lib.find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(int(width)))
        bp.set_attribute('image_size_y', str(int(height)))
        bp.set_attribute('fov', str(float(fov)))
        transform = self._carla.Transform(
            self._carla.Location(x=x, y=y, z=z),
            self._carla.Rotation(pitch=pitch_deg, yaw=yaw_deg, roll=0.0),
        )
        sensor = self._world.spawn_actor(bp, transform)

        captured = {}

        def _on_image(image):
            if 'image' not in captured:
                captured['image'] = image

        sensor.listen(_on_image)
        start = time.time()
        try:
            while 'image' not in captured and (time.time() - start) < timeout_s:
                time.sleep(0.05)
        finally:
            sensor.stop()
            sensor.destroy()

        if 'image' not in captured:
            return None

        import numpy as np
        image = captured['image']
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        rgb = arr[:, :, :3][:, :, ::-1]
        rgb = np.ascontiguousarray(rgb)
        return {
            "rgb_bytes": rgb.tobytes(),
            "width": image.width,
            "height": image.height,
            "fov": fov,
            "frame": image.frame,
            "timestamp_s": image.timestamp,
        }

    def freeze_traffic_lights(self, x: float, y: float, range_m: float = 80.0):
        """Finds every traffic light within range_m of (x, y) and freezes
        it in a fixed state (does not cycle) - Phase 11.5's requirement
        that traffic-light infrastructure be visually present but NEVER
        control right-of-way. This does not delete or hide the traffic
        lights (they remain visible, real CARLA actors, for visual
        realism) - it only stops them from cycling, and this project's
        decision stack (decision/behaviorDecision.m) has no code path
        that reads CARLA traffic-light state at all, so "not controlling
        right-of-way" is true by construction on the MATLAB side
        regardless; this method only prevents a jury from seeing
        conflicting red/green phases that might visually suggest signal
        control that isn't actually happening.
        Returns the number of traffic lights frozen."""
        if not self.is_connected():
            raise CarlaAdapterError("freeze_traffic_lights() called before connect().")

        # All four arms set to the SAME fixed state (never a conflicting
        # red/green pattern that would visually imply active control).
        # Yellow was chosen over red/green: a fixed all-way yellow/amber
        # reads to a viewer as "caution signal, not actively directing
        # traffic" (the same real-world convention as an all-way flashing
        # yellow at an uncontrolled intersection), rather than red (which
        # visually implies "everyone should be stopped, someone is being
        # controlled") or green (which implies active right-of-way grant).
        center = self._carla.Location(x=x, y=y, z=0.0)
        count = 0
        for actor in self._world.get_actors().filter('traffic.traffic_light*'):
            if actor.get_location().distance(center) <= range_m:
                actor.freeze(True)
                actor.set_state(self._carla.TrafficLightState.Yellow)
                count += 1
        return count

    def destroy_other_actors(self):
        """Destroys every actor spawned via spawn_actor_relative_to_ego().
        Safe to call even if none were spawned, and safe to call more
        than once."""
        for actor_id, actor in list(self._other_actors.items()):
            try:
                actor.destroy()
            except Exception:  # noqa: BLE001 - destroy() can fail if the actor is already gone
                pass
            del self._other_actors[actor_id]

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_ego_vehicle(self):
        if self._ego_vehicle is not None:
            self._ego_vehicle.destroy()
            self._ego_vehicle = None

    def destroy_sensors(self):
        """Stops and destroys camera/LiDAR/radar/collision if attached.
        Safe to call even if none were ever attached, and safe to call
        more than once."""
        for attr_sensor, attr_latest in (
            ('_camera', '_camera_latest'),
            ('_lidar', '_lidar_latest'),
            ('_radar', '_radar_latest'),
            ('_collision_sensor', '_collision_events'),
        ):
            sensor = getattr(self, attr_sensor)
            if sensor is not None:
                try:
                    sensor.stop()
                except Exception:  # noqa: BLE001 - stop() can fail if already stopped/destroyed server-side
                    pass
                try:
                    sensor.destroy()
                except Exception:  # noqa: BLE001 - destroy() can fail if the actor is already gone
                    pass
                setattr(self, attr_sensor, None)
                setattr(self, attr_latest, None)

    def disconnect(self):
        """Destroys sensors and the ego actor if present, then drops the
        client reference.

        CARLA's Python client has no explicit "close" call in the stable
        API - dropping the reference is the documented cleanup pattern.
        Safe to call multiple times. Sensors are destroyed BEFORE the ego
        vehicle they're attached to, since a sensor attached to an already
        -destroyed actor can otherwise be left orphaned server-side. Any
        non-ego actors spawned via spawn_actor_relative_to_ego() (Phase 10
        validation scene / coordinate checks) are destroyed too.
        """
        self.destroy_sensors()
        self.destroy_other_actors()
        self.destroy_ego_vehicle()
        self._client = None
        self._world = None

    def cleanup(self):
        """Alias for disconnect(), for callers that prefer this name."""
        self.disconnect()


def self_test(host: str = "localhost", port: int = 2000, timeout: float = 5.0) -> int:
    """Phase 9 integration self-test (connect/spawn/state/control/cleanup),
    runnable standalone: python carla_adapter.py
    Exits 0 only if every step actually succeeded against a live CARLA
    server. Exits 1 (with a clear message, no fabricated success) if CARLA
    is not reachable in this environment.
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
