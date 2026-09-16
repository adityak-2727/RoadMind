"""Phase 15 scene supervision only. Never commands or relocates the ego."""
import json
import math


class HeroSupervisor:
    def __init__(self, adapter):
        self.adapter = adapter
        self.world = adapter._world
        self.client = adapter._client
        self.owned = set()
        self.traffic = []
        self.original_weather = self.world.get_weather()
        self.light_states = [(a, a.get_state(), a.is_frozen())
                             for a in self.world.get_actors().filter('traffic.traffic_light*')]

    def preflight(self):
        server = self.client.get_server_version()
        client = self.client.get_client_version()
        if not server.startswith('0.9.16') or not client.startswith('0.9.16'):
            raise RuntimeError(f'CARLA 0.9.16 required: server={server}, client={client}')
        name = self.world.get_map().name
        if name.split('/')[-1] != 'Town03':
            raise RuntimeError(f'Town03 required, found {name}')
        if self.world.get_settings().synchronous_mode:
            raise RuntimeError('Hero demo requires the existing asynchronous CARLA loop')
        return json.dumps(dict(server=server, client=client, map=name,
                               timing='asynchronous/free-running'))

    def remember_owned(self):
        # Only IDs held by THIS adapter, never all actors or a blueprint filter.
        actors = list(self.adapter._other_actors.values())
        actors += [self.adapter._ego_vehicle, self.adapter._camera,
                   self.adapter._lidar, self.adapter._radar,
                   self.adapter._collision_sensor]
        self.owned.update(a.id for a in actors if a is not None)

    def survivors(self):
        live = {a.id for a in self.world.get_actors()}
        return json.dumps(sorted(self.owned & live))

    def configure_traffic(self, manifest):
        self.traffic = json.loads(manifest)
        self.start = self.world.get_snapshot().timestamp.elapsed_seconds
        for a in self.traffic:
            a['initial_heading'] = math.radians(a['yawDeg'])
            a['heading'] = a['initial_heading']
            a['turn_started'] = None

    def restore_environment(self):
        self.world.set_weather(self.original_weather)
        for actor, state, frozen in self.light_states:
            if actor.is_alive:
                actor.set_state(state)
                actor.freeze(frozen)

    def set_demo_weather(self):
        self.world.set_weather(self.adapter._carla.WeatherParameters.ClearNoon)

    def traffic_step(self):
        """Deterministic NPC intentions, steered with physical VehicleControl.

        Independent start times and smooth heading targets replace instantaneous
        sideways target velocities. No signal state, lane query or contact-based
        displacement is used. The existing 5 m NPC proximity stop is retained.
        """
        carla = self.adapter._carla
        now = self.world.get_snapshot().timestamp.elapsed_seconds
        elapsed = now - self.start
        ego = self.adapter._ego_vehicle.get_location()
        rows = []
        speeds = dict(straight_through=5.0, slow_through=2.5, roadside_edge=3.0,
                      turn_left=4.0, turn_right=4.0, informal_merge=3.5,
                      following_ego_lane=5.5)
        angles = dict(turn_left=-85.0, turn_right=85.0, informal_merge=-40.0)
        for i, a in enumerate(self.traffic):
            actor = self.world.get_actor(a['id'])
            if actor is None:
                raise RuntimeError(f'Required traffic actor {a["id"]} disappeared')
            p = actor.get_location()
            velocity = actor.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            intent = a['intent']
            distance = math.hypot(p.x - 1.10, p.y - 133.72)
            if intent in angles and distance < 16 and a['turn_started'] is None:
                a['turn_started'] = now
            fraction = 0 if a['turn_started'] is None else min(1, (now-a['turn_started'])/5)
            heading = a['initial_heading'] + math.radians(angles.get(intent, 0))*fraction
            yaw = math.radians(actor.get_transform().rotation.yaw)
            error = math.atan2(math.sin(heading-yaw), math.cos(heading-yaw))
            target = speeds[intent]
            if elapsed < i*0.65 or math.hypot(p.x-ego.x, p.y-ego.y) < 5:
                target = 0.0
            # A scheduled slow interval demonstrates independent slowing.
            if intent == 'slow_through' and 12 <= elapsed < 16:
                target = 0.0
            throttle = max(0.0, min(0.6, (target-speed)*0.3))
            brake = max(0.0, min(1.0, (speed-target)*0.4))
            if target == 0:
                brake = 1.0
            actor.apply_control(carla.VehicleControl(
                throttle=throttle, brake=brake,
                steer=max(-0.6, min(0.6, error*1.2))))
            rows.append(dict(id=actor.id, x=p.x, y=p.y, speed=speed,
                             target=target, intent=intent))
        return json.dumps(rows)
