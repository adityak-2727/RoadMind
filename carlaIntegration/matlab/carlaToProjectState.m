function egoState = carlaToProjectState(rawState)
% carlaToProjectState - Task 6: converts CARLA's raw ego-vehicle state
% (as returned by carla/python/carla_adapter.py's get_vehicle_state(), via
% CarlaSession.getRawState()) into this project's own egoState schema
% (config/createEgoState.m). This is the ONE place this conversion
% happens - nothing else in the project should reimplement it.
%
% CONVENTIONS (see carla_adapter.py's module docstring for the CARLA-side
% source of truth this was written against):
%   CARLA:   left-handed coordinate system, X forward / Y right / Z up;
%            location in meters; rotation in DEGREES (pitch, yaw, roll);
%            velocity in m/s (vector, x/y/z).
%   Project: right-handed 2D convention (every existing perception/
%            prediction/planning formula uses standard cos(yaw)/sin(yaw)/
%            atan2 assuming yaw increases counter-clockwise from +x - e.g.
%            evaluation/calculateTTC.m's
%            `egoVel = egoState.velocity * [cos(yaw), sin(yaw)]`); x, y
%            in meters; yaw in RADIANS; velocity as a SCALAR longitudinal
%            speed in m/s (config/createEgoState.m's own schema comment -
%            NOT a vector, unlike agent.velocity elsewhere in the project).
%
% TRANSFORM APPLIED:
%   x   = rawState.location.x                    (meters, unchanged)
%   y   = -rawState.location.y                    (sign flip: CARLA's
%         left-handed Y axis becomes this project's right-handed Y axis)
%   yaw = deg2rad(-rawState.rotation_deg.yaw)     (degrees->radians, AND
%         sign-flipped for the same left-to-right-handed reason as y - a
%         left-handed frame's angle must flip sign under a single-axis
%         mirror to stay CCW-positive in the new frame)
%   velocity = hypot(rawState.velocity_mps.x, -rawState.velocity_mps.y)
%         scalar speed = magnitude of the horizontal velocity vector
%         (z/vertical component dropped, matching this project's flat 2D
%         ego-state model everywhere else). Using magnitude (not a signed
%         projection onto heading) is a deliberate, simple, documented
%         choice for this Phase 9 foundation - it drops any lateral-slip
%         information, which this project's own bicycle-model/controller
%         does not model either.
%   z (altitude) is dropped entirely - this project's ego state and every
%     downstream planning/control formula are 2D.
%   timestamp = rawState.timestamp_s (seconds, CARLA's own simulation
%     clock - NOT necessarily aligned with this project's dt=0.1s tick
%     convention in simulationConfig.m; reconciling CARLA's clock with
%     this project's fixed-dt loop is explicitly deferred to a later
%     phase, not solved here).
%
% THIS TRANSFORM IS UNVERIFIED AGAINST A LIVE CARLA SERVER (see
% docs/carla_integration.md) - the sign conventions above follow CARLA's
% documented left-handed axis convention precisely, but no running CARLA
% instance was available in this session to confirm empirically
% end-to-end. Verify against a live server before relying on this for any
% safety-relevant purpose.
%
% Input:
%   rawState - struct as returned by CarlaSession.getRawState() (mirrors
%              carla_adapter.py's get_vehicle_state() dict)
% Output:
%   egoState - struct matching config/createEgoState.m's schema

egoState = createEgoState();
egoState.x = rawState.location.x;
egoState.y = -rawState.location.y;
egoState.yaw = deg2rad(-rawState.rotation_deg.yaw);
egoState.velocity = hypot(rawState.velocity_mps.x, -rawState.velocity_mps.y);
egoState.timestamp = rawState.timestamp_s;
% egoState.steering is left at createEgoState()'s default (0): CARLA's
% raw state does not report the vehicle's current steering angle in the
% same units/convention as this project's control command, and inventing
% a conversion without a live server to verify against is out of scope
% for this Phase 9 foundation.

end
