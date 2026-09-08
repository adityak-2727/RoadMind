function sensorId = carlaAttachCollisionSensor()
% carlaAttachCollisionSensor - Phase 14: attaches a sensor.other.collision
% to the ego vehicle so real, physics-engine-reported collision events
% (not the planner's predicted/geometric collisionCheck.m result) can be
% checked. Requires carlaSpawnEgoVehicle()/carlaSpawnEgoVehicleAtTransform()
% first.
%
% Why this exists: every collision-avoidance claim through Phase 13 rested
% entirely on collisionCheck.m's predicted TTC evaluation of the planner's
% OWN candidate trajectories - correct for validating decision/planning
% logic, but never cross-checked against whether the real CARLA vehicle
% actually made physical contact with anything. This sensor answers that
% second, independent question. See carlaGetCollisionEvents.m to read the
% accumulated events.

session = getCarlaSession();
sensorId = session.attachCollisionSensor();

end
