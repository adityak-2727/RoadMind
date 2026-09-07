function egoState = carlaGetEgoState()
% carlaGetEgoState - Task 6 interface: reads the ego vehicle's current
% state from CARLA and returns it in this project's own egoState schema
% (config/createEgoState.m), via carlaToProjectState.m's documented
% coordinate/unit conversion. Requires carlaSpawnEgoVehicle() first.
%
% Output:
%   egoState - struct matching config/createEgoState.m's schema

session = getCarlaSession();
rawState = session.getRawState();
egoState = carlaToProjectState(rawState);

end
