function count = carlaFreezeTrafficLights(x, y, rangeM)
% carlaFreezeTrafficLights - Phase 11.5: freezes every CARLA traffic
% light within rangeM of world position (x,y) to a fixed, non-cycling
% state (yellow/amber - reads as "caution, not actively controlling",
% not a claim of active signal control). Traffic-light infrastructure
% stays visible for realism; nothing in this project's decision stack
% (decision/behaviorDecision.m) reads CARLA traffic-light state at all,
% so right-of-way was never determined by signals regardless - this call
% only prevents a jury from seeing a visually misleading cycling signal.
%
% Inputs:
%   x, y    - world position to search around (meters)
%   rangeM  - optional, default 80. Search radius (meters).
% Output:
%   count - number of traffic lights frozen

if nargin < 3 || isempty(rangeM)
    rangeM = 80.0;
end

session = getCarlaSession();
count = session.freezeTrafficLights(x, y, rangeM);

end
