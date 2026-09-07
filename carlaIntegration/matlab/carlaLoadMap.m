function reloaded = carlaLoadMap(mapName)
% carlaLoadMap - Phase 11.6: loads the named CARLA map if it is not
% already the server's active map. carlaConnect.m/connect() (Phase 9,
% unmodified) never loads a specific map on its own - it only attaches
% to whatever the server already has running. Found necessary during the
% Phase 11.6 audit: without this call, a scene built against a freshly-
% launched server (which defaults to Town10HD_Opt) silently used the
% Indian hero scene's Town03-specific coordinates on the wrong map,
% producing widespread spawn failures.
%
% Input:
%   mapName - CARLA map name (e.g. 'Town03')
% Output:
%   reloaded - true if a reload actually happened, false if the
%              requested map was already active (skips the slow
%              teardown/reload in that case).

session = getCarlaSession();
reloaded = session.loadMap(mapName);

end
