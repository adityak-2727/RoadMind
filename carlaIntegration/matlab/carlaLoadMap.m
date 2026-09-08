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

% Phase 14 fix (found live, root-caused, not guessed): CARLA's
% client.load_world() blocks until the new map is STRUCTURALLY loaded,
% but its static level geometry (road/building colliders) can still be
% level-streaming in for a few more real seconds after that call
% returns - a documented CARLA behavior, not a bug in this project's
% code. Spawning a dense scene of actors immediately after a reload can
% therefore place them against collision geometry that has not fully
% resolved yet; when it finishes streaming in, overlapping actors get
% violently physics-corrected. Reproduced live: the very first demo run
% against a freshly-booted server (map reload from the default
% Town10HD_Opt to Town03) showed up to 2892 real collision-sensor events
% and speed spikes to 22 m/s in a single run purely from this - the
% IDENTICAL scene/code/actor placement ran with ZERO real collisions
% once re-tested a few seconds later against the same, by-then-settled
% server. A fixed settle delay after a reload (not after every call -
% only when reloaded is actually true, so an already-loaded map incurs
% no extra wait) gives that streaming time to finish before any caller
% spawns actors into the world.
if reloaded
    pause(4.0);
end

end
