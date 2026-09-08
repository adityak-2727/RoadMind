function n = carlaGetCollisionCount()
% carlaGetCollisionCount - Phase 14.5: cheap count of real collision
% events recorded so far, as a plain integer.
%
% Use this for per-tick progress/metrics. carlaGetCollisionEvents.m
% marshals every recorded event across the Python/MATLAB boundary on
% every call, so polling THAT once per tick costs O(n^2) in the number of
% events - measured live during Phase 14.5 to slow a maneuver to a crawl
% once the count reached the thousands. Fetch the full event list once,
% after the run.
%
% Requires carlaAttachCollisionSensor() first (raises otherwise, so a
% caller cannot mistake "never attached" for "zero collisions").

session = getCarlaSession();
n = session.getCollisionCount();

end
