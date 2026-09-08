function events = carlaGetNewCollisionEvents(sinceIndex)
% carlaGetNewCollisionEvents - Phase 14.11: returns only collision events
% recorded since a previous call's returned count (0-based sinceIndex),
% as a struct array with the same shape as carlaGetCollisionEvents.m.
%
% Exists for safe per-tick incremental logging: carlaGetCollisionEvents.m
% re-marshals the WHOLE, ever-growing event list on every call, which
% Phase 14.5 measured to crawl a run to a halt once the count reached the
% thousands if polled every tick. Call this instead with the count you
% last saw (numel(previousEvents) or a running total), and it returns
% only what is new.
%
% Requires carlaAttachCollisionSensor() first.

session = getCarlaSession();
events = session.getNewCollisionEvents(sinceIndex);

end
