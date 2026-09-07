function available = isCarlaAvailable(cfg)
% isCarlaAvailable - Phase 9 helper: a quick, non-throwing check for
% whether a CARLA server is actually reachable right now. Used by
% carla/tests/testCarlaIntegration.m to SKIP (not fail, and never fake a
% pass for) the live-CARLA integration tests when no server is reachable
% in this environment - the honest, expected outcome on a machine with no
% CARLA installation.
%
% Input:
%   cfg - optional, struct from config/carlaConfig.m; defaults to
%         carlaConfig() if omitted, with its timeout shortened here for a
%         fast check.
% Output:
%   available - logical

if nargin < 1 || isempty(cfg)
    cfg = carlaConfig();
end
cfg.timeoutSeconds = min(cfg.timeoutSeconds, 3.0); % keep the check fast regardless of the configured default

try
    carlaConnect(cfg);
    available = true;
catch
    available = false;
end

end
