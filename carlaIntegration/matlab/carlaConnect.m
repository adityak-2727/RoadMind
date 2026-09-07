function carlaConnect(cfg)
% carlaConnect - Task 3 interface: establishes (or reuses) a CARLA client
% connection via carla/python/carla_adapter.py. See
% docs/carla_integration.md for setup requirements and
% config/carlaConfig.m for what to configure first. Raises a MATLAB error
% (not a silent no-op) if the carla Python package is missing or the
% server is unreachable.
%
% Input:
%   cfg - optional, struct from config/carlaConfig.m; defaults to
%         carlaConfig() if omitted.

if nargin < 1 || isempty(cfg)
    cfg = carlaConfig();
end

session = getCarlaSession();
session.connect(cfg);

end
