function session = getCarlaSession()
% getCarlaSession - returns the single shared CarlaSession handle-object
% for this MATLAB process, creating it on first call. A handle-class
% instance is used (not a plain persistent struct) specifically so
% carlaConnect.m/carlaSpawnEgoVehicle.m/carlaGetEgoState.m/
% carlaApplyControl.m/carlaDisconnect.m - each a separate file/function -
% all operate on the exact same live session.

persistent session_
if isempty(session_)
    session_ = CarlaSession();
end
session = session_;

end
