function carlaDisconnect()
% carlaDisconnect - Task 3/9 interface: destroys the ego actor if present
% and closes the CARLA session cleanly. Safe to call even if never
% connected, and safe to call multiple times.

session = getCarlaSession();
session.disconnect();

end
