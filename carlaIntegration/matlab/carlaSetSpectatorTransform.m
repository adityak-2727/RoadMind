function carlaSetSpectatorTransform(x, y, z, pitchDeg, yawDeg, rollDeg)
% carlaSetSpectatorTransform - EMERGENCY DEMO BRIDGE: points CARLA's
% free-fly spectator camera (the window the evaluator sees) at an
% explicit world transform. Moves only the spectator - never an actor,
% never the ego.
%
% Inputs: x,y,z world position (m); pitchDeg,yawDeg,rollDeg orientation (deg)

session = getCarlaSession();
session.setSpectatorTransform(x, y, z, pitchDeg, yawDeg, rollDeg);

end
