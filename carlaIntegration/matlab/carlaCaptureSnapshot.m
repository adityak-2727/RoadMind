function frame = carlaCaptureSnapshot(x, y, z, yawDeg, pitchDeg, width, height, fov)
% carlaCaptureSnapshot - Phase 11.5: captures one RGB frame from a
% temporary camera at an explicit world transform (e.g. an elevated
% bird's-eye view of the hero scene), independent of the ego-mounted
% camera slot (carlaAttachCamera.m/carlaGetCameraFrame.m). Evidence
% capture only - not part of the production sensor path.
%
% Inputs:
%   x, y, z    - world position (meters)
%   yawDeg     - camera heading (degrees)
%   pitchDeg   - optional, default -30 (looking down)
%   width,height - optional, default 1280x800
%   fov        - optional, default 90
% Output:
%   frame - struct with .image (HxWx3 uint8), .width, .height; or []
%           if no frame arrived within the adapter's timeout.

if nargin < 5 || isempty(pitchDeg); pitchDeg = -30.0; end
if nargin < 6 || isempty(width); width = 1280; end
if nargin < 7 || isempty(height); height = 800; end
if nargin < 8 || isempty(fov); fov = 90.0; end

session = getCarlaSession();
frame = session.captureSnapshot(x, y, z, yawDeg, pitchDeg, width, height, fov);

end
