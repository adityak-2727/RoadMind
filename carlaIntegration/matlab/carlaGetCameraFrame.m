function frame = carlaGetCameraFrame()
% carlaGetCameraFrame - Phase 10: retrieves the most recently received
% RGB camera frame. Requires carlaAttachCamera() first.
%
% Output:
%   frame - [] if no frame has arrived yet (sensor just attached / world
%           hasn't ticked since attach - callers must handle this).
%           Otherwise a struct:
%             .image     HxWx3 uint8 RGB image
%             .width, .height, .fov
%             .frame     CARLA simulation frame number the image was
%                        captured on
%             .timestamp CARLA simulation timestamp (seconds) the image
%                        was captured on

session = getCarlaSession();
frame = session.getCameraFrame();

end
