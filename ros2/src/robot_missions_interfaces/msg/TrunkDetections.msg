# Tree trunks the kit's 360° camera sees in one frame (docs/kit-soketi.md,
# "Kitin kamerasıyla algı"). One entry per trunk, in header.frame_id — the
# camera, kit_cam360: x forward, y left, z up (a body frame, NOT an optical one).
std_msgs/Header header
# rad, counter-clockwise from the camera's +x, in its horizontal plane
float32[] bearing
# rad below the camera's horizontal to where the trunk meets the ground: the
# range, given the camera's height (range = height / tan(depression)). NaN when
# the base is hidden (grass, another trunk) and only the bearing is known.
float32[] depression
# 0..1, the detector's own score
float32[] confidence
