#!/usr/bin/env bash
# cam_check.sh — list connected RealSense cameras (serial + USB) and save one colour frame.
#   slam_tools/cam_check.sh [SERIAL]      -> $KACHAKA_TOOLS_DIR/camcheck.jpg
# SERIAL defaults to KACHAKA_CAMERA_SERIAL from .env. Do not run while a recording holds the camera.
set -euo pipefail
. "$(dirname "$(readlink -f "$0")")/_env.sh"
SERIAL="${1:-${KACHAKA_CAMERA_SERIAL:-}}"
[ -n "$SERIAL" ] && [ "$SERIAL" != "YOUR_REALSENSE_SERIAL" ] || { echo "set KACHAKA_CAMERA_SERIAL in .env or pass SERIAL"; exit 2; }
docker start "$GW_C" >/dev/null
timeout 60 docker exec "$GW_C" python3 -c "
import pyrealsense2 as rs, numpy as np, cv2
for d in rs.context().query_devices():
    print('camera', d.get_info(rs.camera_info.serial_number), 'USB', d.get_info(rs.camera_info.usb_type_descriptor))
p=rs.pipeline(); c=rs.config(); c.enable_device('$SERIAL'); c.enable_stream(rs.stream.color,640,480,rs.format.bgr8,15)
p.start(c)
for _ in range(20): f=p.wait_for_frames()
cv2.imwrite('/fungi/camcheck.jpg', np.asanyarray(f.get_color_frame().get_data())); p.stop()"
echo "saved $FUNGI_HOST/camcheck.jpg  (serial $SERIAL)"
