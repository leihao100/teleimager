# Copyright 2025-2026 YuShu TECHNOLOGY CO.,LTD ("Unitree Robotics")
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ------------------------------------------------------------------------------
# read_depth.py
# Standalone example: subscribe to the depth stream published by a
# teleimager-test server and read/inspect the depth values.
#
# The server publishes aligned depth as raw uint16 (z16) bytes on a dedicated
# ZMQ port (see enable_depth / depth_port in cam_config_server.yaml). The
# client reshapes those bytes into an HxW uint16 array; a pixel value of 0
# means "no depth", and each unit is `--depth-scale` meters (D400 default
# 0.001 m == 1 mm).
#
# Usage (after `pip install -e .`):
#     teleimager-test-depth --host <server-ip>
#     teleimager-test-depth --host <server-ip> --camera head --show
#   or:
#     python -m teleimager_test.read_depth --host <server-ip>
# ------------------------------------------------------------------------------
import argparse
import time

import numpy as np
import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

from .image_client import ImageClient

# camera key -> (client depth getter, cam_config topic)
_CAMERAS = {
    "head":        ("get_head_depth_frame",        "head_camera"),
    "left_wrist":  ("get_left_wrist_depth_frame",  "left_wrist_camera"),
    "right_wrist": ("get_right_wrist_depth_frame", "right_wrist_camera"),
}


def main():
    parser = argparse.ArgumentParser(
        description="Read RealSense depth published by a teleimager-test server."
    )
    parser.add_argument("--host", type=str, default="192.168.123.164",
                        help="IP address of the image server")
    parser.add_argument("--camera", type=str, default="head", choices=list(_CAMERAS),
                        help="Which camera's depth to read (default: head)")
    parser.add_argument("--depth-scale", type=float, default=0.001,
                        help="Meters per depth unit (D400 default 0.001 = 1mm)")
    parser.add_argument("--show", action="store_true",
                        help="Show a colorized depth window (requires a display)")
    args = parser.parse_args()

    getter_name, topic = _CAMERAS[args.camera]

    client = ImageClient(host=args.host)          # depth needs no BGR decoding
    cam_config = client.get_cam_config()
    cfg = cam_config.get(topic, {})

    if not cfg.get("enable_depth", False) or cfg.get("depth_port") is None:
        logger_mp.error(
            f"[read_depth] '{topic}' is not publishing depth "
            f"(enable_depth={cfg.get('enable_depth')}, depth_port={cfg.get('depth_port')}). "
            f"Enable it in cam_config_server.yaml (RealSense only) and restart the server with --rs."
        )
        client.close()
        return

    getter = getattr(client, getter_name)
    logger_mp.info(
        f"[read_depth] reading depth from '{topic}' @ {args.host} "
        f"(depth_port={cfg.get('depth_port')}, image_shape={cfg.get('image_shape')})"
    )

    cv2 = None
    if args.show:
        import cv2 as _cv2
        cv2 = _cv2

    try:
        while True:
            depth = getter()   # HxW uint16, 0 == invalid/no-return
            if depth is None:
                logger_mp.warning("[read_depth] no depth frame yet (is the server running with --rs?)")
                time.sleep(0.05)
                continue

            valid = depth[depth > 0]
            cy, cx = depth.shape[0] // 2, depth.shape[1] // 2
            center = int(depth[cy, cx])

            if valid.size == 0:
                logger_mp.info(f"[read_depth] {depth.shape} received, but all pixels invalid (0).")
            else:
                logger_mp.info(
                    f"[read_depth] {depth.shape} {depth.dtype} | "
                    f"center({cx},{cy})={center} ({center * args.depth_scale:.3f} m) | "
                    f"valid min/max={int(valid.min())}/{int(valid.max())} "
                    f"({valid.min() * args.depth_scale:.3f}/{valid.max() * args.depth_scale:.3f} m) | "
                    f"coverage={valid.size / depth.size * 100:.1f}%"
                )

            if cv2 is not None:
                vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow(f"{args.camera} depth", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.03)
    except KeyboardInterrupt:
        logger_mp.info("[read_depth] stopped by user.")
    finally:
        client.close()
        if cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
