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
# read_lidar.py
# Standalone example: read the Unitree G1 LiDAR point cloud over DDS.
#
# Interface (see https://support.unitree.com/home/zh/G1_developer/lidar_services_interface):
#   - Enable/disable stream : publish std_msgs/String "ON"/"OFF" to  rt/utlidar/switch
#   - Point cloud topic     : rt/utlidar/cloud   (sensor_msgs/PointCloud2)
#     (other PointCloud2 topics: rt/utlidar/voxel_map, rt/utlidar/height_map,
#      rt/utlidar/range_map ; frame_id is typically "utlidar_lidar")
#
# Requires the Unitree SDK (unitree_sdk2py). This repo ships one at
# ../unitree_sdk2_python; install it or put it on PYTHONPATH:
#     pip install -e /home/ur3-exp/unitree/unitree_sdk2_python
#
# Usage:
#     python read_lidar.py --iface eth0                 # subscribe & print stats
#     python read_lidar.py --iface eth0 --switch ON     # turn the lidar on first
#     python read_lidar.py --iface eth0 --save-npy cloud.npy
#     python read_lidar.py --iface eth0 --topic rt/utlidar/voxel_map
# ------------------------------------------------------------------------------
import argparse
import sys
import time

import numpy as np

try:
    from unitree_sdk2py.core.channel import (
        ChannelSubscriber, ChannelPublisher, ChannelFactoryInitialize,
    )
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
except ImportError as e:
    print(
        "ERROR: unitree_sdk2py not found. Install the Unitree SDK, e.g.:\n"
        "    pip install -e /home/ur3-exp/unitree/unitree_sdk2_python\n"
        "or add it to PYTHONPATH.\n"
        f"(import error: {e})",
        file=sys.stderr,
    )
    sys.exit(1)

# ROS sensor_msgs/PointField datatype codes -> numpy dtypes
_PF_DTYPE = {
    1: np.int8,   2: np.uint8,  3: np.int16,  4: np.uint16,
    5: np.int32,  6: np.uint32, 7: np.float32, 8: np.float64,
}


def pointcloud2_to_columns(msg):
    """Decode a PointCloud2_ into {field_name: 1D np.ndarray} using its field metadata.

    Robust to layout: reads each field at its byte offset with its declared dtype,
    so it works whether the cloud is xyz, xyzi, xyz+ring+time, etc.
    """
    n = int(msg.width) * int(msg.height)
    if n == 0 or msg.point_step == 0:
        return {}, 0
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    usable = n * int(msg.point_step)
    if raw.size < usable:                      # short/truncated buffer -> keep whole rows only
        n = raw.size // int(msg.point_step)
        usable = n * int(msg.point_step)
    raw = raw[:usable].reshape(n, int(msg.point_step))

    cols = {}
    for f in msg.fields:
        dt = _PF_DTYPE.get(int(f.datatype))
        if dt is None:
            continue
        itemsize = np.dtype(dt).itemsize
        seg = raw[:, int(f.offset):int(f.offset) + itemsize].copy()  # contiguous copy
        cols[f.name] = seg.view(dt).reshape(-1)
    return cols, n


class LidarReader:
    def __init__(self, args):
        self.args = args
        self.count = 0
        self._last_log = time.monotonic()

    def handle(self, msg: PointCloud2_):
        self.count += 1
        frame_id = getattr(msg.header, "frame_id", "?")
        field_names = [f.name for f in msg.fields]
        cols, n = pointcloud2_to_columns(msg)

        if n == 0 or not all(k in cols for k in ("x", "y", "z")):
            print(f"[read_lidar] frame#{self.count} points={n} fields={field_names} (no xyz to decode)")
            return

        xyz = np.stack([cols["x"], cols["y"], cols["z"]], axis=1).astype(np.float32)
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        if xyz.shape[0] == 0:
            print(f"[read_lidar] frame#{self.count} points={n} (all non-finite)")
            return

        extra = ""
        if "intensity" in cols:
            inten = cols["intensity"][finite]
            if inten.size:
                extra = f" | intensity[{float(inten.min()):.0f},{float(inten.max()):.0f}]"

        print(
            f"[read_lidar] frame#{self.count} frame_id={frame_id} fields={field_names} "
            f"points={xyz.shape[0]} "
            f"x[{xyz[:,0].min():.2f},{xyz[:,0].max():.2f}] "
            f"y[{xyz[:,1].min():.2f},{xyz[:,1].max():.2f}] "
            f"z[{xyz[:,2].min():.2f},{xyz[:,2].max():.2f}] (m){extra}"
        )

        if self.args.save_npy and self.count == 1:
            np.save(self.args.save_npy, xyz)
            print(f"[read_lidar] saved first cloud ({xyz.shape[0]} pts) -> {self.args.save_npy}")

        if self.args.max_frames and self.count >= self.args.max_frames:
            print(f"[read_lidar] reached --max-frames={self.args.max_frames}, exiting.")
            raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description="Read Unitree G1 LiDAR point cloud over DDS.")
    parser.add_argument("--iface", type=str, default=None,
                        help="Network interface connected to the robot (e.g. eth0). "
                             "Omit to use the default DDS interface.")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id (0 for real robot).")
    parser.add_argument("--topic", type=str, default="rt/utlidar/cloud",
                        help="PointCloud2 topic (default rt/utlidar/cloud; also voxel_map/height_map/range_map).")
    parser.add_argument("--switch", type=str, default="none", choices=["ON", "OFF", "none"],
                        help="Publish ON/OFF to rt/utlidar/switch before subscribing "
                             "(default none: just subscribe). Use ON if no frames arrive.")
    parser.add_argument("--save-npy", type=str, default=None,
                        help="Save the first decoded cloud (Nx3 float32) to this .npy path.")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="Stop after receiving this many frames (0 = run until Ctrl+C).")
    args = parser.parse_args()

    # init DDS
    if args.iface:
        ChannelFactoryInitialize(args.domain, args.iface)
    else:
        ChannelFactoryInitialize(args.domain)

    # optionally enable/disable the lidar stream
    if args.switch in ("ON", "OFF"):
        pub = ChannelPublisher("rt/utlidar/switch", String_)
        pub.Init()
        cmd = std_msgs_msg_dds__String_()
        cmd.data = args.switch
        pub.Write(cmd)
        print(f"[read_lidar] sent switch '{args.switch}' to rt/utlidar/switch")
        time.sleep(0.5)

    reader = LidarReader(args)
    sub = ChannelSubscriber(args.topic, PointCloud2_)
    sub.Init(reader.handle, 10)   # callback mode, queue length 10

    print(f"[read_lidar] subscribed to '{args.topic}' (domain={args.domain}, iface={args.iface}). "
          f"Waiting for frames... Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
            if reader.count == 0:
                print("[read_lidar] no frames yet. If it stays at 0, try '--switch ON' "
                      "and check --iface / that the lidar service is running.")
    except KeyboardInterrupt:
        print(f"\n[read_lidar] stopped. total frames={reader.count}")


if __name__ == "__main__":
    main()
