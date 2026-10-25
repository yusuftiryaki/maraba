"""A camera shim with no camera: camera.py's JSON-lines protocol over a
simulated FakeCamera, for the dev machine and the tests.

    ros2 run robot_kit fake_camera_shim [--overheat-after 30]

kit_camera starts it (shim_command) exactly as it would start the real,
SDK-linked shim; nothing on the ROS side can tell the difference — which is
the point, and also all this proves.
"""

from __future__ import annotations

import argparse
import json
import select
import sys
import time

from robot_kit.camera import CameraError, FakeCamera


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--overheat-after", type=float, default=0.0,
                        help="stop recording (too hot) after this many seconds; 0 = never")
    parser.add_argument("--free-gb", type=float, default=400.0)
    args = parser.parse_args()
    camera = FakeCamera(overheat_after_s=args.overheat_after,
                        free_bytes=int(args.free_gb * 1e9))

    def send(message):
        sys.stdout.write(json.dumps(message) + "\n")
        sys.stdout.flush()

    while True:
        readable, _, _ = select.select([sys.stdin], [], [], 0.2)
        camera.tick(time.time())
        for event in camera.pending_events:
            send({"event": event})
        camera.pending_events.clear()
        if not readable:
            continue
        line = sys.stdin.readline()
        if not line:
            return
        try:
            request = json.loads(line)
        except ValueError:
            continue
        ident = request.pop("id", None)
        cmd = request.pop("cmd", "")
        try:
            send({"id": ident, "ok": True, "data": camera.handle(cmd, request, time.time())})
        except (CameraError, KeyError, ValueError) as exc:
            send({"id": ident, "ok": False, "error": str(exc)})


if __name__ == "__main__":
    main()
