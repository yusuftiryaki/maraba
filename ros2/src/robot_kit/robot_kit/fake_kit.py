"""A pretend orchadOS kit on the socket, for the dev machine and the tests.

    ros2 run robot_kit fake_kit plug [--variant v1|v1-nocam|v2] [--serial K001]
    ros2 run robot_kit fake_kit unplug
    ros2 run robot_kit fake_kit blank          # plugged, chip never written
    ros2 run robot_kit fake_kit gnss  --lat 37.06 --lon 37.38    # a still F9P on a pty
    ros2 run robot_kit fake_kit caster --port 2101               # an NTRIP caster

plug/unplug/blank write the files kit_manager reads with fake_dir:=/tmp/fake_kit
(hw.py). `gnss` is an F9P that never moves (f9p_sim.py) — robot_sim's sim_node
runs one that follows the simulated robot instead (its kit_gnss_link). `caster`
serves syntactically valid RTCM 3 (filler payloads): enough to flip the fake
F9P to RTK, meaningless to a real receiver.

Like fake_esp32, this checks the PLUMBING — the chip format, the mode switch,
the frames, the corrections path — and nothing about real hardware.
"""

from __future__ import annotations

import argparse
import os
import pty
import select
import socket
import threading
import time
import tty

from robot_kit import f9p_sim, hw, manifest as m, rtcm

FAKE_DIR = "/tmp/fake_kit"
GNSS_LINK = "/tmp/fake_kit_gnss"

# The kit's sensors in kit_mount (docs/kit-soketi.md, "montaj plakası"): the
# plate is 0.1825 m off the ground, the antenna ~1.7 m, the camera ~1.6 m and
# the LiDAR ~0.6 m above the ground (orchadOS, 2026-11-20).
GNSS_POSE = m.Pose(0.0, 0.0, 1.52)
CAM_POSE = m.Pose(0.0, 0.0, 1.42)
LIDAR_POSE = m.Pose(0.0, 0.0, 0.42)


def variant(name: str, serial: str = "K001") -> m.Manifest:
    if name == "v1":
        caps = {"gnss_rtk", "pps", "cam360", "cam360_ctrl"}
        frames = {"kit_gnss": GNSS_POSE, "kit_cam360": CAM_POSE}
        link, rev = "usb", (1, 0)
    elif name == "v1-nocam":
        caps = {"gnss_rtk", "pps", "cam360"}
        frames = {"kit_gnss": GNSS_POSE, "kit_cam360": CAM_POSE}
        link, rev = "usb", (1, 0)
    elif name == "v2":
        caps = set(m.CAPS)
        frames = {"kit_gnss": GNSS_POSE, "kit_cam360": CAM_POSE, "kit_lidar": LIDAR_POSE}
        link, rev = "ros", (2, 0)
    else:
        raise ValueError(f"unknown variant {name!r}")
    return m.Manifest(kind="orchados-scan-kit", serial=serial, hw_rev=rev,
                      caps=frozenset(caps), gnss_link=link, calib_id=20261025, frames=frames)


class Pty:
    """A raw pseudo-terminal symlinked at `link` (the same trick as fake_esp32)."""

    def __init__(self, link: str) -> None:
        self.master, self._slave = pty.openpty()
        tty.setraw(self._slave)
        tty.setraw(self.master)
        os.set_blocking(self.master, False)
        self.link = link
        if os.path.islink(link) or os.path.exists(link):
            os.remove(link)
        os.symlink(os.ttyname(self._slave), link)

    def read(self) -> bytes:
        try:
            return os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            return b""

    def write(self, data: bytes) -> None:
        try:
            os.write(self.master, data)
        except (BlockingIOError, OSError):
            pass                     # nobody reading: a real receiver drops it too

    def close(self) -> None:
        if os.path.islink(self.link):
            os.remove(self.link)
        for fd in (self.master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass


def run_gnss(link: str, lat: float, lon: float, alt: float, rate_hz: float, seed: int) -> None:
    port = Pty(link)
    receiver = f9p_sim.F9pSim(seed=seed)
    print(f"fake F9P on {link} at {lat}, {lon}")
    try:
        while True:
            receiver.feed(port.read(), time.time())
            port.write(receiver.frame(time.time(), lat, lon, alt))
            time.sleep(1.0 / rate_hz)
    finally:
        port.close()


def serve_caster(port: int, mountpoint: str, rate_hz: float, chunked: bool,
                 stop: threading.Event | None = None, ready: threading.Event | None = None,
                 log=print) -> None:
    """A minimal NTRIP 2 caster: one mountpoint, no auth, filler RTCM.
    Answers anything else with its source table, as real casters do."""
    server = socket.create_server(("127.0.0.1", port), reuse_port=False)
    server.settimeout(0.5)
    if ready is not None:
        ready.set()
    stop = stop or threading.Event()

    def client(conn: socket.socket) -> None:
        with conn:
            conn.settimeout(5.0)
            try:
                head = conn.recv(4096).decode("latin-1")
            except OSError:
                return
            path = head.split(" ", 2)[1] if head.count(" ") >= 2 else ""
            if path != f"/{mountpoint}":
                conn.sendall(b"SOURCETABLE 200 OK\r\nContent-Type: text/plain\r\n\r\n"
                             b"STR;" + mountpoint.encode() + b";\r\nENDSOURCETABLE\r\n")
                return
            headers = "HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n"
            if chunked:
                headers += "Transfer-Encoding: chunked\r\n"
            conn.sendall((headers + "\r\n").encode())
            conn.setblocking(False)
            heard_gga = False
            while not stop.is_set():
                readable, _, _ = select.select([conn], [], [], 1.0 / rate_hz)
                if readable:
                    try:
                        data = conn.recv(4096)
                    except OSError:
                        return
                    if not data:
                        return
                    heard_gga = heard_gga or b"GGA" in data
                # A VRS mountpoint stays silent until it hears where the rover is.
                if not heard_gga:
                    continue
                payload = rtcm.make_message(1005) + rtcm.make_message(1077, 120)
                if chunked:
                    payload = f"{len(payload):x}\r\n".encode() + payload + b"\r\n"
                try:
                    conn.sendall(payload)
                except OSError:
                    return

    log(f"fake NTRIP caster on 127.0.0.1:{port}/{mountpoint}")
    try:
        while not stop.is_set():
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            threading.Thread(target=client, args=(conn,), daemon=True).start()
    finally:
        server.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", default=FAKE_DIR, help="kit_manager's fake_dir")
    sub = parser.add_subparsers(dest="command", required=True)
    plug = sub.add_parser("plug")
    plug.add_argument("--variant", default="v1", choices=("v1", "v1-nocam", "v2"))
    plug.add_argument("--serial", default="K001")
    sub.add_parser("unplug")
    sub.add_parser("blank")
    gnss = sub.add_parser("gnss")
    gnss.add_argument("--link", default=GNSS_LINK)
    gnss.add_argument("--lat", type=float, default=41.0)
    gnss.add_argument("--lon", type=float, default=29.0)
    gnss.add_argument("--alt", type=float, default=1.7)
    gnss.add_argument("--rate", type=float, default=5.0)
    gnss.add_argument("--seed", type=int, default=0)
    caster = sub.add_parser("caster")
    caster.add_argument("--port", type=int, default=2101)
    caster.add_argument("--mountpoint", default="FAKE")
    caster.add_argument("--rate", type=float, default=1.0)
    caster.add_argument("--chunked", action="store_true")
    args = parser.parse_args()

    if args.command == "plug":
        hw.write_fake(args.dir, True, m.encode(variant(args.variant, args.serial)))
        print(f"plugged {args.variant} {args.serial} into {args.dir}")
    elif args.command == "unplug":
        hw.write_fake(args.dir, False, None)
        print(f"unplugged ({args.dir})")
    elif args.command == "blank":
        hw.write_fake(args.dir, True, b"\xff" * m.EEPROM_SIZE)
        print(f"plugged a kit with a blank chip into {args.dir}")
    elif args.command == "gnss":
        run_gnss(args.link, args.lat, args.lon, args.alt, args.rate, args.seed)
    elif args.command == "caster":
        serve_caster(args.port, args.mountpoint, args.rate, args.chunked)


if __name__ == "__main__":
    main()

