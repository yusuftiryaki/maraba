"""The camera shim protocol (camera.py) against fake_camera_shim. ROS-free;
spawns the fake shim as a subprocess, exactly as kit_camera spawns the real one."""

import os
import sys

import pytest

from robot_kit.camera import CameraError, FakeCamera, ShimClient

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def shim(*args):
    env_cmd = [sys.executable, "-c",
               "import sys; sys.path.insert(0, %r); from robot_kit.fake_camera_shim import main; "
               "sys.argv[1:] = %r; main()" % (PACKAGE, list(args))]
    return ShimClient(env_cmd, timeout_s=5.0)


def test_connect_record_stop():
    client = shim()
    try:
        assert client.call("connect")["model"] == "fake X5"
        client.call("sync_time", utc_ms=1_800_000_000_000, tz_offset_s=0)
        assert client.call("status")["recording"] is False
        client.call("start", resolution="5.7K30")
        assert client.call("status")["recording"] is True
        with pytest.raises(CameraError, match="already recording"):
            client.call("start", resolution="5.7K30")
        assert client.call("stop")["files"] == ["/DCIM/Camera01/VID_001.insv"]
        pair = client.time_pair(lambda: 1_800_000_000_000)
        assert abs(pair.camera_media - 1_800_000_000_000) < 60_000
    finally:
        client.close()


def test_overheat_stops_recording_and_says_so():
    client = shim("--overheat-after", "0.5")
    try:
        client.call("connect")
        client.call("start", resolution="5.7K30")
        import time
        deadline = time.monotonic() + 5.0
        events = []
        while time.monotonic() < deadline and "capture_stopped" not in events:
            events += client.events()
            time.sleep(0.1)
        assert events[:2] == ["temperature_high", "capture_stopped"]
        assert client.call("status")["recording"] is False
    finally:
        client.close()


def test_a_dead_shim_is_an_error_not_a_hang():
    client = ShimClient([sys.executable, "-c", "pass"], timeout_s=2.0)
    import time
    time.sleep(0.3)
    with pytest.raises(CameraError, match="not running|exited"):
        client.call("status")


def test_fake_camera_fills_its_card():
    camera = FakeCamera(free_bytes=1000, bytes_per_s=1000)
    camera.handle("connect", {}, 0.0)
    camera.handle("start", {}, 0.0)
    camera.tick(0.5)
    assert camera.recording_since is not None
    camera.tick(1.5)
    assert camera.recording_since is None
    assert camera.pending_events == ["storage_full", "capture_stopped"]
