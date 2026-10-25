"""Kit mode (state.py) and the socket's sense lines (hw.py). ROS-free."""

import pytest

from robot_kit import hw, manifest as m, state as s


def kit(caps=("gnss_rtk", "pps", "cam360", "cam360_ctrl")):
    frames = {"kit_gnss": m.Pose(0, 0, 1.92), "kit_cam360": m.Pose(0, 0, 1.82)}
    return m.Manifest(kind="orchados-scan-kit", serial="K001", hw_rev=(1, 0),
                      caps=frozenset(caps), gnss_link="usb", calib_id=1, frames=frames)


def plug(monitor, t, manifest=None):
    """Plug in at t, past the debounce, chip read as asked."""
    monitor.presence(True, t)
    status = monitor.tick(t + 0.6, idle=True)
    assert status.read_chip
    monitor.chip(manifest or kit(), "", t + 0.6)
    return t + 0.6


def test_nothing_plugged_is_none_on_the_neo6m():
    status = s.KitMonitor().tick(0.0, idle=True)
    assert (status.state, status.gps_source, status.present) == (s.NONE, s.NEO6M, False)


def test_contact_bounce_is_ignored():
    monitor = s.KitMonitor(debounce_s=0.5)
    for i in range(10):                        # a connector being screwed on
        monitor.presence(i % 2 == 0, i * 0.1)
        assert not monitor.tick(i * 0.1, idle=True).present
    monitor.presence(True, 1.0)
    assert not monitor.tick(1.4, idle=True).present
    assert monitor.tick(1.51, idle=True).present


def test_attach_streams_then_ready_and_gps_moves_to_the_kit():
    monitor = s.KitMonitor()
    t = plug(monitor, 0.0)
    status = monitor.tick(t, idle=True)
    assert status.state == s.READY and "attached orchados-scan-kit K001" in status.events
    # no fix yet: the GPS stays where it is
    assert status.gps_source == s.NEO6M
    monitor.seen("gnss", t + 1)
    monitor.seen("camera", t + 1)
    status = monitor.tick(t + 1.2, idle=True)
    assert status.gps_source == s.KIT and "gps source neo6m -> kit" in status.events


def test_gps_source_waits_for_idle():
    monitor = s.KitMonitor()
    t = plug(monitor, 0.0)
    monitor.seen("gnss", t + 1)
    moving = monitor.tick(t + 1.1, idle=False)
    assert (moving.gps_source, moving.gps_wanted) == (s.NEO6M, s.KIT)
    monitor.seen("gnss", t + 2)
    assert monitor.tick(t + 2.1, idle=True).gps_source == s.KIT


def test_silent_streams_degrade_after_the_grace_period():
    monitor = s.KitMonitor(grace_s=15.0)
    t = plug(monitor, 0.0)
    monitor.seen("gnss", t + 1)
    assert monitor.tick(t + 10, idle=True).state == s.DEGRADED   # gnss went quiet at t+3
    monitor.seen("gnss", t + 10)
    status = monitor.tick(t + 10.1, idle=True)
    assert status.state == s.READY, "camera still inside its grace"
    monitor.seen("gnss", t + 16)
    status = monitor.tick(t + 16, idle=True)
    assert status.state == s.DEGRADED and status.missing == ("camera",)
    assert status.gps_source == s.KIT, "a silent camera does not cost the GPS"


def test_a_kit_without_camera_control_does_not_expect_the_camera():
    monitor = s.KitMonitor()
    t = plug(monitor, 0.0, kit(caps=("gnss_rtk", "pps", "cam360")))
    monitor.seen("gnss", t + 30)
    assert monitor.tick(t + 30, idle=True).state == s.READY


def test_unplugged_mid_mission_the_source_waits_for_the_stop():
    monitor = s.KitMonitor()
    t = plug(monitor, 0.0)
    monitor.seen("gnss", t + 1)
    assert monitor.tick(t + 1, idle=True).gps_source == s.KIT
    monitor.presence(False, t + 5)
    status = monitor.tick(t + 5.6, idle=False)
    assert status.state == s.NONE and "detached K001" in status.events
    assert (status.gps_source, status.gps_wanted) == (s.KIT, s.NEO6M)
    assert monitor.tick(t + 7, idle=True).gps_source == s.NEO6M


def test_a_bad_chip_keeps_none_and_is_retried():
    monitor = s.KitMonitor(chip_retry_s=2.0)
    monitor.presence(True, 0.0)
    assert monitor.tick(0.6, idle=True).read_chip
    monitor.chip(None, "CRC mismatch", 0.6)
    status = monitor.tick(1.0, idle=True)
    assert (status.state, status.error, status.read_chip) == (s.NONE, "CRC mismatch", False)
    assert "chip: CRC mismatch" in status.events
    assert monitor.tick(2.7, idle=True).read_chip
    monitor.chip(kit(), "", 2.7)
    assert monitor.tick(2.8, idle=True).state == s.READY


def test_replug_starts_over():
    monitor = s.KitMonitor()
    t = plug(monitor, 0.0)
    monitor.seen("gnss", t + 1)
    monitor.tick(t + 1, idle=True)
    monitor.presence(False, t + 2)
    monitor.tick(t + 3, idle=True)
    monitor.presence(True, t + 4)
    status = monitor.tick(t + 4.6, idle=True)
    assert status.state == s.NONE and status.read_chip, "a re-plugged kit is read again"


# --- hw.py ---------------------------------------------------------------------

def test_fake_readers(tmp_path):
    fake = str(tmp_path / "kit")
    presence, eeprom = hw.make_readers(fake)
    assert presence.read() is False
    with pytest.raises(hw.ChipReadError):
        eeprom.read()
    image = m.encode(kit())
    hw.write_fake(fake, True, image)
    assert presence.read() is True and m.decode(eeprom.read()).serial == "K001"
    hw.write_fake(fake, False, None)
    assert presence.read() is False


def test_w1_reader(tmp_path):
    reader = hw.W1Eeprom(str(tmp_path / "2d-*" / "eeprom"))
    with pytest.raises(hw.ChipReadError, match="no DS2431"):
        reader.read()
    for name in ("2d-0000aaaa", "2d-0000bbbb"):
        (tmp_path / name).mkdir()
        (tmp_path / name / "eeprom").write_bytes(m.encode(kit()))
    with pytest.raises(hw.ChipReadError, match="2 DS2431"):
        reader.read()
    (tmp_path / "2d-0000bbbb" / "eeprom").unlink()
    (tmp_path / "2d-0000bbbb").rmdir()
    assert m.decode(reader.read()).serial == "K001"
