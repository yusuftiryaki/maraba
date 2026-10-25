"""kit_gate.py: which missions the kit socket allows, and when a kit loss
pauses one (docs/kit-soketi.md). ROS-free."""

from robot_missions import kit_gate

SCAN = ("gnss_rtk", "cam360")


def kit_state(state="READY", caps=("gnss_rtk", "pps", "cam360", "cam360_ctrl"),
              missing=(), source="kit", error=""):
    kit = {"serial": "K001", "caps": list(caps)} if state != "NONE" else None
    return {"state": state, "kit": kit, "missing": list(missing), "gps_source": source,
            "error": error}


def test_missions_that_need_nothing_run_anywhere():
    assert kit_gate.unmet((), None) is None
    assert kit_gate.unmet((), kit_state("NONE", source="neo6m")) is None


def test_a_scan_needs_a_working_kit_on_its_own_gps():
    assert kit_gate.unmet(SCAN, kit_state()) is None
    assert "socket is off" in kit_gate.unmet(SCAN, None)
    assert "no kit plugged in" in kit_gate.unmet(SCAN, kit_state("NONE", source="neo6m"))
    assert "blank" in kit_gate.unmet(SCAN, kit_state("NONE", source="neo6m",
                                                     error="the chip is blank"))
    assert "has no cam360" in kit_gate.unmet(SCAN, kit_state(caps=("gnss_rtk",)))
    assert "gnss_rtk is not working" in kit_gate.unmet(
        SCAN, kit_state("DEGRADED", missing=("gnss",)))
    assert "not localizing on the kit's GPS" in kit_gate.unmet(SCAN, kit_state(source="neo6m"))


def test_a_camera_recorded_by_hand_is_not_a_missing_camera():
    hand = kit_state(caps=("gnss_rtk", "pps", "cam360"))
    assert kit_gate.unmet(SCAN, hand) is None
    assert "cam360 is not working" in kit_gate.unmet(
        SCAN, kit_state("DEGRADED", missing=("camera",)))


def test_unplugged_on_the_kit_gps_pauses_any_mission():
    assert "unplugged" in kit_gate.lost(kit_state(), kit_state("NONE", source="kit"), ())
    assert "went silent" in kit_gate.lost(kit_state(),
                                          kit_state("DEGRADED", missing=("gnss",)), ())


def test_unplugged_on_the_neo6m_does_not_pause_a_mowing_mission():
    on_neo = kit_state(source="neo6m")
    assert kit_gate.lost(on_neo, kit_state("NONE", source="neo6m"), ()) is None


def test_a_scan_pauses_when_its_camera_goes():
    assert "cam360 is not working" in kit_gate.lost(
        kit_state(), kit_state("DEGRADED", missing=("camera",)), SCAN)
    assert kit_gate.lost(kit_state(), kit_state(), SCAN) is None


def test_no_previous_state_never_pauses():
    assert kit_gate.lost(None, kit_state("NONE"), SCAN) is None
