"""The identity chip's format (manifest.py). ROS-free.

test_vectors_are_the_contract is the one that matters across repos: the
orchadOS writer runs the same vectors (docs/kit-soketi.md, "Repolar arası
sözleşme"). If it fails after a change here, the change broke every chip
already written.
"""

import json
import math
import os

import pytest

from robot_kit import manifest as m

VECTORS = os.path.join(os.path.dirname(__file__), "vectors", "kit_manifest_v1.json")


def v1_kit(**changes):
    base = dict(
        kind="orchados-scan-kit", serial="K001", hw_rev=(1, 0),
        caps=frozenset({"gnss_rtk", "pps", "cam360", "cam360_ctrl"}), gnss_link="usb",
        calib_id=20261025,
        frames={"kit_gnss": m.Pose(0.0, 0.0, 1.92),
                "kit_cam360": m.Pose(0.0, 0.0, 1.82, 0.0, 0.0, math.pi / 2)})
    base.update(changes)
    return m.Manifest(**base)


def test_vectors_are_the_contract():
    with open(VECTORS, encoding="utf-8") as handle:
        vectors = json.load(handle)
    assert vectors, "no vectors"
    for vector in vectors:
        manifest = m.from_dict(vector["manifest"])
        image = bytes.fromhex(vector["image_hex"])
        assert len(image) == m.EEPROM_SIZE
        assert m.encode(manifest) == image, vector["name"]
        assert m.to_dict(m.decode(image)) == m.to_dict(manifest), vector["name"]


def test_round_trip():
    kit = v1_kit()
    back = m.decode(m.encode(kit))
    assert back.serial == "K001" and back.hw_rev == (1, 0)
    assert back.caps == kit.caps and back.gnss_link == "usb"
    assert back.frames["kit_cam360"].yaw == pytest.approx(math.pi / 2, abs=1e-6)
    assert back.frames["kit_gnss"].z == pytest.approx(1.92, abs=1e-6)


def test_image_fits_the_chip_with_room_for_a_lidar():
    image = m.encode(v1_kit(
        caps=frozenset(m.CAPS), gnss_link="ros",
        frames={"kit_gnss": m.Pose(0, 0, 1.92), "kit_cam360": m.Pose(0, 0, 1.82),
                "kit_lidar": m.Pose(0.05, 0, 1.02, 0, 0.1, 0)}))
    used = len(image.rstrip(b"\xff"))
    assert used <= 102 < m.EEPROM_SIZE
    assert m.decode(image).has("lidar")


@pytest.mark.parametrize("damage, why", [
    (lambda b: b"\xff" * m.EEPROM_SIZE, "blank"),
    (lambda b: b"XKIT" + b[4:], "magic"),
    (lambda b: b[:4] + bytes([2]) + b[5:], "schema 2"),
    (lambda b: b[:30] + bytes([b[30] ^ 0x01]) + b[31:], "CRC"),
    (lambda b: b[:10], "bytes"),
])
def test_damaged_chips_are_refused(damage, why):
    with pytest.raises(m.ManifestError, match=why):
        m.decode(damage(m.encode(v1_kit())))


def test_newer_schema_is_refused_not_guessed():
    image = bytearray(m.encode(v1_kit()))
    image[4] = 2
    end = 6 + image[5]
    image[end:end + 2] = m.crc16_ccitt(bytes(image[:end])).to_bytes(2, "little")
    with pytest.raises(m.ManifestError, match="schema 2; this robot reads only 1"):
        m.decode(bytes(image))


@pytest.mark.parametrize("changes, why", [
    ({"caps": frozenset({"lidar"}), "frames": {}}, "kit_lidar has no calibration"),
    ({"gnss_link": "none"}, "gnss_link is 'none'"),
    ({"caps": frozenset({"cam360_ctrl"}), "frames": {}}, "cam360_ctrl without cam360"),
    ({"frames": {"kit_gnss": m.Pose(0, 0, 19.2), "kit_cam360": m.Pose(0, 0, 1.8)}},
     "more than 5.0 m"),
    ({"frames": {"kit_gnss": m.Pose(0, 0, float("nan")), "kit_cam360": m.Pose(0, 0, 1.8)}},
     "non-finite"),
    ({"serial": "TOO-LONG-1"}, "serial"),
    ({"caps": frozenset({"gnss_rtk", "sonar"})}, "unknown capabilities"),
])
def test_invalid_manifests_are_not_written(changes, why):
    with pytest.raises(m.ManifestError, match=why):
        m.encode(v1_kit(**changes))


def test_crc_reference_value():
    # CRC-16/CCITT-FALSE check value from the catalogue of parametrised CRCs.
    assert m.crc16_ccitt(b"123456789") == 0x29B1
