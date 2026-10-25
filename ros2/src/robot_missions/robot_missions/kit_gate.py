"""Which missions the kit socket allows, and when a kit loss pauses one. ROS-free.

robot_kit's kit_manager publishes kit/state (docs/kit-soketi.md); mission_runner
reads it through these two functions:

- `unmet(requires, kit)`: a task type that needs the kit (TaskType.requires,
  e.g. a scan needs the RTK receiver and the 360° camera) is refused at start
  — and at resume — with the reason, instead of running a scan without its
  sensors and reporting DONE.
- `lost(before, after, requires)`: while a mission runs, the kit going away
  pauses it. Not only a kit mission: any mission localizing on the kit's GPS
  loses its fixes the moment the plug comes out (gps_selector keeps the silent
  source until the robot is stopped — state.py), so it stops rather than drive
  rows on dead reckoning. The pause cancels the leg; the robot stops; the GPS
  goes back to the NEO-6M; a person decides whether to resume.

No kit/state at all (robot_kit not running, use_kit:=false) is a robot without
a socket: kit missions are refused, everything else is untouched.
"""

from __future__ import annotations

from typing import Optional, Sequence

# requirement -> the kit stream that must be flowing for it (state.py names)
STREAM_OF = {"gnss_rtk": "gnss", "cam360": "camera", "lidar": "lidar"}


def unmet(requires: Sequence[str], kit: Optional[dict]) -> Optional[str]:
    """None when the kit can do what the mission needs; else why not."""
    if not requires:
        return None
    if kit is None:
        return f"this mission needs the kit ({', '.join(requires)}); the kit socket is off"
    if kit.get("state") not in ("READY", "DEGRADED") or not kit.get("kit"):
        why = kit.get("error") or "no kit plugged in"
        return f"this mission needs the kit ({', '.join(requires)}): {why}"
    caps = set(kit["kit"].get("caps", ()))
    absent = [r for r in requires if r not in caps]
    if absent:
        return f"the kit {kit['kit'].get('serial')} has no {', '.join(absent)}"
    missing = set(kit.get("missing", ()))
    silent = [r for r in requires if STREAM_OF.get(r) in missing]
    # A camera recorded by hand (no cam360_ctrl) is not watched, so not missing.
    if silent:
        return f"the kit's {', '.join(silent)} is not working (no {', '.join(missing)} data)"
    if "gnss_rtk" in requires and kit.get("gps_source") != "kit":
        return "the robot is not localizing on the kit's GPS yet (it switches once stopped)"
    return None


def lost(before: Optional[dict], after: Optional[dict], requires: Sequence[str]) -> Optional[str]:
    """Why a running mission must pause on this kit/state change, or None."""
    if before is None or after is None:
        return None
    if before.get("gps_source") == "kit":
        if after.get("state") == "NONE":
            return "the kit was unplugged while the robot localized on its GPS"
        if "gnss" in after.get("missing", ()):
            return "the kit's GPS went silent while the robot localized on it"
    if requires and unmet(requires, before) is None:
        why = unmet(requires, after)
        if why is not None:
            return why
    return None
