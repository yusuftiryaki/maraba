# Bluetooth Kumandayla Elle Sürüş — Uygulama Planı

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eldeki `VR-PARK` BLE kumandasıyla (fare modu `@+D`) robotu twist_mux üzerinden güvenle sürmek; akış kesilince ≤ 0,25 s'de sıfır komut.

**Architecture:** ROS'suz bir kapı (`joystick_gate.py`, `teleop.py` gibi saat enjekte) bütün güvenlik kurallarını taşır. İnce bir düğüm (`joystick_teleop.py`) olayları evdev'den (gerçek) ya da bir topic'ten (sahte) okuyup kapıya verir, kapının çıktısını `/cmd_vel_joy`'a 20 Hz yayınlar; twist_mux'ta öncelik 110. Web rozeti ve öğrenme etiketleri `joy/state` / `/cmd_vel_joy`'u okur.

**Tech Stack:** ROS 2 Jazzy (rclpy), python3-evdev (yalnız Pi'da, tembel import), BlueZ `busctl`, twist_mux, pytest.

**Spec:** `docs/superpowers/specs/2026-11-20-bluetooth-kumanda-design.md`

## Global Constraints

- Kumanda `/cmd_vel`'e **asla** doğrudan yazmaz; yalnız `/cmd_vel_joy` (twist_mux girişi `joystick`, timeout 0.5, priority 110).
- Alt güvenlik katmanlarına dokunulmaz: twist_mux diğer girişleri/kilitleri, köprü `cmd_timeout` 0.5 s, `max_linear_vel` 1.0 / `max_angular_vel` 1.5, kit ivme rampası, ESP32 firmware, E-stop, tampon vetosu.
- Varsayılanlar (spec'ten aynen): `deadman_s 0.25`, `release_s 2.0`, `arm_idle_s 60.0`, `min_battery_pct 25.0`, `reverse_max 0.2`, `linear_levels [0.2, 0.4, 0.6]`, `angular_levels [0.4, 0.7, 1.0]`, `arm_button "BTN_SIDE"`, `speed_up_button ""`, `speed_down_button ""` (boş = tuş yok; ölçülünce doldurulur), `device_name "VR-PARK"`, `battery_poll_s 30.0`.
- Yalnız `REL_X` / `REL_Y` sürer; `KEY_*` (düğmeler hariç), `ABS_*` asla sürmez.
- Kapının saati **her zaman** `time.monotonic` — `use_sim_time` true olsa bile (ölü adam insanın dünyasında yaşar).
- `robot.launch.py`'de `use_joystick` varsayılanı `"false"`.
- Kod yorumları İngilizce, belgeler Türkçe (repo deseni).
- Çalışma dalı: `feat/bluetooth-kumanda` (main'e doğrudan commit yok).

## Review Focus

1. **Yeniden bağlanınca cihaz yeni `/dev/input/eventN` alır ya da kumanda birden fazla düğüm açar** (klavye + fare ayrı) → kaynak her seferinde adı/MAC'i eşleyen **ve `EV_REL` destekleyen** düğümü seçmeli. Test: Task 2 `test_pick_device_*`.
2. **Yakında ikinci bir `VR-PARK`** → `device_mac` verilmişse yalnız o eşlenir. Test: Task 2 `test_pick_device_respects_mac`.
3. **Sim zamanı / saat sıçraması** → kapı ROS saatini değil `time.monotonic`'i kullanır. Test: Task 1 `test_default_clock_is_monotonic`.
4. **`busctl` yok, adaptör `hci1`, kumandada pil servisi yok** → pil `None`, düğüm çökmez, sürüş serbest. Test: Task 2 `test_bluez_battery_*`.
5. **Kısa radyo boşluğunda çubuk hâlâ itili** → kabul edilen davranış (yeniden etkinleştirmesiz devam) testle sabitlenir ki sessizce değişmesin. Test: Task 1 `test_short_gap_without_disconnect_resumes_while_armed`.

---

### Task 1: ROS'suz kapı — `JoystickGate`

**Files:**
- Create: `ros2/src/robot_missions/robot_missions/joystick_gate.py`
- Test: `ros2/src/robot_missions/test/test_joystick_gate.py`

**Interfaces:**
- Produces:
  ```python
  @dataclass(frozen=True)
  class JoyEvent:
      kind: str      # "rel" | "key" | "other" | "connected" | "battery"
      code: str      # "x"/"y" for rel; evdev key name for key; "" otherwise
      value: float   # rel delta | key 1/0/2 | connected 1/0 | battery % (nan = unknown)

  @dataclass(frozen=True)
  class Buttons:
      arm: str = "BTN_SIDE"
      speed_up: str = ""
      speed_down: str = ""

  class JoystickGate:
      def __init__(self, linear_levels=(0.2, 0.4, 0.6), angular_levels=(0.4, 0.7, 1.0),
                   reverse_max=0.2, deadman_s=0.25, release_s=2.0, arm_idle_s=60.0,
                   min_battery_pct=25.0, invert_x=False, clock=time.monotonic): ...
      def rel(self, axis: str, value: int) -> None
      def other(self) -> None
      def connected(self, up: bool) -> None
      def battery(self, pct: Optional[float]) -> None
      def toggle_arm(self) -> bool            # armed state after the click
      def speed(self, delta: int) -> int      # 0-based level after the change
      def output(self) -> Optional[Tuple[float, float]]   # (m/s, rad/s) or None = publish nothing
      def state(self) -> Dict[str, Any]
  def apply_event(gate: JoystickGate, event: JoyEvent, buttons: Buttons) -> None
  ```
  `state()` anahtarları: `connected, armed, mode_hint, speed_level` (1'den başlar), `battery, rel_age_s, linear, angular, refusal` (son ret/kapatma sebebi, Türkçe kısa metin ya da `None`).

Kurallar (spec "Kurallar" 1–6, builder kararları dahil) — test isimleri bunları birebir sabitler:
- Başlangıç: `connected=False`, `armed=False`, kademe 0, `output() is None`.
- `rel()` her zaman `last_rel_seen`'i günceller; yalnız `armed`'ken eksen zamanını/işaretini kaydeder. Değer 0 → o eksenin bileşeni 0.
- `armed` çıktı: eksen yaşı < `deadman_s` olan bileşen sürer (`y<0` → `+v`, `y>0` → `-min(v, reverse_max)`, `x>0` → `-w`; `invert_x` işareti çevirir). Taze eksen yoksa son `REL`'den `release_s`'e kadar `(0, 0)`, sonra `None`. Etkinleştirildikten sonra hiç `REL` yoksa `None`.
- `toggle_arm()` etkinleştirmeyi **reddeder** (False döner, `refusal` dolar): bağlı değil / pil biliniyor ve `< min_battery_pct` / `now - last_rel_seen < deadman_s`. Kabulde: kademe 0'a, eksen zamanları temizlenir, `armed_at = now`.
- Kapatma (tıklama, `connected(False)`, pil eşiği, `arm_idle_s`): kapatma anında `output()` `None` değilse `now + release_s`'e kadar `(0, 0)`, sonra `None`.
- `arm_idle_s`: `now - max(armed_at, son REL) ≥ arm_idle_s` → kapatma (`output()`/`state()` içinde değerlendirilir).
- `mode_hint == "@+D?"`: bağlı ve son 2 s içinde `other()` gelmiş ve son 2 s içinde `REL` yok; aksi halde `None`.
- `apply_event`: `rel` → `rel()`; `key` ve `value == 1`: arm/speed_up/speed_down düğmesiyse ilgili çağrı, başka tuşsa `other()`; `key` `value` 0/2 → hiçbir şey; `other` → `other()`; `connected` → `connected(value >= 1)`; `battery` → `battery(None if nan else value)`. Boş düğme adı hiçbir tuşla eşleşmez.
- Parametre doğrulama `ValueError`: seviyeler boş/eşit uzunlukta değil/pozitif-sonlu değil, `reverse_max <= 0`, `not 0 < deadman_s < release_s`, `arm_idle_s <= release_s`.

- [ ] **Step 1: Dalı aç**

```bash
git checkout -b feat/bluetooth-kumanda
git add docs/superpowers/specs/2026-11-20-bluetooth-kumanda-design.md docs/superpowers/plans/2026-11-20-bluetooth-kumanda.md
git commit -m "docs: Bluetooth kumanda tasarımı + uygulama planı"
```
(Çalışma ağacındaki ilgisiz değişiklikler — `docs/wiring-map.md`, `.devcontainer/devcontainer.json` — bu commit'lere **eklenmez**.)

- [ ] **Step 2: Başarısız testleri yaz** — `test_joystick_gate.py`, adım saatli (`teleop.py` testlerindeki gibi `Clock` sınıfı: `self.t`, `__call__`, `advance(dt)`). Testler ve iddiaları:

```python
def test_starts_disarmed_disconnected_and_silent(): out is None; s["armed"] is False; s["connected"] is False
def test_rel_while_disarmed_never_drives(): connected; rel("y",-17) ×10 → output() is None
def test_arm_needs_connection(): toggle_arm() is False; state()["refusal"] is not None
def test_arm_refused_while_stick_pushed(): connected; rel("y",-17); advance(0.1); toggle_arm() is False
def test_arm_refused_on_low_battery(): connected; battery(20.0); toggle_arm() is False
def test_unknown_battery_does_not_block(): connected; battery(None); toggle_arm() is True
def test_forward_is_lowest_level_after_arming(): arm; rel("y",-17) → output() == (0.2, 0.0)
def test_reverse_is_capped(): arm; speed(+2); rel("y",17) → output() == (-0.2, 0.0)
def test_right_stick_turns_clockwise_and_invert_flips(): rel("x",17) → (0.0, -0.4); gate(invert_x=True) → (0.0, 0.4)
def test_arc_uses_both_axes(): rel("y",-17); rel("x",-17) → (0.2, 0.4)
def test_stale_axis_drops_out_of_an_arc(): y every 0.03 s, x once; advance past 0.25 → (0.2, 0.0)
def test_silence_gives_zero_then_nothing(): drive; advance(0.25) → (0.0,0.0); advance to 2.0 after last rel → None
def test_disconnect_mid_drive_disarms_with_zero_window(): driving; connected(False) → (0,0); armed False; after 2.0 → None
def test_disarm_click_mid_drive_zero_window(): driving; toggle_arm() is False; output()==(0,0) until 2.0 s
def test_disarm_while_silent_stays_silent(): armed, no rel; toggle_arm() → output() is None
def test_low_battery_mid_drive_disarms(): driving; battery(10.0) → armed False, (0,0)
def test_idle_disarms_after_arm_idle_s(): arm; advance(60.0) → state()["armed"] is False
def test_reconnect_needs_rearm(): connected(False); connected(True); rel("y",-17) → output() is None
def test_short_gap_without_disconnect_resumes_while_armed(): drive; advance(1.0) (no rel, still connected) → (0,0); rel("y",-17) → (0.2, 0.0)
def test_speed_levels_clamp(): speed(+5) == 2; speed(-9) == 0; arm resets to 0
def test_mode_hint_when_only_non_rel_events(): connected; other() → "@+D?"; rel("y",-17) → None
def test_apply_event_press_toggles_release_ignored(): key BTN_SIDE 1 → armed; key BTN_SIDE 0 → still armed
def test_apply_event_unmapped_key_is_other_and_empty_button_matches_nothing(): Buttons(speed_up="") ; key "" 1 → no speed change; key "KEY_PREVIOUSSONG" 1 → mode hint
def test_apply_event_battery_nan_is_unknown(): battery event nan → state()["battery"] is None
def test_bad_parameters_raise(): each invalid combination above → pytest.raises(ValueError)
def test_default_clock_is_monotonic(): JoystickGate()._clock is time.monotonic
```

- [ ] **Step 3: Testlerin düştüğünü gör**

Run: `cd ros2/src/robot_missions && python3 -m pytest test/test_joystick_gate.py -q`
Expected: FAIL — `ModuleNotFoundError: robot_missions.joystick_gate`

- [ ] **Step 4: `joystick_gate.py`'yi yaz** — yukarıdaki arayüz; `threading.Lock` (düğümde okuma iş parçacığı + zamanlayıcı aynı kapıyı çağırır). Modül docstring'i kuralları `teleop.py` gibi tablo halinde anlatır ve "kısa radyo boşluğu" kabulünü yazar.

- [ ] **Step 5: Testlerin geçtiğini gör**

Run: `cd ros2/src/robot_missions && python3 -m pytest test/test_joystick_gate.py -q`
Expected: tümü PASS. Ayrıca `./scripts/ci.sh unit` PASS (ROS'suz toplama hatası yok).

- [ ] **Step 6: Commit**

```bash
git add ros2/src/robot_missions/robot_missions/joystick_gate.py ros2/src/robot_missions/test/test_joystick_gate.py
git commit -m "robot_missions: kumanda kapısı (ROS'suz), Bluetooth kumanda tasarımı adım 1"
```

---

### Task 2: Olay kaynağı + düğüm — `joystick_source.py`, `joystick_teleop.py`

**Files:**
- Create: `ros2/src/robot_missions/robot_missions/joystick_source.py`
- Create: `ros2/src/robot_missions/robot_missions/joystick_teleop.py`
- Modify: `ros2/src/robot_missions/setup.py` (console_scripts: `joystick_teleop = robot_missions.joystick_teleop:main`)
- Test: `ros2/src/robot_missions/test/test_joystick_source.py`

**Interfaces:**
- Consumes: Task 1 `JoyEvent`, `Buttons`, `JoystickGate`, `apply_event`.
- Produces:
  ```python
  # joystick_source.py — ROS-free; `evdev` imported lazily inside EvdevSource only
  EV_KEY, EV_REL, EV_ABS = 1, 2, 3            # linux/input-event-codes.h
  def to_event(ev_type: int, code: int, value: int,
               key_name: Callable[[int], str]) -> Optional[JoyEvent]
  def pick_device(candidates: Sequence[Tuple[str, str, str, bool]],   # (path, name, uniq, has_rel)
                  name: str, mac: str) -> Optional[str]
  def bluez_battery(mac: str, adapter: str = "hci0",
                    run: Callable = subprocess.run) -> Optional[float]
  class EvdevSource:
      def __init__(self, name: str, mac: str, emit: Callable[[JoyEvent], None], retry_s: float = 1.0)
      def start(self) -> None
      def stop(self) -> None
      mac: Optional[str]          # uniq of the opened device, for bluez_battery
  ```
  Düğüm `joystick_teleop`: parametreler Global Constraints'teki adlarla + `fake: false`, `device_mac: ""`; yayın `cmd_vel_joy` (`geometry_msgs/Twist`, 20 Hz, yalnız `output()` `None` değilken), `joy/state` (`std_msgs/String` JSON, 10 Hz); `fake: true` iken `joystick/fake_events` (`std_msgs/String`, JSON `{"kind","code","value"}`) dinler, evdev'e hiç dokunmaz, pil yoklamaz.

Kurallar:
- `to_event`: `EV_REL` code 0 → `rel/"x"`, 1 → `rel/"y"`, diğer REL kodları → `other`; `EV_KEY` → `key/key_name(code)/value`; `EV_ABS` → `other`; diğer türler (`EV_SYN`, `EV_MSC`) → `None`.
- `pick_device`: ad tam eşleşmeli, `mac` boş değilse `uniq` (büyük/küçük harf duyarsız) eşleşmeli, `has_rel` True olmalı; birden çok uyan varsa yol sırasında ilk.
- `bluez_battery`: `busctl get-property org.bluez /org/bluez/<adapter>/dev_<MAC_alt_çizgili> org.bluez.Battery1 Percentage` → çıktı `y 49` → `49.0`. Sıfırdan farklı dönüş kodu, ayrıştırılamayan çıktı, `FileNotFoundError`, zaman aşımı (2 s) → `None`. Asla istisna fırlatmaz.
- `EvdevSource` iş parçacığı: `evdev.list_devices()` → aday listesi → `pick_device`; bulunca `grab()`, `emit(connected 1)`, okuma döngüsü; `OSError`'da `emit(connected 0)`, `retry_s` sonra yeniden tara. Bulunamazsa sessizce yeniden dener (log her 30 s'de bir).
- Düğüm: kapı `clock=time.monotonic` ile kurulur (Review Focus 3); pil `battery_poll_s`'de bir `bluez_battery(source.mac)`, sonuç `JoyEvent("battery","", pct or nan)` olarak kapıya.

- [ ] **Step 1: Başarısız testleri yaz** — `test_joystick_source.py` (evdev'siz, ROS'suz):

```python
def test_to_event_rel_axes(): to_event(2,0,17,_) == JoyEvent("rel","x",17); code 1 → "y"
def test_to_event_wheel_and_abs_are_other(): to_event(2,8,1,_).kind == "other"; to_event(3,16,-1,_).kind == "other"
def test_to_event_key_uses_name(): to_event(1,275,1,lambda c: "BTN_SIDE") == JoyEvent("key","BTN_SIDE",1)
def test_to_event_syn_and_msc_dropped(): to_event(0,0,0,_) is None; to_event(4,4,0x90004,_) is None
def test_pick_device_prefers_the_node_with_rel(): two "VR-PARK" nodes, only second has_rel → second path
def test_pick_device_respects_mac(): mac "7F:AB:6E:93:C4:30" vs candidate uniq "7f:ab:6e:93:c4:31" → None
def test_pick_device_name_must_match(): "VR-PARK2" → None
def test_bluez_battery_parses(): fake run → CompletedProcess(returncode=0, stdout="y 49\n") → 49.0; assert the object path ends "dev_7F_AB_6E_93_C4_30"
def test_bluez_battery_failures_are_none(): returncode 1 → None; stdout "garbage" → None; run raises FileNotFoundError → None; raises subprocess.TimeoutExpired → None
```

- [ ] **Step 2: Düştüğünü gör**

Run: `cd ros2/src/robot_missions && python3 -m pytest test/test_joystick_source.py -q`
Expected: FAIL — modül yok.

- [ ] **Step 3: `joystick_source.py`'yi yaz** (arayüz ve kurallar yukarıda).

- [ ] **Step 4: Geçtiğini gör**

Run: `cd ros2/src/robot_missions && python3 -m pytest test/test_joystick_source.py test/test_joystick_gate.py -q`
Expected: PASS

- [ ] **Step 5: `joystick_teleop.py` düğümünü yaz + `setup.py` girişi.** Gerçek mod yalnız Pi'da çalışır; burada derlenip `fake:=true` ile ayağa kalktığını doğrula:

Run: `cd ros2 && colcon build --symlink-install --packages-select robot_missions && source install/setup.bash && timeout 5 ros2 run robot_missions joystick_teleop --ros-args -p fake:=true; echo rc=$?`
Expected: hata yok, `rc=124` (timeout ile kesildi). Ayrı terminalde `ros2 topic echo /joy/state --once` → `"armed": false`.

- [ ] **Step 6: Commit**

```bash
git add ros2/src/robot_missions/robot_missions/joystick_source.py ros2/src/robot_missions/robot_missions/joystick_teleop.py ros2/src/robot_missions/setup.py ros2/src/robot_missions/test/test_joystick_source.py
git commit -m "robot_missions: kumanda düğümü + evdev/BlueZ kaynağı (sahte mod topic'ten)"
```

---

### Task 3: twist_mux + launch + yığın testi

**Files:**
- Modify: `ros2/src/robot_bringup/config/twist_mux.yaml` (giriş + üstteki ASCII şemaya satır)
- Create: `ros2/src/robot_bringup/config/joystick.yaml` (düğüm parametreleri, Global Constraints değerleri)
- Modify: `ros2/src/robot_bringup/launch/robot.launch.py` (argümanlar `use_joystick` = `"false"`, `joystick_fake` = `"false"`; `web_ui` düğümünün yanına `joystick_teleop` düğümü, `IfCondition(use_joystick)`, parametreler `joystick.yaml` + `{"fake": joystick_fake}`)
- Modify: `ros2/src/robot_sim/test/test_twist_mux.py`
- Modify: `ros2/src/robot_sim/test/conftest.py` (`JoystickStack(WebStack)` + `joystick_stack` fixture; `launch_args` += `use_joystick:=true`, `joystick_fake:=true`)
- Create: `ros2/src/robot_sim/test/test_joystick_stack.py`

**Interfaces:**
- Consumes: Task 2 düğümü, topic'leri ve `joystick/fake_events` JSON biçimi.
- Produces: launch argümanları `use_joystick`, `joystick_fake`; twist_mux girişi `joystick` (`cmd_vel_joy`, 0.5, 110).

- [ ] **Step 1: Başarısız testleri yaz**

`test_twist_mux.py`'ye (mevcut `player` fikstürü + `/cmd_vel_joy` yayıncısı):
```python
def test_joystick_outranks_web_teleop(player): teleop 0.3 and joy 0.2 both at 20 Hz → /cmd_vel linear == 0.2
def test_teleop_gets_the_wheels_back_when_the_joystick_goes_quiet(player): joy stops → within 0.5 s + margin /cmd_vel == teleop's 0.3
```
`test_joystick_stack.py` (sahte olaylar 30 ms'de bir, kendi küçük `JoyWatcher`'ı: `/cmd_vel`, `/cmd_vel_joy`, `/joy/state`, `/ground_truth`):
```python
def test_joystick_drives_and_silence_stops_it(joystick_stack):
    # connected 1, battery 80, BTN_SIDE 1/0, y=-17 every 0.03 s for 3 s
    assert max /cmd_vel linear while driving == pytest.approx(0.2, abs=1e-6)
    assert 0.4 < forward travel (ground truth) < 0.8
    assert first zero on /cmd_vel after the last event < 0.25 + 0.15 s
    assert coast after the last event < 0.12 m
    assert no /cmd_vel_joy later than last event + 2.0 + 0.2 s
def test_disconnect_mid_drive_stops_and_reconnect_needs_rearm(joystick_stack):
    # drive, then connected 0 → zero within 0.15 s, joy/state armed False;
    # connected 1 + y events without a click → /cmd_vel_joy stays silent, robot does not move (< 0.02 m)
def test_arming_with_the_stick_pushed_is_refused(joystick_stack):
    # y events streaming, then BTN_SIDE 1 → joy/state armed False, refusal not None, robot does not move
```

- [ ] **Step 2: Düştüğünü gör**

Run: `./scripts/ci.sh stack` (ya da yalnız: `cd ros2 && colcon build --symlink-install && source install/setup.bash && python3 -m pytest src/robot_sim/test/test_twist_mux.py src/robot_sim/test/test_joystick_stack.py -q`)
Expected: FAIL — `joystick` girişi / launch argümanı yok.

- [ ] **Step 3: twist_mux girişini, `joystick.yaml`'ı, launch argümanlarını ve düğümü ekle.**

- [ ] **Step 4: Geçtiğini gör**

Run: aynı komut.
Expected: yeni testler ve mevcut `test_twist_mux.py` / `test_web_stack.py` PASS; `test_nav2`'nin "`/cmd_vel`'in tek yayıncısı twist_mux" iddiası etkilenmez (kumanda `use_joystick:=false`'ta hiç kalkmaz).

- [ ] **Step 5: Commit**

```bash
git add ros2/src/robot_bringup/config/twist_mux.yaml ros2/src/robot_bringup/config/joystick.yaml ros2/src/robot_bringup/launch/robot.launch.py ros2/src/robot_sim/test/test_twist_mux.py ros2/src/robot_sim/test/conftest.py ros2/src/robot_sim/test/test_joystick_stack.py
git commit -m "bringup: kumanda twist_mux'ta 110, use_joystick launch argümanı + yığın testleri"
```

---

### Task 4: Web rozeti

**Files:**
- Modify: `ros2/src/robot_missions/robot_missions/web_ui.py` (`joy/state` aboneliği; `/api/state` yanıtına `"joystick"`)
- Modify: `ros2/src/robot_missions/robot_missions/web/index.html` (`<div id="joystick" hidden></div>`, `#kit`'in yanında)
- Modify: `ros2/src/robot_missions/robot_missions/web/app.js` (`showJoystick(j)`, `showKit` yanında, `s.joystick` ile çağrılır)
- Modify: `ros2/src/robot_missions/robot_missions/web/style.css` (rozet renkleri: kapalı / etkin / uyarı)
- Test: `ros2/src/robot_sim/test/test_joystick_stack.py`

**Interfaces:**
- Consumes: `joy/state` JSON (Task 1 `state()` anahtarları).
- Produces: `/api/state` → `"joystick": <son joy/state dict> | null` (son mesaj 2 s'den eskiyse `null`).

Rozet metinleri: bağlı değil → `🎮 bağlı değil`; bağlı, kapalı → `🎮 kapalı`; etkin → `🎮 etkin · kademe N`; pil biliniyorsa sona ` · pil %P` (`< 25` ise uyarı rengi). `title`: `refusal` ve `mode_hint` (`"@+D?"` → `"kumanda fare modunda değil (@ + D)"`). `joystick` `null` ise rozet gizli.

- [ ] **Step 1: Başarısız test** — `test_joystick_drives_and_silence_stops_it`'e ekle: sürerken `http("GET","/api/state")["joystick"]["armed"] is True` ve `["speed_level"] == 1`.
- [ ] **Step 2: Düştüğünü gör** — Run: `python3 -m pytest src/robot_sim/test/test_joystick_stack.py -q` → FAIL (`KeyError: 'joystick'`).
- [ ] **Step 3: `web_ui.py`, `index.html`, `app.js`, `style.css` değişiklikleri.**
- [ ] **Step 4: Geçtiğini gör** — aynı komut PASS; `./scripts/sim_demo.sh` + `ros2 run robot_missions joystick_teleop --ros-args -p fake:=true` ile sayfada rozetin `🎮 bağlı değil` gösterdiğini gözle kontrol et.
- [ ] **Step 5: Commit**

```bash
git add ros2/src/robot_missions/robot_missions/web_ui.py ros2/src/robot_missions/robot_missions/web/ ros2/src/robot_sim/test/test_joystick_stack.py
git commit -m "web: başlıkta kumanda rozeti (bağlı / etkin / kademe / pil)"
```

---

### Task 5: Öğrenme etiketleri — kumanda insan sürüşüdür

**Files:**
- Modify: `ros2/src/robot_learning/robot_learning/dataset.py` (`SOURCES = ("none", "teleop", "nav", "joystick")` — **sona** eklenir, eski indeksler korunur; docstring)
- Modify: `ros2/src/robot_learning/robot_learning/export.py` (`EXTRACT["/cmd_vel_joy"] = ("joystick", lambda m: (m.linear.x, m.angular.z))`)
- Modify: `ros2/src/robot_bringup/config/learning_topics.yaml` (`/cmd_vel_joy`, `/joy/state`)
- Test: `ros2/src/robot_learning/test/test_dataset.py`, `ros2/src/robot_learning/test/test_episode.py:133`

`build_frames` öncelik sırası: `joystick` taze → `source = 3`, `teleop_held = True`; değilse mevcut `teleop` (1) → `nav` (2) sırası aynen.

- [ ] **Step 1: Başarısız testler**

```python
def test_joystick_frames_are_a_person_driving(): series {"joystick": [(0.0, 0.2, 0.0), ...]} → source == SOURCES.index("joystick") == 3; teleop flag True; action == (0.2, 0.0)
def test_joystick_wins_over_web_teleop_when_both_fresh(): both fresh → source 3, action from joystick
def test_old_source_indices_unchanged(): SOURCES[:3] == ("none", "teleop", "nav")
```
`test_episode.py:133` listesine `/cmd_vel_joy`, `/joy/state`.

- [ ] **Step 2: Düştüğünü gör** — Run: `cd ros2/src/robot_learning && python3 -m pytest test -q` → FAIL.
- [ ] **Step 3: `dataset.py`, `export.py`, `learning_topics.yaml` değişiklikleri.**
- [ ] **Step 4: Geçtiğini gör** — aynı komut PASS; `./scripts/ci.sh unit` PASS.
- [ ] **Step 5: Commit**

```bash
git add ros2/src/robot_learning/ ros2/src/robot_bringup/config/learning_topics.yaml
git commit -m "robot_learning: kumandayla sürüş insan sürüşü olarak etiketlenir"
```

---

### Task 6: Belgeler + saha kontrol listesi

**Files:**
- Modify: `docs/deployment.md` — yeni "Adım 10 — Bluetooth kumanda": `sudo apt install bluez python3-evdev`; `sudo usermod -aG input enes` (servis kullanıcısı, `deploy/robot.service`); `bluetoothctl` `power on / agent on / scan on / pair / trust / connect`; `robot.service` `ExecStart`'a `use_joystick:=true`; her açılışta `@ + D`.
- Modify: `docs/missions.md` — "5. aşama" altına `### 5a-ek: Bluetooth kumanda (2026-11-20)`: kurallar özeti (spec'e bağlantı), tuşlar, rozet, ölçülenler tablosu (spec'ten), **saha kontrol listesi** (aşağıda).
- Modify: `docs/handoff.md` — "ŞU AN NEREDEYIZ"a kısa giriş + açık işler: tuş kodları, mod hafızası, saha durma mesafesi.

Saha kontrol listesi (missions.md'ye, işaretlenecek kutular olarak):
1. Fare modunda `A B C D` ve iki tetiğin `evtest` kodları → `joystick.yaml` `speed_up_button` / `speed_down_button`.
2. Kumanda kapat-aç → hâlâ `@ + D` mı? Uykudan uyanınca?
3. Tekerlekler havada: etkinleştir, ileri; kumandayı **kapat** → tekerlek ≤ 0,25 s + köprü içinde duruyor.
4. Yerde, en yüksek kademe (0,6 m/s): kumandayı kapat ve ayrıca menzil dışına yürü → durma mesafesi ölç (hedef < 0,25 m).
5. Wi-Fi açıkken menzil (Pi 4'te BT ile aynı çip).
6. Pil `%25` altında etkinleştirme reddi (rozette görünür).

- [ ] **Step 1: Üç belgeyi yaz.**
- [ ] **Step 2: Bağlantıları kontrol et** — Run: `grep -n "2026-11-20-bluetooth-kumanda" docs/missions.md docs/handoff.md` → iki eşleşme.
- [ ] **Step 3: Tam doğrulama** — Run: `./scripts/ci.sh unit stack` → PASS (atlama yok, CLAUDE.md: skip = hata).
- [ ] **Step 4: Commit**

```bash
git add docs/deployment.md docs/missions.md docs/handoff.md
git commit -m "docs: Bluetooth kumanda — Pi kurulumu, saha kontrol listesi, handoff"
```
