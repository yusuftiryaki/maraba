# Bluetooth kumandayla elle sürüş — tasarım

> Durum: **kullanıcı onayladı (2026-11-20).** Builder kontrolünden
> geçti; çıkan üç soru bu belgede "karar" olarak işlendi.
> Kumanda ölçümleri kullanıcının host'unda `evtest` / `btmon` ile yapıldı
> (2026-11-20, bu belgede "Ölçülen" başlığı altında).

## Amaç ve neden

Bahçede **envanter taraması elle**: operatör robotun yanında yürür, robotu
eldeki Bluetooth kumandayla sürer; robotun üstündeki orchadOS kiti sırttaki /
el arabasındaki taramanın aynısını yapar. Robotla otonom envanter ertelendi
çünkü robot gövdeleri canlı göremiyor (kit-soketi.md "Envanter sırası");
gövdeleri insan görünce bu engel kalkar.

Kayıt tarafında robota iş yok: orchadOS'ta oturumu **telefon uygulaması**
başlatır/durdurur (kitin kendi Pi'ı, BLE), 360° kamera elle başlatılır, saat
QR ile eşlenir (orchadOS `docs/tarama-kiti.md:169`, `docs/arayuzler.md:173`).
`scan_platform = robot` orchadOS'ta zaten var. Telefon robottaki tutucuda,
operatörün elinde yalnız kumanda kalır — kumandanın asıl kazancı bu.

Web sayfasından sürüş (5a) aynen kalır; kumanda **ek** bir giriş.

## Hedefler / hedef olmayanlar / anti-hedefler

- **Hedef:** eldeki kumandayla (aşağıda) robotu güvenle sürmek; kopan
  bağlantıda robotun ≤ ~0,3 s içinde sıfır komut alması.
- **Hedef değil:** orantılı hız (kumanda vermiyor — ölçüldü); kumandadan kayıt
  başlatma (telefonun işi); otonom envanter.
- **Anti-hedef:** motor komut yolunun alt katmanlarından herhangi birini
  zayıflatmak (twist_mux zaman aşımı, köprü `cmd_timeout` 0,5 s, ESP32 200 ms
  watchdog, E-stop, tampon vetosu, hız limitleri, kit ivme rampası). Kumanda
  `/cmd_vel`'e **doğrudan yazmaz**.
- **Anti-hedef:** "bağlantı canlı" kararını Linux'un ürettiği olaylara
  dayandırmak (aşağıda, tuzak 1).

## Kumanda: ne olduğu ve ölçülenler

`VR-PARK` (kılavuzda "Y1 Bluetooth Remote Controller"), 2× AAA. Kendini Apple
klavyesi gibi tanıtıyor (`bus 0x5 vendor 0x5ac product 0x22c`), **BLE HID**
(HID over GATT, `0x1812`), pil servisi var (`Battery Percentage: 49`).

| Ölçülen | Değer | Sonuç |
|---|---|---|
| BLE supervision timeout | **6000 ms** (kumanda bağlanırken istiyor) | Linux kopuşu 3–10 s geç fark ediyor (kapatınca 3–4 s, menzil dışı ~10 s) → güvenlik buna **dayanamaz** |
| `@+D` fare modu, çubuk itili | `REL_Y = -17` her **~30 ms**, en uzun aralık 37 ms; yarım/tam itiş **aynı** değer | Orantı yok, ama **kumandanın kendi** düzenli akışı var → canlılık kanıtı |
| `@+B` müzik modu | `KEY_PREVIOUSSONG` + `value 2` tekrarları | **Tuzak 1:** `value 2`'yi kernel'in klavye otomatik tekrarı üretir (`REP_PERIOD 33`), kopuşta da sürer. Elendi |
| `@+C` yön tuşu modu | `ABS_HAT0X/Y` ∈ {-1,0,1}, yalnız değişimde | Orantı yok, sabitken akış yok. Elendi (kullanıcı kararı: A) |
| Arka tetik (`@+D`) | `BTN_SIDE` 1 → 30 ms sonra 0, basılı tutulsa da | Tuşlar **tıklama**; basılı-tut ölü adam tuşu olamaz |
| Uyku | dokunulmazsa ~5–10 dk | Uyuyunca bağlantı düşer → robot durur (güvenli); operatör uyandırır |
| Kılavuz SSS 1 | "düşük pil çubuğu bozabilir" | Pil eşiği gerekir |
| Kılavuz SSS 3 | "eşleşme Wi-Fi'dan etkilenebilir" | Pi 4'te Wi-Fi ve BT aynı çip; saha testi Wi-Fi açıkken |

## Yaklaşımlar

| | Ne | Bedel | Karar |
|---|---|---|---|
| **A** | Fare modu (`@+D`): `REL` akışı canlılık kanıtı, yaylı çubuk ölü adam | Sabit hız (kademeli) | **Seçildi** (kullanıcı, 2026-11-20) |
| B | Yön tuşu modu (`@+C`) + ham `hidraw` raporlarıyla canlılık | Ucuz cihazın rapor biçimini çözmek; canlılık ölçülmemiş davranışa dayanır | Elendi |
| C | PS4/PS5 kumandası (sürekli IMU akışı, analog) | Satın alma | Elendi — kullanıcı eldekini istedi |
| — | ROS `joy` + `teleop_twist_joy` | `joy` düğümü son durumu tekrar yayınlar (autorepeat); fare `REL` olaylarını eksen gibi okumaz | Kullanılmaz |

## Tasarım (A)

### Akış

```
VR-PARK (BLE, @+D) ─► /dev/input/eventN ─► joystick_teleop düğümü ─► /cmd_vel_joy ─► twist_mux (öncelik 110) ─► /cmd_vel ─► köprü
                                              └─► joy/state (JSON, 10 Hz) ─► web rozeti + öğrenme kaydı
```

### Kurallar (sırayla)

1. **Yalnız `REL_X` / `REL_Y` sürer.** Başka olay türleri (`KEY_*`, `ABS_*`)
   sürüşe hiç girmez — kumanda yanlış moddaysa robot kıpırdamaz. Düğüm
   bağlıyken 2 s boyunca yalnız `REL` dışı olay görürse durumda
   `mode_hint: "@+D?"` gösterir.
2. **Canlılık = son `REL` olayının yaşı.** Yaş < `deadman_s` → komut; yaş
   ≥ `deadman_s` → **sıfır** yayınla; `release_s` sonra hiçbir şey yayınlama
   (yaş, `REL_X` ve `REL_Y`'den hangisi en son geldiyse ona göre; yay
   çizerken eksenlerden biri susarsa o eksenin bileşeni sıfırlanır)
   (mux web'e/Nav2'ye döner). Web kapısındaki (`teleop.py`) üç bölgeli
   davranışın aynısı. Varsayılan `deadman_s = 0,25` (akış 30 ms, en uzun
   ölçülen boşluk 37 ms → ~6 kaçan rapora dayanıklı), `release_s = 2,0`.
   Zaman damgası olayın kendi damgası değil, **düğümün monotonik saati**
   (olay okunduğu an).
3. **Etkinleştirme şartı.** Düğüm `DISARMED` başlar; `arm_button` tıklaması
   → `ARMED`, tekrar tıklama → `DISARMED`. `DISARMED`'da `REL` yok sayılır,
   hiçbir şey yayınlanmaz. Kendiliğinden `DISARMED`'a düşer:
   - cihaz kaybolursa (kopuş / uyku) — geri gelince **yeniden etkinleştirme
     gerekir**,
   - `arm_idle_s` (varsayılan 60 s) boyunca hiç `REL` gelmezse,
   - pil `min_battery_pct` altına inerse.
   Etkinleştirme, `REL` akışı **sürerken** (çubuk itiliyken) reddedilir —
   etkinleştirildiği an robot yürümesin.
   **Kapatma da sıfırdan geçer** (karar, builder kontrolünden): hangi sebeple
   olursa olsun `DISARMED`'a geçişte düğüm `release_s` boyunca **sıfır**
   yayınlar, sonra susar — çubuğu bırakmakla aynı yol. Doğrudan susmak,
   mux'ı hareket hâlindeyken bir sonraki girişe (web kirası / Nav2 hedefi)
   devrederdi.
   **Bilinen davranış — kısa radyo boşluğu:** Fare modunda "çubuk ortada" ile
   "6 s'den kısa kopuş" düğüm için aynı görünür (ikisinde de `REL` yok).
   Bağlantı Linux cihazı silmeden geri gelirse düğüm `ARMED` kalır; operatör
   çubuğu hâlâ itiyorsa robot yeniden yürür. Arada ölü adam robotu
   durdurmuştur. Ayırt etmenin zamanında bir yolu yok (BlueZ `Connected` de
   geç) — kabul edildi, saha testinde gözlenir.
4. **Yön → komut** (çubuğun yalnız işareti kullanılır, büyüklüğü değil):
   `REL_Y < 0` ileri, `> 0` geri; `REL_X` dönüş. İleri/geri ile dönüş birlikte
   gelirse yay çizilir. `linear = sign · v_kademe`, `angular = sign · w_kademe`;
   geri hız `reverse_max` ile ayrıca sınırlı (varsayılan 0,2 m/s; panorama
   yönü hareket yönünden hesaplandığı için tarama sırasında geri gitmek
   istenmez). Dönüş yönü işareti (`invert_x`) parametre — sahada doğrulanır.
5. **Hız kademeleri:** `speed_up_button` / `speed_down_button` tıklamaları;
   kademeler `[0,2, 0,4, 0,6]` m/s, dönüş `[0,4, 0,7, 1,0]` rad/s; açılışta ve
   her yeniden etkinleştirmede **en düşük** kademe. Tavan web'le aynı
   (0,6 / 1,0); köprünün 1,0 / 1,5'i ve kitli moddaki ivme rampası altta aynen
   uygulanır.
6. **Pil:** BlueZ `org.bluez.Battery1.Percentage` 30 s'de bir okunur.
   `< min_battery_pct` (varsayılan 25) → etkinleştirme reddedilir / sürüş
   kapanır. Okunamazsa sürüş serbest, durumda `battery: null` + uyarı.
7. **Cihaz sahipliği:** düğüm cihazı `EVIOCGRAB` ile kendine alır (başka
   tüketici olmasın). Cihaz adı + MAC (`uniq`) ile eşlenir; Apple klavyesi
   kimliğiyle değil.
8. **twist_mux:** yeni giriş `joystick`, topic `cmd_vel_joy`, timeout 0,5,
   **öncelik 110** — robotun yanındaki insan, sayfadaki insanı geçer.
   `DISARMED` kumanda sessiz olduğu için web sürüşünü etkilemez.

### Tuş eşlemesi
Fare modunda `A B C D` ve iki tetiğin hangi kodu gönderdiği **henüz
ölçülmedi** (yalnız arka tetik: `BTN_SIDE`). Tuşlar parametre (`arm_button`,
`speed_up_button`, `speed_down_button`, evdev kod adı); varsayılanlar ilk
ölçümde doldurulur. Önerilen kullanım: arka tetik = etkinleştir/kapat,
`A`/`B` = hız +/−.

### Durum ve arayüz
`joy/state` (String JSON, 10 Hz): `connected`, `armed`, `mode_hint`,
`speed_level`, `battery`, `rel_age_s`, `linear`, `angular`. Web'de başlıkta
küçük bir kumanda rozeti (bağlı / etkin / kademe / pil).

### Öğrenme verisi (karar, builder kontrolünden)
Kumandayla sürüş **insan sürüşüdür**. Bugün `robot_learning` (`dataset.py`,
`export.py`) "insan sürdü" kararını yalnız `/cmd_vel_teleop` tazeyken veriyor;
kumandanın sürdüğü kareler `none` ya da `nav` etiketi alırdı — eğitim için
yanlış etiket. Bu yüzden: `learning_topics.yaml`'a `/cmd_vel_joy` ve
`/joy/state` eklenir; `dataset.py` / `export.py`'ye `joystick` kaynağı
eklenir ve mux'taki gibi **web teleop'tan önce** gelir; ilgili testler
güncellenir.

### Kod yerleşimi (öneri — builder'a bırakılabilir)
`robot_missions` paketi (web teleop'un yanında): `joystick_gate.py` (ROS'suz,
saat enjekte — `teleop.py` gibi), `joystick_teleop.py` (düğüm: evdev okuma
iş parçacığı + 20 Hz yayın + BlueZ pil), `twist_mux.yaml`'a giriş,
`robot.launch.py`'de `use_joystick:=false` argümanı (varsayılan kapalı).
Bağımlılık: `python3-evdev` (apt); pil için D-Bus (`python3-dbus` ya da
`busctl` alt süreci — builder seçer).

## Pi tarafı (deployment.md'ye eklenecek)
- `sudo apt install bluez python3-evdev`; robot servisinin kullanıcısı `input`
  grubunda.
- Eşleştirme bir kez: `bluetoothctl` → `pair/trust/connect`. `trust` sayesinde
  kumanda açılınca kendiliğinden bağlanır.
- Kumanda her açılışta `@+D`'ye alınır (mod hafızası **ölçülmedi** — sahada
  kontrol).

## Test planı
1. **Birim (ROS'suz, adım saatli):** kapı kuralları — `DISARMED` sessiz; akış
   kesilince `deadman_s`'de sıfır, `release_s`'de sessiz; kopuş → `DISARMED` +
   yeniden etkinleştirme şartı; itiliyken etkinleştirme reddi; yalnız `REL`
   sürer, `KEY/ABS` sürmez; geri hız sınırı; kademe sınırları; pil eşiği;
   NaN/aşırı değer → sıfır; hareket hâlinde kapatma → önce `release_s` sıfır;
   yay çizerken tek eksen susunca o bileşen sıfır. `robot_learning`:
   kumandanın sürdüğü kare `joystick` etiketi alır, web'le aynı anda tazeyse
   `joystick` kazanır.
2. **Yığın (kinematik dünya, gerçek mux + köprü, `test_web_stack.py` gibi):**
   sahte olay kaynağıyla tam kademede sür → akışı kes → ilk sıfır
   ≤ `deadman_s` + bir tik, robotun durduğu mesafe ölçülür; web sürerken
   kumanda etkinleşince kumanda kazanır, kumanda susunca web'e döner;
   `/cmd_vel`'in tek yayıncısı hâlâ twist_mux (mevcut assert).
3. **Gerçek kumanda (Pi, tekerlekler havada → sonra yerde):** tuş eşlemesi
   ölçümü; tam kademede kumandayı **kapat** ve **menzil dışına çık** → durma
   mesafesi; Wi-Fi açıkken menzil; uyku → robot durur, uyanınca yeniden
   etkinleştirme gerekir.

## Builder'a bırakılanlar
Evdev okuma biçimi (iş parçacığı / asyncio), D-Bus kütüphanesi, web
rozetinin görünümü, parametre dosyasının yeri, test dosyalarının bölünmesi.
