# Hoverboard Anakart Bring-Up Checklist — TXTY150914NNC-6052MAIN_V2.1

> Bu kart topluluk veritabanlarında **belgesiz**. Aşağıdaki hiçbir pin
> konumu/kablo rengi garanti değil — her ⚠️ adımında **multimetreyle
> doğrula**. MCU/firmware seviyesindeki adımlar (unlock, config, protokol)
> tüm STM32F103R kartlarında aynıdır ve güvenilirdir.

MCU: STM32F103, LQFP64 (F103R ailesi) — firmware hedefi `STM32F103RCT6`.
Firmware: `EFeru/hoverboard-firmware-hack-FOC` (PlatformIO).

> 🧭 **Sihirbaz:** bu dosyanın her adımı `./scripts/b1_flash.sh` ile adım adım
> yürütülür — komutları koşar, çıktıları ayrıştırır, `config.h`'yi denetler,
> UART'ı dinleyerek TX telini bulur, tekerlek + timeout testini yapar
> (`tools/b1_flash/`). Donanımsız prova: `./scripts/b1_flash.sh --prova`.
> Bu dosya hâlâ tek kaynak; sihirbaz onu uygular.

> 📌 **2026-11-12 düzeltmeleri** (EFeru `main` kaynağı okunarak):
> varyant `config.h`'den değil **PlatformIO env'inden** seçilir;
> `CTRL_MOD_REQ` varsayılanı `VLT_MODE`'dur (elle değiştir); akım limitleri
> 20 A sigortaya göre **8 / 10 A**; J4'te **NRST yok** ("Connect Under
> Reset" için ayrıca bulunmalı); timeout davranışı kaynaktan doğrulandı.

---

## 0. Güvenlik ön koşulu (pazarlıksız)

- [ ] Tekerlekler **yerden kesik** — kartı/motoru mengeneye al ya da robotu sehpaya kaldır.
- [ ] Batarya çıkışına **20 A / 60 V sigorta** takılı (elde var, yuvalı).
      Voltaj dayanımı dolu paketin 42 V'unun üstünde olmalı — 32 V'luk otomobil
      bıçak sigortası burada yanlış parça.
- [ ] Fiziksel **E-stop** (mantar buton, NC) güç hattını fiziksel kesiyor.
- [ ] Yangın söndürücü / kova kum yakında (Li-ion, ilk flash denemesi).

**Araçlar** (dev container'da hazır — `docs/devcontainer.md`; yoksa
`sudo apt install stlink-tools openocd`, `pio`):
- [ ] ST-Link takılıyken `lsusb` → `0483:3748` görünüyor.
- [ ] `st-info --probe` "Found 0 stlink programmers" / `libusb error: -4`
      diyorsa: container'da `/dev/bus/usb` yok ya da izin sorunu
      (`docs/devcontainer.md` "İzinler").
- [ ] **ST-Link klonunun kasa yazısına güvenme:** pin 3/4'ün (GND) USB
      soketinin metal gövdesiyle sürekliliği var (multimetre, bip).

---

## 1. Çip kimliğini kesinleştir (GD32 klon riski)

> Sen "biri STM32 biri GD32" demiştin. Üzerinde "STM32F103" yazsa bile
> çoğu klon GD32 çıkıyor.

- [ ] ST-Link'i **sadece GND + SWDIO + SWCLK** ile bağla. **3V3 pinini bağlama**
      (programlayıcının 3.3V'u kartı beslerse yakabilir — wiki'nin açık uyarısı).
- [ ] Batarya bağlı, güç butonuna basılı tut (self-latch — bırakırsan MCU kapanır).
- [ ] Çalıştır ve sonucu not et:
      ```
      st-info --probe
      ```
  - Beklenen (RCT6): `chipid 0x414` (F1 high-density), `flash 262144` (256 KiB).
    RDP kilitliyken flash boyutu okunamayabilir — bağlantının kurulması yeter.
  - `chipid` 0 / "unknown device" → hedef yok: kablo, güç butonu, NRST (aşağıda).
- [ ] **Çekirdek kimliği (CPUID)** — `chipid` GD32'de de 0x414 çıkabilir;
      ayıran şey çekirdek revizyonu:
      ```
      openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c init -c "mdw 0xE000ED00" -c shutdown
      ```
  - `411fc231` (Cortex-M3 r1p1 — STM32F10x erratası bu revizyonu yazar) → standart yol, devam.
  - Başka bir değer, ya da OpenOCD `UNEXPECTED idcode` uyarısı → **klon
    olabilir. DUR**, değeri ve çip yazısını not et. GD32'de unlock ve
    hatta baud farklı olabilir.

**⚠️ SWD pad sırası sabit değil.** Pad'leri çipe kadar izle:
`SWDIO = PA13`, `SWCLK = PA14`. Renge/sıraya güvenme.

**✅ Kart 1'de ölçüldü (2026-10-16, multimetre süreklilik):** SWD = **`J4`**,
çipin sol üstündeki 3 yuvarlak pad (C19/C24'ün üstü). 3V3 dışarı verilmemiş.

| J4 pad (çipe bakarken) | Çip bacağı | Sinyal | ST-Link pini |
|---|---|---|---|
| sol | sol kenar en üst = **49** | SWCLK (PA14) | 6 |
| orta | — (batarya eksisiyle bip) | GND | 3 veya 4 |
| sağ | üst kenar soldan 3. = **46** | SWDIO (PA13) | 2 |

Yan ürün: **pin 1 = çip gövdesindeki noktalı köşe (sol alt)**. PCB'deki
sol üst köşe beyaz üçgeni pin 1 değil — bacak sayarken ona güvenme.
Sol-üst bacağın 49 çıkması bunu kanıtlıyor (diğer yorumda o bacak VBAT olurdu).
Bacak 46/49 = PA13/PA14 STM32F103 LQFP64 datasheet'iyle de tutuyor.

**⚠️ J4'te NRST yok.** "Connect Under Reset" (adım 2) ST-Link'in RST
pininin (1) çipin NRST'sine bağlı olmasını ister. Önce **normal bağlantıyı
dene** — stok firmware SWD pinlerini kapatmamışsa yeter. Bağlanamazsan:
NRST = LQFP64 **bacak 7** (datasheet; tüm F103R'lerde aynı). Karttaki
izini **multimetreyle bul** (genelde 100 nF'lık bir kondansatöre ya da bir
test pad'ine gider) — tahminle tel lehimleme. Bulunca tabloya ekle.

---

## 2. Okuma korumasını (RDP) kaldır

> Fabrika firmware'i kilitli. Unlock, stok firmware'i **geri dönüşsüz** siler
> (RDP yüzünden önceden yedeği de alınamaz). Tuğlalama riski pratikte yok —
> sonrasında istediğin kadar flash'larsın. Kart 2 yedek.

OpenOCD (sihirbazın kullandığı):
```
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c init -c "reset halt" -c "stm32f1x unlock 0" -c shutdown
```
Başarı satırı: `stm32x unlocked`.

veya STM32CubeProgrammer (GUI):
- [ ] Bağlantı modu: önce **Normal**, olmazsa **"Connect Under Reset"**
      (NRST teli gerekir — adım 1'deki not).
- [ ] `OB` (Option Bytes) → `RDP` = `AA` (Disable) → Apply.

Her iki yolda da:
- [ ] Kartın gücünü **kes-ver** (yeni option byte'lar ancak öyle geçerli).
- [ ] Doğrula: flash artık okunabilir ve **silinmiş** (hep `ffffffff`):
      ```
      openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c init -c "reset halt" -c "mdw 0x08000000 4" -c shutdown
      ```
      Kilitliyken bu okuma hata verir ya da sıfır döner; okunuyor ama
      `ffffffff` değilse kilit zaten açık ve içinde firmware var.

---

## 3. Firmware'i yapılandır (`Inc/config.h`)

```
git clone https://github.com/EFeru/hoverboard-firmware-hack-FOC
cd hoverboard-firmware-hack-FOC && git rev-parse HEAD   # hangi sürümü flash'ladığını not et
```

**Varyant `config.h`'de değil, PlatformIO env'inde seçilir.**
`platformio.ini`'de her varyant bir env (`[env:VARIANT_USART]` →
`-D VARIANT_USART`); `default_envs` satırlarının hepsi yorumda, yani env
vermezsen **10 varyantın hepsi** derlenir. Her komutta `-e VARIANT_USART`.
(`config.h`'deki `//#define VARIANT_...` listesi `#if !defined(PLATFORMIO)`
altında — PlatformIO'da hiç okunmaz.)

`VARIANT_USART` bloğu zaten istediğimiz gibi gelir — **dokunma**:
```c
#define CONTROL_SERIAL_USART2  0    // sol kablo (sayı = dual-input önceliği)
#define FEEDBACK_SERIAL_USART2      // hall'dan hız feedback'i geri gelsin
// #define TANK_STEERING            // KAPALI kalmalı: ESP32 steer+speed gönderiyor
```

Global bölümde (`MOTOR CONTROL`) **değiştirilecekler**:
```c
#define CTRL_TYP_SEL    FOC_CTRL     // varsayılan zaten FOC
#define CTRL_MOD_REQ    SPD_MODE     // ⚠️ varsayılan VLT_MODE — DEĞİŞTİR
#define I_MOT_MAX       8            // ⚠️ varsayılan 15 (motor başına)
#define I_DC_MAX        10           // ⚠️ varsayılan 17; kural: I_DC_MAX = I_MOT_MAX + 2
#define N_MOT_MAX       1000         // [rpm] varsayılan; SPD_MODE'da 1000 komut ↔ N_MOT_MAX
```
- [ ] ⚠️ **Akım limitleri 20 A sigortaya göre.** Limitler **motor başına**;
      varsayılanla iki motor 2 × 17 = 34 A çekebilir → sert kalkışta/sıkışmada
      sigorta atar. 8/10 A ile toplam ≤ 20 A. Bu tezgah için muhafazakâr
      başlangıç; yük altında sürüşten (B5) önce yeniden değerlendirilecek.
- [ ] `BAT_CELLS 10` (varsayılan zaten 10).
- [ ] `ENABLE_ODOMETRY` **kapalı kalmalı.** Açılırsa feedback çerçevesine
      `wheelR_cnt/wheelL_cnt` girer (18 → 22 bayt) ve ESP32'deki
      `HoverFeedback` (`firmware/esp32_bridge/src/main.cpp`) yanlış ayrıştırır.
- [ ] `USART2_BAUD 115200` (ESP32 `HOVER_BAUD` ile aynı), `SERIAL_TIMEOUT 160`
      (≈ 0.8 s; 5 ms döngü).
- [ ] `BAT_CALIB_REAL_VOLTAGE` / `BAT_CALIB_ADC` — şimdi **dokunma**, B4 işi.
- [ ] Derle:
      ```
      pio run -e VARIANT_USART
      ```
      Çıktı: `.pio/build/VARIANT_USART/firmware.bin`.

**Timeout davranışı (kaynaktan, `Src/util.c` `readInputRaw`):** geçerli
komut (doğru start + checksum) `SERIAL_TIMEOUT` döngü gelmezse
`ctrlModReq = OPEN_MODE` ve komutlar 0 → motor gücü kontrollü sıfırlanır,
tekerlek **serbest döner** (coast), fren yok. Komut gelince kendiliğinden
çıkar. Feedback'teki `cmd1/cmd2` o an 0'a düşer — adım 6 bunu ölçer.

---

## 4. Flash

- [ ] Batarya bağlı (>36V), güç butonuna **flash boyunca** basılı tut,
      tekerlekler havada.
- [ ] Yaz:
      ```
      pio run -e VARIANT_USART -t upload
      ```
      (`upload_protocol = stlink` `platformio.ini`'de hazır; ek flag yok.)
- [ ] Flash *sırasında* MCU halt → motor dönmez. Risk flash *sonrası* firmware
      çalışınca başlar; onu havada tekerlekle karşılıyorsun.
- [ ] Hangi EFeru commit'inin flash'landığını not et (`git rev-parse HEAD`).

---

## 5. ⚠️ UART hattını multimetreyle doğrula (ESP32 bağlamadan ÖNCE)

> Firmware kuralı: **sol kablo = USART2** (bizim varyantın kullandığı),
> sağ kablo = USART3. Fiziksel pin sırası ve GND/15V yeri bu kartta belgesiz.
> EFeru'nun kendi uyarısı: *"kırmızı tel 15 V — her şeyi yakar."*
> (Checklist'in eski "USART3 5V toleranslı" notu `config.h`'de geçmiyor —
> **doğrulanmadı**; bu kurulumda önemsiz, iki taraf da 3.3 V.)

Yan kart (BLB) sökülü. **Sol** kablo demeti üzerinde:
- [ ] Süreklilik modu: her teli kartın **GND'sine** (batarya eksi / büyük bakır alan)
      değdir → **GND telini** bul.
- [ ] Batarya bağlıyken (dikkatli) her tele karşı GND voltajı ölç:
      **~15V olan = besleme.** Bunu **ESP32'ye/USB-TTL'e ASLA bağlama** — ucunu bantla.
- [ ] Kalan iki tel (~3.3V boşta) = TX / RX. Voltajla ayırt **edilemez**
      (ikisi de boşta yüksek). Ayırt etmenin güvenli yolu **yalnız dinlemek**:
      USB-TTL'in (FT232RL, VCCIO jumper **3.3V**) **sadece GND + RX**'ini
      bağla, aday teli RX'e değdir; firmware 10 ms'de bir feedback
      gönderdiği için geçerli `0xABCD` çerçeveleri geliyorsa o tel
      **STM32 TX**'tir. Kalan tel STM32 RX. (USB-TTL TX'i bu sırada bağlı değil
      → karta hiçbir şey sürülmüyor.)
- [ ] Sonuç tablosu:

  | Tel rengi | Ölçüm | Rol | ESP32'ye |
  |-----------|-------|-----|----------|
  |           | 0V/süreklilik | GND | GND |
  |           | ~15V  | 15V | **BAĞLAMA** |
  |           | ~3.3V, çerçeve geliyor | STM32 TX | RX2 (GPIO16) |
  |           | ~3.3V | STM32 RX | TX2 (GPIO17) |

- [ ] STM32 ve ESP32 ikisi de 3.3V mantık — seviye çevirici gerekmez.

---

## 6. Tekerlek testi (başarı kriteri)

- [ ] USB-TTL'i (ya da ESP32'yi) UART'a bağla, 115200 baud, TX↔RX çapraz.
      Dev container'da USB-TTL için `--device=/dev/ttyUSB0` gerekir
      (`docs/devcontainer.md`) — cihaz container başlamadan **önce** takılı olmalı.
- [ ] Önce yalnız dinle: feedback çerçeveleri (start `0xABCD`, checksum tutuyor).
- [ ] **Küçük** hız komutu gönder (`steer 0`, `speed` ≤ 100 / 1000 ≈ ≤ 100 rpm):
      start `0xABCD`, int16 steer, int16 speed, checksum = `start ^ steer ^ speed`.
- [ ] **Tekerlek dönüyor.** ✅ Projenin en riskli kısmı bitti.
- [ ] Feedback'te `speedR_meas` / `speedL_meas` okunuyor (odometrinin temeli)
      — işareti ve hangi tekerleğin ileri döndüğünü not et.
- [ ] **Timeout testi (güvenlik — atlanmaz):** dönerken komutu **kes**.
      Beklenen: ~0.8 s içinde feedback'te `cmd1/cmd2 = 0` ve tekerlek
      serbest dönüşle duruyor. Bağlantı kopunca güvenli duruş buna bağlı
      (ESP32 watchdog'unun arkasındaki ikinci katman).
- [ ] Yön ters/stutter varsa: `INVERT_*_DIRECTION` veya hall kombinasyonunu değiştir
      (yanlış hall = dönmez/titrer, **hasar vermez**).

---

## Hoverboard seri protokolü (referans)

**Komut (ESP32 → STM32), 115200 baud, 8 bayt, little-endian:**
```
uint16 start   = 0xABCD
int16  steer
int16  speed
uint16 checksum = start ^ steer ^ speed
```
**Feedback (STM32 → ESP32), 10 ms'de bir, 18 bayt (`ENABLE_ODOMETRY` kapalıyken):**
```
uint16 start = 0xABCD
int16  cmd1, cmd2                 // firmware'in uyguladığı komut (timeout'ta 0)
int16  speedR_meas, speedL_meas   // hall'dan [rpm] — odometri
int16  batVoltage                 // ~santi-volt (kalibrasyon B4'te)
int16  boardTemp                  // [°C×10]
uint16 cmdLed
uint16 checksum                   // tüm alanların XOR'u
```

## Bilinen tuzaklar
- **Self-latch:** açılışta butona bas, kapatmak için uzun bas. Flash boyunca basılı tut.
- **Buzzer/LED pinleri** bu varyantta belgesiz — motorlar çalışsa bile
  buzzer sessiz / LED farklı olabilir. Kritik değil.
- **GD32 ise:** farklı baud (bazı raporlarda 52177) ve farklı unlock. Adım 1'de yakala.
- **J4'te NRST yok** — "Connect Under Reset" gerekirse bacak 7'nin izi bulunmalı.
- **`pio run` env'siz** tüm varyantları derler ve `-t upload` hangisini
  yazdığı belirsiz olur — her zaman `-e VARIANT_USART`.
