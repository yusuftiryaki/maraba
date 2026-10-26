# Kit soketi — orchadOS tarama kitinin robota takılması

*(tasarım taslağı, 2026-10-25 — donanım yok, hiçbir şey ölçülmedi)*

## Amaç ve sınırlar

- **Robot bugünkü donanımıyla tek başına çalışır.** Kit yoksa hiçbir şey
  değişmez: NEO-6M, Pi Camera V2, mevcut hız limitleri, mevcut görevler.
- Robotta **hazır bir soket** var. orchadOS kiti takılınca robot bunu anlar ve
  **kitli moda** geçer; çıkarılınca güvenli biçimde kitsiz moda döner.
- Kitin iki sürümü var, soket ikisini de baştan karşılar:
  - **v1 (LiDAR'sız):** ZED-F9P + anten, 360° kamera, ESP32 GNSS kaydedici.
  - **v2 (LiDAR'lı):** v1'e ek olarak Livox Mid-360 sınıfı LiDAR ve Pi 5 (ROS 2).
- **Kit, robot olmadan da çalışır** (sırtta ya da motosiklette). Soket kite eklenen
  bir fiştir; kitin kendi kayıt yolu (ESP32 → microSD, kamera → kendi kartı,
  BLE → telefon) robot takılı olsa da aynen çalışır.
- **Repolar ayrı.** Soketin sahibi robottur: pin dizilimi, kimlik formatı ve
  topic sözleşmesi bu belgede tanımlıdır. orchadOS kiti bu sözleşmeye uyar.
  Seri protokoldeki kural burada da geçerli: kimlik formatının test vektörleri
  iki repoda aynı tutulur (bkz. "Repolar arası sözleşme").

## Kullanıcı kararları (2026-10-25)

- **Kit takılınca hız limitleri değişmez.** Aşağıdaki devrilme hesabı bunu
  destekliyor, ama yalnız montaj plakası destek alanının ortasındaysa.
- **Kitli mod için ayrı bir manyetometre profili gerekmez.** Direk alüminyum.
  Kalan tek şüpheli parça kitin aküsü (hücrelerin kılıfı çelik); kit ilk
  takıldığında bir kez yön kontrolü yapılır (aşağıda).
- Konnektör, kimlik çipi ve montaj yeri kararları tasarıma bırakıldı; seçimler
  ve gerekçeleri aşağıda.
- **Repolar ayrı kalır;** soketin sözleşmesi bu belgede.
- **Kitin kamerası kitli modda ek bir algı katmanı olur,** robotun temel
  algısının yerine geçmez (bkz. "Kitin kamerasıyla algı").
- **Ağaç haritası bir başlangıç tahminidir;** robot onu denetler, sahada
  değiştirmez (bkz. "Ağaç haritası").
- **Envanter önce insanla, robot önce periyodik ziyaretlerle başlar;** robotla
  ilk envanter LiDAR ya da olgun bir gövde modeliyle açılır (bkz. "Envanter
  sırası").

## Kit nasıl algılanır

Robot iki şeye bakar:
1. **Fiş takılı mı?** Kit tarafında fişin içinde `KIT_PRESENT` pini GND'ye
   köprülüdür. Robot tarafında pin pull-up'lıdır, yani LOW "fiş takılı" demektir.
2. **Takılan şey ne?** Fişin içindeki kimlik çipi okunur (aşağıda manifest).
   Ardından veri akışına bakılır: F9P konum gönderiyor mu, v2'de LiDAR bulutu
   geliyor mu?

Hız limiti değişmediği için bu algılama güvenlik için kritik değil. Bundan
etkilenenler GPS kaynağı, hangi görevlerin kabul edildiği ve kayıt.

## Soket — elektrik

**Konnektör:** robot gövdesinde panele takılan dişi bir **M16, 12 pinli, IP67
dairesel konnektör**; kitin kablosunun ucunda bunun erkek fişi. Bahçe tozuna
dayanıklı, vidalı kilitli (sarsıntıda çıkmaz), yaygın ve ucuz. Kablo boyu
≤ 1 m, diferansiyel çiftler (USB, Ethernet) bükümlü tel.

| # | Sinyal | Yön | v1 | v2 | Not |
|---|---|---|---|---|---|
| 1 | GND | — | ✅ | ✅ | Ortak GND (karar 13'teki tek nokta) |
| 2 | `KIT_PRESENT` | kit→robot | ✅ | ✅ | Kit tarafında GND'ye köprü; robotta pull-up. LOW = fiş takılı |
| 3 | `KIT_ID` | ↔ | ✅ | ✅ | 1-Wire kimlik çipi (aşağıda). **IMU'nun I2C bus'ı DEĞİL** |
| 4 | `PPS` | kit→robot | ✅ | ✅ | F9P TIMEPULSE, 3,3 V; Pi'da `pps-gpio` + chrony. Seri direnç |
| 5–6 | USB D+ / D− | ↔ | ✅ | ✅ | Kitte **kendi beslemeli bir USB hub**: F9P (`/dev/kit_gnss`) + 360° kamera (Camera SDK; bkz. "360° kamera"). v2'de kamerayı Pi 5 sürer, bu çift boş kalabilir |
| 7 | USB VBUS | — | ⚠️ | — | **Bağlanmaz.** Kit kendi aküsünden beslenir; VBUS bağlanırsa kit aküsü ile powerbank birbirini besler |
| 8–11 | Ethernet TX± / RX± | ↔ | rezerve | ✅ | 100BASE-TX; robot Pi 4 ↔ kit Pi 5 |
| 12 | `KIT_PWR` | robot→kit | rezerve | rezerve | Şimdi kullanılmaz: elektronik powerbank'ten besleniyor (karar 13) ve kiti besleyecek bütçesi yok |

- **Güç soketten geçmez.** Kit kendi 18 V aküsüyle çalışır. Hem v1 hem v2 böyle.
  Pin 12, robota ileride kendi DC-DC çeviricisi takılırsa diye rezerve.
- **Kimlik çipi: DS2431** (1-Wire EEPROM, 128 bayt). Fişin içine sığan, birkaç
  liralık bir hafıza çipi; kitin seri numarasını, donanımını ve kalibrasyonunu
  taşır. Tek veri teli + GND yetiyor, Pi'da `w1-gpio` hazır. I2C yerine 1-Wire
  seçmemizin sebebi şu: uzun kablodaki bir I2C hatası, IMU'yu, manyetometreyi ve
  INA228'i taşıyan bus'ı kilitleyebilir.
- ⚠️ **USB 2.0 genel amaçlı bir konnektörden geçiyor.** 1 m'de genelde çalışır,
  ama kamera SDK'sı yüksek hızlı akış isterse kopmalar olabilir. K3'te ilk
  denenecek şey bu; sorun çıkarsa USB'ye ayrı, su geçirmez bir USB konnektörü
  verilir.
- Pin numaraları ve GPIO atamaları **başlangıç önerisi**; donanım gelince
  wiring-map'e işlenir ve multimetreyle doğrulanır.

## Soket — mekanik: montaj plakası

**Yer: şasinin tepesinde, tam ortada.** `base_link`'e göre
**x = +0,11 m, y = 0, z = 0,1825 m** (şasi üst yüzeyi). URDF'te `kit_mount`
frame'i. Bu nokta şasinin ortası ve aynı zamanda destek alanının ortası.
Destek alanı, tekerlek ekseni (x = 0) ile sarhoş tekerlekler (x = +0,22)
arasında kalıyor.

**Neden burası: devrilme payı.** İlk hesap kabaydı (robot 17 kg tek nokta,
x = 0,11'de, "~21°"). K2b'de `robot_sim/kit_model.py` ile URDF'in kütleleriyle
yeniden hesaplandı ve Gazebo'da ölçüldü (`test_gazebo_kit.py`). URDF: şasi
12 kg (merkezi x = 0,11), tekerlekler 2×2,5 kg aks üstünde (x = 0), sarhoş
tekerlekler önde (x = 0,22). Tekerleklerin kütlesi aksta olduğu için robotun
ortak ağırlık merkezi x = 0,08'e geliyor ve **zayıf taraf arka** (aks çizgisi):

| Durum | Ağırlık merkezi (x, h) | Arkaya (burun yukarı) | Öne (burun aşağı) | Kalkış / fren ivmesi |
|---|---|---|---|---|
| Kit yok | 0,08 / 0,12 m | 31,9° | 48,7° | 0,62 g / 1,14 g |
| **Kit, plaka şasi ortasında (x = 0,11)** | **0,08 / 0,29 m** | **16,4°** | **25,3°** | **0,29 g / 0,47 g** |
| Kit, robotun kendi direğinin yerinde (x = −0,15) | 0,03 / 0,29 m | **6,0°** | 33,5° | 0,10 g / 0,66 g |

**Gazebo'da ölçülen** (URDF'in sarhoş tekerlek düzeni ve kütleleri, tekerlek
sürtünmesi μ 0,5, yerçekimi eğilerek eğim):
- 12° yokuşta burun yukarı **duruyor**, 22°'de **arkaya devriliyor** (şasinin
  kuyruğu yere değiyor); 20° yokuşta burun aşağı duruyor. Elle hesap 16,4° ve
  25,3° diyordu; fizik bununla tutarlı.
- **Sert kalkış** (0 → 0,5 m/s basamak, ölçülen 0,46 g): kitle **ön tekerlekler
  kalkıyor** (burun 1,25° yukarı, sonra geri iniyor); kitsiz kalkmıyor
  (0,33 g'de 0,02°). Sert duruş (0,33 g) kitle de bir şey yapmıyor (fren payı
  0,47 g).
- **Sonuç:** Eğim tarafında "hız limiti değişmesin" kararı tutuyor (16°'nin
  altındaki eğimlerde). Sert kalkış ise kitle ön tekerlekleri kaldırıyordu.
- ✅ **Kitli modda hızlanma rampası** (kullanıcı kararı 2026-10-26): köprü,
  `kit/state` "present" iken **yalnız hızlanmayı** `kit_max_linear_accel`
  (1,5 m/s² ≈ 0,15 g) ile sınırlıyor (`command_limits.ramp_speed_up`; v ve ω
  birlikte ölçeklenir, yay aynı kalır). Yavaşlama ve durma **hiç** rampalanmıyor:
  sert duruş payın altında (0,33 g < 0,47 g) ve rampalı bir duruş her deadman /
  zaman aşımı duruşunu uzatırdı. Kit yokken (ya da soket hiç yokken) hiçbir şey
  değişmiyor. Gazebo'da aynı basamak kalkış: rampasız 0,46 g → burun 1,25°
  kalkıyor; **rampalı 0,16 g → 0,06°** (kalkmıyor).
- Plakayı öne almak biraz yardım ediyor ama çözmüyor: x = 0,20'de arka pay
  19,8°, kalkış payı 0,36 g.
- ⚠️ Gazebo modeli (hoverbot.sdf) ön ve arka sarhoş tekerlekle elmas düzende;
  kit testleri tekerlekleri URDF'in ön çiftine taşıyor, çünkü riskli düzen o.
  Şasi daha yapılmadı (B3); yapılınca robot ve kit tartılıp tablo yeniden
  hesaplanmalı (`kit_model.margins`).
- **Plaka arkaya kayarsa pay üçte birine iner.** Kaymaması gereken tek ölçü bu.
- **Kitin aküsü direğin dibine konmalı.** Akü yukarıda olursa ağırlık merkezi
  yükselir ve tablo değişir.
- **Kalan risk:** direğin tepesinin bir dala takılması. Bu, 2 m'lik bir kaldıraç
  demek. Tampon yere yakın olduğu için bunu algılamaz.

**Yükseklikler yerden ölçülür ve sırttaki kitle aynı tutulur.** Böylece
robotla ve sırtla toplanan veri karşılaştırılabilir kalır. GNSS anteni ~2,1 m,
360° kamera ~2,0 m, v2'de LiDAR ~1,2 m. Plaka 0,18 m'de olduğu için robot
üzerinde kamera plakanın 1,82 m üstüne gelir. Kitin direği bu yüksekliğe
ayarlanabilir olmalı (orchadOS tarafının işi).

**Diğer sensörlerle çakışma:**
- **Pi Camera V2 ve kenar kamerası** öne bakıyor (x = +0,30, plakanın önünde).
  Direk ikisinin de görüşüne girmez; docking etiketi de etkilenmez.
- **Manyetometre** robotun kendi direğinde (x = −0,15, ~0,63 m yükseklikte).
  Kit direğine yatayda 0,26 m uzakta. Alüminyum direk sorun değil. Kitin aküsü
  ~0,45 m uzakta kalıyor.
  **Kontrol:** Kit ilk takıldığında robot yerinde bir tur döner ve
  manyetometrenin verdiği yön, kitsizkiyle karşılaştırılır
  (`mag-calibration.md`). Fark birkaç dereceden büyükse ancak o zaman ikinci bir
  kalibrasyon profili yapılır.
- **360° kamera görüntüsünde robot** aşağıda küçük bir bölge olarak kalır;
  maskelenir.
- **Nav2 ayak izi** değişmez (`robot_radius` 0,4 m). Direk bu çemberin içinde.

**Mekanik bağlantı:** Plaka alüminyum, üzerinde kit direğinin çapına uygun hızlı
bağlantılı bir boru yuvası. Paslanmaz ya da alüminyum vida kullanılmalı;
manyetometrenin yakınında çelik olmamalı. M16 soketi plakanın yanında, kablo
direğin dibinden iner.

## Kimlik çipinin içeriği (manifest)

Kit kendini bu çiple tanıtır. Kitin bilgisayarı açılmadan da okunur, yani v1'de
de çalışır. Biçim sabit alanlı ikili veri ve CRC'dir; toplam ~90 bayt, DS2431'in
128 baytına sığar. Alanlar (şema `1`):

| Alan | Örnek | Not |
|---|---|---|
| `kind` | `orchados-scan-kit` | Başka türden bir aksesuar ileride aynı soketi kullanabilir |
| `schema` | 1 | Robot bilmediği bir şemayı **reddeder** (kitsiz mod + uyarı) |
| `serial`, `hw_rev` | `K001`, `1.0` | |
| `caps` | `gnss_rtk`, `pps`, `cam360`, `cam360_ctrl`, `lidar` | Bit alanı: kitte **takılı** olan donanım. `cam360_ctrl`: kamera USB'den SDK ile sürülebiliyor |
| `gnss_link` | `usb` (v1) / `ros` (v2) | GNSS robota hangi yoldan geliyor |
| `calib_id` + dönüşümler | anten, kamera, LiDAR → `kit_mount` | float32 x/y/z/r/p/y. 3 sensör × 24 B = 72 B |

- Kalibrasyonun **asıl kaynağı orchadOS'tur.** Kit kalibre edilince orchadOS'taki
  bir araç dönüşümleri ve `calib_id`'yi çipe yazar. Robot kitin frame'lerini
  (`kit_gnss`, `kit_cam360`, `kit_lidar`) çipteki değerlerden statik TF olarak
  yayınlar. Böylece orchadOS'un "her kayıt hangi kalibrasyonla yapıldığını bilir"
  kuralı robotta da korunur.
- `caps`, takılı olan donanımı söyler; **çalışıyor olduğunu söylemez.** Çalışıp
  çalışmadığı veri akışından anlaşılır (aşağıda `DEGRADED`).

## Mod durum makinesi (`kit_manager`)

```
         fiş takılı + çip okundu          veri akıyor mu?
NONE ─────────────────────────► READY ◄──────────────► DEGRADED
  ▲                               │                      │
  └───────── fiş çıktı ───────────┴──────────────────────┘
```

| Durum | GPS kaynağı | Kitli görevler | LiDAR engel katmanı |
|---|---|---|---|
| `NONE` | NEO-6M | reddedilir | yok |
| `READY` | **kit F9P** | kabul | `lidar` varsa açık |
| `DEGRADED` | eksik olana göre | eksik olanı gerektirmiyorsa kabul | veri yoksa kapalı |

Hız limitleri her durumda aynıdır (kullanıcı kararı). `/kit/state` latch'li
yayınlanır ve web arayüzünde bir kit kartı olarak görünür. Fiş takılı ama çip
okunamıyor ya da şemayı bilmiyorsak durum `NONE` kalır ve uyarı verilir.

**Geçiş kuralları:**
- **GPS kaynağı yalnız robot dururken değişir:** aktif görev yokken ve komut
  sıfırken. NEO-6M ile F9P arasında birkaç metre fark var ve `map` çerçevesi
  datum'a çapalı (A3c tuzakları). Değişimden sonra `ekf_global` sıfırlanır ve
  datum yeniden kurulur. Bu adım **ölçülmeden güvenilmez.**
- **Görev sırasında fiş çıkarsa:** Görev duraklar, robot durur, alarm verir.
  GPS NEO-6M'e ancak durduktan sonra döner. Görevin devamı operatöre bırakılır.
- Pin debounce edilir. Bir geçişin kabul edilmesi için durumun ~0,5 s boyunca
  kararlı kalması gerekir.

## GNSS ve zaman

**v1 (`gnss_link: usb`):**
- **F9P'nin ayarı orchadOS'a aittir.** ESP32 kaydedici, UART1'den PPK için ham
  veri (RAWX/SFRBX) alıyor. Robot F9P'ye **hiç CFG göndermez**: sadece okur
  (NAV-PVT) ve RTCM yazar. Hazır ublox ROS sürücüleri açılışta alıcıyı
  yapılandırdığı için kullanılmaz. Yerine küçük bir `kit_gnss` düğümü yazılır:
  NAV-PVT → `/kit/gnss/fix` (frame `kit_gnss`, kovaryans hAcc/vAcc'den).
  Sahte bir F9P akışıyla test edilir, `fake_esp32` deseninde.
- **RTK düzeltmesi:** Pi'da bir NTRIP istemcisi TUSAGA-Aktif'e bağlanır ve
  RTCM'i USB'den F9P'ye yazar. İnternet yoksa F9P RTK'sız çalışır (1–2 m
  civarı); bu bile NEO-6M'den iyi, ve kovaryans durumu doğru yansıtır. PPK yolu
  bundan bağımsız, orchadOS'ta aynen kalır.
- **`gps_selector`:** NEO-6M sürücüsü `/gps/neo6m/fix`'e yayınlar. Seçici, o anki
  moda göre ya onu ya `/kit/gnss/fix`'i `/gps/fix`'e aktarır. `navsat_transform`
  ve `ekf.yaml` değişmez. Antenin kaldıraç kolu header'daki frame'den gelir
  (navsat_transform bunu TF'ten okuyor; sim'deki "anten değil base_link" hatası
  burada da tuzak).
- **Zaman:** PPS pini + chrony ile Pi'ın saati GPS zamanına kilitlenir. Robotun
  MCAP damgaları ile kitin UBX/`events.csv` kayıtları aynı saati paylaşır.

**v2 (`gnss_link: ros`):**
- Kit Pi 5 ROS 2 Jazzy çalıştırır. LiDAR'ı, F9P'yi ve PTP'yi o yönetir. Kit
  robotsuz da aynı şekilde çalışır.
- Robot Pi 4 ile kit Pi 5 soketin Ethernet çiftleri üzerinden bağlanır. Sabit
  IP'ler kullanılır, aynı `ROS_DOMAIN_ID`, DDS sadece bu arayüzle sınırlıdır.
  Kit `/kit/gnss/fix`, `/kit/lidar/points` ve `/kit/state_detail` yayınlar.
- PPS pini v2'de de bağlıdır, robotun saati yine doğrudan GPS'e kilitlenir.
- Bant genişliği: LiDAR ~4 MB/s ≈ 32 Mbit/s. 100 Mbit yetiyor ama ham bulut
  yerine seyreltilmiş bir bulut aktarmak daha güvenli.

## 360° kamera — Camera SDK ve Media SDK

Kaynak: Insta360'ın geliştirici belgeleri (github.com/Insta360Develop,
`Insta360-Developer_Docs`, Camera SDK V2.1.8 / Media SDK V3.1.5, 2026-08-28).
SDK'nın kendisi başvuruyla geliyor ve **repoya girmez**: lisansı belgede yazmıyor.
Repo dışında `~/.robot/sdk/` altında durur. Yoksa `kit_camera` düğümü
`cam360_ctrl`'ü kapatır ve robot çalışmaya devam eder.

**Belgeye göre cevaplar** (hiçbiri gerçek kamerayla denenmedi):

| Soru | Cevap | Tasarıma etkisi |
|---|---|---|
| Pi'da çalışır mı? | **Evet, belgeye göre.** Ubuntu 22.04 x86-64 ve **aarch64** destekleniyor; belgede Raspberry Pi de anılıyor | Robotta Ubuntu 24.04 (Jazzy) var; 22.04 için derlenmiş kütüphane büyük olasılıkla çalışır, ama ilk denenecek şey bu |
| Bağlantı | **Yalnız USB.** Kamera "Android" USB moduna alınmalı (X4/X5'te bağlanınca ekranda seçiliyor). Wi-Fi kontrolü ayrı bir protokolden (OSC) yapılıyor | Soketteki USB + kit içindeki hub tasarımı doğru. Linux'ta USB izni gerekiyor; `sudo` yerine bir udev kuralı yazılır (USB vendor `0x2e1a`) |
| Kayıt kontrolü | `StartRecording` / `StopRecording`, çözünürlük ve bit hızı ayarı, kart durumu, pil, **ısınma, kart dolu ve "kayıt durdu" bildirimleri** | `kit_camera` kaydı başlatır, doğrular ve izler. Isınma ya da kayıt durması görevi duraklatır |
| USB koparsa kayıt? | Kayıt varsayılan olarak **durmuyor** (`EnableAutoStopRecordAfterDisconnet` kapalı) | Fiş çıkınca kayıt sürer. Görev durur ama veri kaybolmaz |
| Saat | `SyncLocalTimeToCamera(utc, tz)` ve `GetCameraMediaTime()` | Robot kameranın saatini PPS'e kilitli Pi saatinden ayarlar ve kayıt boyunca (Pi zamanı, kamera zamanı) çiftleri kaydeder. **Robotta QR saat adımı gerekmez.** Bu yolun hassasiyeti bilinmiyor; ince eşleme jiroskopla yapılmaya devam eder |
| Canlı görüntü | `StartLiveStreaming`: tipik olarak **1920×960, 30 fps**, H.264/H.265, birleştirilmemiş iki balıkgözü görüntü. Aynı akışta **jiroskop** ve pozlama verisi de geliyor | Kitin kamerasıyla algı mümkün (aşağıda). Canlı jiroskopla ince eşleme sahada da yapılabilir |
| Kayıt sürerken canlı görüntü? | **Belgede yazmıyor** | Algının tamamı buna bağlı. K3'te ilk denenecek şey bu, ısınmayla birlikte |
| Media SDK | Birleştirme (stitching); V3.1.5'te kayıt dosyasının sonundaki bilgiden **IMU verisini çıkaran** bir araç geldi | orchadOS'un işleme istasyonunda kullanılır. İnce eşleme için kayıttan jiroskop verisi buradan alınır. Robot reposunu ilgilendirmez |
| Model | X5 ve X6 destekleniyor | Kit siparişindeki "güncel Insta360 X" seçimi SDK açısından uygun |

**v1'de kamerayı robot sürer** (soketteki USB'den, `kit_camera` düğümüyle).
v2'de kitin Pi 5'i sürer. Robotsuz v1 kitte kamerayı süren bir bilgisayar
yok; personel kaydı elle başlatır ve QR adımı orada kalır. Robot görev başında
ve sonunda yerinde kısa bir yalpalama yapar; böylece kameranın jiroskobu ile
robotun IMU'su arasında net bir ortak işaret oluşur.

## Kitin kamerasıyla algı (kitli modda ek bir göz)

Kitin kamerası robotun temel algısının **yerine geçmez**. Robot kitsiz de
çalışmalı; GPS, IMU, Pi Kamerası ve tampon aynen kalır. 360° kamera kitli
modda **ek bir katmandır**, ve yalnız kayıt sürerken canlı görüntü
alınabiliyorsa çalışır. Çakışmada **orchadOS kaydı kazanır**: canlı akış
kaydı bozuyor ya da kamerayı ısıtıyorsa kapatılır.

| Kullanım | Ne kazandırır |
|---|---|
| **Gövdelere göre konum** | Görülen gövdelerin yönleri ağaç haritasıyla eşleştirilir. RTK yokken ya da ağaç altında bozulduğunda konum ayakta kalır. Kaba hesap: 1920×960'ta ~0,2°/piksel; 4 m'deki 15 cm'lik gövde ~11 piksel. Yön 4 m'de ~1,5 cm yanal hassasiyetle ölçülür. Mesafe gövdenin yere değdiği noktanın açısından ~3–4 cm ile ölçülür; bu düz zemin varsayımıyla, eğimde hata büyür |
| **Satır takibi** | Satırın ortası GPS'ten değil gövdelerden bulunur |
| **Ağaç etrafında dönme (6f)** | 6f'de politika öğrenemedi, çünkü dönüş sırasında ağaç 62°'lik ön kameranın dışında kalıyordu (learning.md). 360° kamera ağacı turun tamamında görür |
| **Dal uyarısı** | Kamera direğin tepesinde. Öndeki dalı görüp robotu durdurabilir; açık kalan "direk dala takılır" riskine karşı |
| **İnsan algılama** | Yandan ya da arkadan yaklaşanı fark eder. Bir güvenlik katmanı sayılmaz, erken yavaşlama içindir |

**Sınırlar:** Robotun gövdesi tam altı kapatıyor, küçük taşlar düşük
çözünürlükte kaybolur, yani tamponun yerini tutmaz. Derinlik yok. Pi 4'te
ivmeleyici olmadığı için gövde tespiti saniyede birkaç kare olur; bu, konum
düzeltmesi için yeterli. v2'de engel algısının asıl kaynağı LiDAR olur.

## Ağaç haritası: bir başlangıç tahmini, kesin bilgi değil (kullanıcı kararı)

Haritadaki gövde konumlarının garantisi yok: ağaç kurur, kesilir, yeni fidan
dikilir, envanterde hata olur. Bu yüzden:

- Robot haritayı **"ağaçlar muhtemelen burada"** diye kullanır ve gördükleriyle
  karşılaştırır. Eşleşmeyen gövdeler konum hesabına girmez: haritada olup
  görünmeyen ağaç da, haritada olmayan yeni gövde de.
- Bu farklar **bulgu olarak orchadOS'a gider** ("bu ağaç yok", "burada yeni bir
  gövde var"). orchadOS'un periyodik ziyaretlerde istediği değişim tespiti budur.
- **Robot asıl haritayı sahada değiştirmez.** Asıl harita orchadOS'ta uzman ve
  sahip onayından geçen haritadır. Robotun çıkardığı her harita bir taslaktır.

## Envanter sırası (kullanıcı kararı)

İlk ziyarette harita yok, yani gövdeler robot için **survey edilmemiş
engeller** (karar 12): robot onları göremez ve içlerine sürer. Robotla ilk
envanter, gövdeleri canlı olarak görmeyi gerektirir.

1. **İlk bahçeler: orchadOS'taki plan aynen.** Envanter sırtta ya da motosikletle
   taranır, ofiste tıklanarak üçgenlenir. Tıklamalar gövde modelinin eğitim
   verisi olur.
2. **Robot periyodik ziyaretlere girer.** Onaylı harita var. Robot onu başlangıç
   tahmini olarak kullanır ve değişimleri bildirir. v1 kitle de olur.
3. **Robotla ilk envanter** iki şeyden biri gerçekleşince açılır:
   - LiDAR'lı kit (v2): 0,7–1 m'deki bulut diliminde silindir oturtma;
     makine öğrenmesi gerektirmez. En sağlam yol.
   - Gövde modeli canlı tespit için yeterince olgunlaşır (v1, canlı görüntü
     şartıyla).
   Bu durumda bile envanterin onayı ofiste yapılır.
4. **Robotla envanterde bile insan gereken iki adım kalır:**
   - **Sınır:** Sahibin göstermesi gerekiyor. Operatör robotu sınır boyunca
     sayfadan sürer (öğret-kaydet, 5b). Kitin RTK'sıyla sınır santimetre
     hassasiyetinde çıkar.
   - **Satır yönü:** Uydu görüntüsünden kaba satırlar çizilir (web arayüzünde
     haritadan çizim var).

## Robot tarafında ne değişir

| Parça | Değişiklik |
|---|---|
| **Yeni paket `robot_kit`** | `kit_manager` (pin, çip, durum makinesi, statik TF), `kit_gnss`, `gps_selector`, `ntrip_client`, `kit_camera` (Camera SDK); `fake_kit` (sahte pin + çip + F9P akışı + sahte kamera) |
| `robot_bringup` | URDF'e `kit_mount` (x = 0,11, z = 0,1825); `sensors.launch.py`'de NEO-6M `/gps/neo6m/fix`'e; v2 için Nav2'ye LiDAR engel katmanlı ikinci bir profil |
| `robot_missions` | `orchard_grid` (iki yönlü koridor) ve `tree_closeup` (`orbit_path` ile liste); `requires: [gnss_rtk, cam360]`. Kit yoksa planlayıcı reddeder |
| Web arayüzü | Kit kartı: durum, seri no, caps, RTK durumu (fix/float/yok), PPS kilidi, kamera kaydı |
| `robot_sim` | `kit:=none\|v1\|v2`: RTK kalitesinde GPS, Gazebo'da direk kütlesi (devrilme payının ölçülmesi), v2'de sahte LiDAR |
| `hoverboard_bridge`, ESP32 firmware, seri protokol | **Değişmez.** Hız limitleri, watchdog ve deadman aynen kalır |

**Kit tarafında (orchadOS repo'sunda) ne değişir:** fiş kablosu, `KIT_PRESENT`
köprüsü, DS2431 çipi, çipe manifest yazan araç, direğin yüksekliğinin robota
göre ayarlanması, akünün direk dibinde olması; v2'de Pi 5'in topic sözleşmesi.

## Repolar arası sözleşme

- Asıl kaynak bu belge (`hoverboard/docs/kit-soketi.md`). orchadOS'taki
  `docs/arayuzler.md` bu belgeye bağlantı verir ve uyduğu sürümü yazar.
- Manifestin ikili formatı **iki repoda** kodlanır: robotta okuyan taraf,
  orchadOS'ta yazan araç. İkisi aynı test vektörlerini (`kit_manifest_v1.json` +
  beklenen baytlar) kullanır. Biri değişince diğeri de değişir; seri protokoldeki
  kuralın aynısı.
- Şema numarası sadece artar. Robot bilmediği bir şemayı reddeder ve kitsiz moda
  geçer, tahmin etmez.

## Aşamalar

| Aşama | Ne | Donanım |
|---|---|---|
| **K0** | Bu belge | yok |
| **K1** | `robot_kit` + `fake_kit`; durum makinesi, `gps_selector`, görev kapısı, web kartı; CI'da: tak → mod → kaynak değişimi → görev ortasında çıkar → robot durur | yok |
| **K2** | `robot_sim` `kit:=v1`: kitli modda bahçe grid taramasının kaplamasını NEO-6M ile karşılaştır; direk kütlesiyle devrilme payını Gazebo'da ölç; **gövde yönlerinden konum bulmanın** hatayı NEO-6M ve RTK'ya göre ne kadar düşürdüğünü ölç, haritaya **eksik ve fazladan ağaç** koyup konumun bozulmadığını ve değişimlerin bildirildiğini doğrula | yok |
| **K3** | Robotta plaka ve soket; kitte fiş ve çip (orchadOS). İlk denenecekler: Camera SDK'nın Pi'da çalışması, **kayıt sürerken canlı görüntü**, ısınma, USB'nin konnektörden geçişi, manyetometrede yön kontrolü | soket parçaları + kit |
| **K4** | v2: Ethernet + DDS, LiDAR engel katmanı | LiDAR'lı kit |

## K1: yapılanlar (`robot_kit`, donanımsız)

```
fake_kit / Pi pini + DS2431 ─► kit_manager ─► kit/state (latched), tf kit_mount -> kit_*
sim_node'un F9P'si / /dev/kit_gnss ─► kit_gnss ─► kit/gnss/fix ─┐
NEO-6M ───────────────────────────► gps/neo6m/fix ──────────────┴► gps_selector ─► gps/fix ─► navsat
TUSAGA-Aktif ─► ntrip_client ─► kit/gnss/rtcm ─► kit_gnss ─► F9P
```

- **ROS'suz çekirdek, hepsi testli:** `manifest.py` (çip formatı + test
  vektörleri `robot_kit/test/vectors/`), `state.py` (durum makinesi), `ubx.py`
  (NAV-PVT; **CFG yazacak kod yok**), `rtcm.py`, `ntrip.py` (VRS için GGA,
  NTRIP 2 chunked, "SOURCETABLE 200" hata sayılıyor), `f9p_sim.py` (sahte F9P:
  RTCM gelince float, 8 sn sonra fixed; kendisine yazılan her CFG'yi sayıyor),
  `hw.py` (gpiod + w1 sysfs; sahteleri dosya).
- **Düğümler:** `kit_manager`, `kit_gnss`, `gps_selector`, `ntrip_client`;
  `fake_kit` (plug/unplug/blank/gnss/caster). `robot.launch.py use_kit:=true`
  ile açılıyor (varsayılan kapalı; robotta Adım 9, `docs/deployment.md`).
  `use_kit` açıkken NEO-6M `gps/neo6m/*`'e taşınıyor.
- **sim_node:** `kit_gnss_link` ile kitin F9P'sini bir pty'de koşturuyor.
  Antenin yeri `base_link -> kit_gnss` TF'inden, yani sahte çipten okunan
  kalibrasyondan geliyor; zincirde bir hata yanlış konum olarak görünür.
- **Ölçülen** (`test_kit_stack.py`, kinematik sim, 2026-10-25): robotun `map`'teki
  konumu ile gerçek lat/lon'unun `map`'e düştüğü yer arasındaki fark (datum
  hatası iki tarafta da var, sadeleşiyor):

  | | dururken | 0,3 m/s |
  |---|---|---|
  | NEO-6M (sim gürültüsü) | 0,99 m | — |
  | Kit, RTK fixed | **0,012 m** (en kötü 0,017) | 0,019 m (en kötü 0,036) |

  Aynı test: GPS kaynağı robot hareket ederken değişmiyor, durunca değişiyor;
  boş çip kit sayılmıyor; robot F9P'ye hiç CFG yazmıyor (sahte alıcı sayıyor).
- ⚠️ Bu sayılar sahte bir F9P'nin "iyi gün" gürültüsü. Ağaç altında RTK'nın ne
  yaptığı bir saha ölçümü (K3).
- **Görevler** (`robot_missions/kit_gate.py`): görev tipleri `requires` ile kit
  isteyebiliyor (bahçe taraması K2'de). Kit yoksa, eksikse ya da robot henüz
  kitin GPS'ine geçmediyse görev **başlatılmıyor** (resume de). Robot kitin
  GPS'iyle konumlanırken fiş çıkarsa ya da kitin GPS'i susarsa çalışan görev
  **duraklıyor**, robot duruyor, GPS robot durunca NEO-6M'e dönüyor; devam
  kararı insanın. Ölçülen (`test_mission_kit.py`): fiş → PAUSED 0,69 sn
  (0,5 sn'si debounce), → robot durdu 0,88 sn.
- **Web:** başlıkta kit rozeti (seri no, RTK durumu ve hAcc, GPS kaynağı; üstüne
  gelince donanım, eksik veri, NTRIP). Görev formu "Kit gerekir" diyor.
- **360° kamera** (`camera.py`, `kit_camera`): SDK repoya girmediği için robot
  kamerayla ayrı bir **shim** programı üzerinden JSON satırlarıyla konuşuyor;
  SDK'daki bir çökme ROS düğümünü düşürmüyor ve SDK'sız robot da derleniyor.
  `kit_camera` bağlanınca kameranın saatini robotun saatinden ayarlıyor,
  kayıt sürerken (robot ms, kamera media time) çiftlerini `time_log`'a yazıyor
  (robotta QR adımının yerine). Kamera kendi şikâyetlerini (ısınma, kart dolu,
  kayıt durdu) `fault`'ta tutuyor. `fake_camera_shim` aynı protokolü sahte bir
  kamerayla konuşuyor (ısınıp duruyor, kartı doluyor).
  ✅ **Gerçek shim yazıldı** (`robot_kit/shim/kit_camera_shim.cpp`, SDK 2.1.8):
  SDK repoya girmiyor, shim `scripts/build_camera_shim.sh <SDK dizini>` ile
  `~/.robot/sdk/` altına derleniyor (kit_camera varsayılan olarak oraya
  bakıyor; kütüphane yanında, rpath `$ORIGIN/lib`). Pi için paketteki
  **aarch64** (gcc-arm-11.2) SDK'sı kullanılır; betik mimari uymazsa durur.
  Bağlanınca `EnableAutoStopRecordAfterDisconnet(false)` (fiş çıkınca kayıt
  sürer), ısınma / kart dolu / pil / kayıt durdu bildirimleri olay olarak
  geliyor, saat `SyncLocalTimeToCamera` (saniye, SDK başlığına göre).
  Doğrulanan: x86-64'te derleniyor ve gerçek SDK'yla protokolü konuşuyor
  (kamera yokken "no camera found"; `test_camera.py`, `KIT_CAMERA_SHIM` ile).
  ⚠️ **Gerçek bir kamerayla hiç denenmedi**; Pi'da da derlenmedi.

## K2: yapılanlar (simülatör)

- ✅ **K2a — `orchard_grid` görevi** (`docs/missions.md`, aynı adlı bölüm):
  ağaçlar sahada `ağaç` poi'leri; koridorlar sıraların ortasında, iki yönde.
  Kit kapısı, kameranın kayıt başlat/durdur ve arıza duraklatması runner'da.
  Ölçülen (`test_orchard_stack.py`, 3×2 ağaç, 5 m aralık, RTK + sahte kamera):
  kitsiz başlatma reddedildi; tarama 6 dk, 6 ağacın hepsi dört yönden geçildi,
  temas yok, kamera tarama boyunca kaydetti ve sonunda durdu.
  ⚠️ **Yolda bulunan:** odom'a çapalı satırlar RTK'da bile pusula hatası
  kadar eğik gidiyordu (16 m'de 0,8–1 m); RTK fixed'da satırlar artık `map`'te
  sürülüyor, sapma p95 0,095 m.
- ✅ **K2b — devrilme payı** (`kit_model.py` + `test_gazebo_kit.py`, Gazebo
  aşaması): elle hesap ve fizik tutarlı; eğimde zayıf taraf arka (16,4°), sert
  kalkışta kitle ön tekerlekler kalkıyor (ayrıntı "Soket — mekanik"). İvme
  limiti kararı açık.
- ✅ **K2c — gövdelere göre konum + ağaç denetimi** (`robot_kit/trunk_localizer.py`,
  `trunk_localizer` düğümü, `test_trunk_stack.py`):
  - Kameranın her karesindeki gövde yönleri (ve gövde dibi görünüyorsa
    eğim açısından mesafe) sahadaki `ağaç` noktalarıyla eşleştiriliyor. Önce
    EKF'in konumu etrafında kaba ve ince bir hipotez araması, sonra birebir
    eşleştirme, Gauss-Newton ile (x, y, yaw), en son tek tek aykırı eşleşmeler
    atılıyor. Sonuç `kit/trunk_pose` olarak `ekf_global`'a giriyor (x, y **ve
    yaw**: pusuladan gelmeyen tek yön bilgisi). EKF ayrıca 5 Mahalanobis'ten
    uzak bir düzeltmeyi reddediyor.
  - **Reddedilen kareler:** 4'ten az gövde, büyük artık, ya da **belirsizlik**
    (bahçe periyodik: iki yer neredeyse aynı skoru alırsa kare kullanılmıyor;
    yanlış bir düzeltme robotu bir sıra öteye, gövdelerin yanına koyardı).
  - **Harita yalnız başlangıç tahmini:** eşleşmeyen gövde konumu çekmiyor.
    `kit/tree_audit` orchadOS için iki şey bildiriyor: haritada olup defalarca
    görülmesi gerekirken görülmeyen ağaçlar (`missing`, ağaç kodlarıyla) ve
    haritada olmayan ama birkaç farklı yerden aynı noktada görülen gövdeler
    (`new`, `map` ve lat/lon). **Harita robotta değişmiyor.**
  - Sahte dedektör (`trunk_sim.py`): menzil 12 m, robotun gövdesinin
    kapattığı alt bölge, yakın gövdenin arkasındakini örtmesi, yön/eğim
    gürültüsü 0,3°, %5 kaçırma, %10 gövde dibi görünmez, ara sıra yanlış tespit.
  - **Ölçülen** (kinematik sim, 4×3 ağaç, 6 m aralık, kitin F9P'si RTK'sız
    tek noktada, sahada bir ölü ağaç ve bir yeni dikilmiş ağaç; 3 koşu):

    | | `map` hatası ortalama | p95 |
    |---|---|---|
    | Gövdeler kapalı (F9P tek nokta) | 0,75 m | 1,6 m |
    | **Gövdeler açık** | **0,05 m** | **0,14–0,18 m** |

    Denetim üç koşuda da ölü ağacı (`S1-3`) ve yeni gövdeyi (gerçek yerinden
    ~0,4–0,8 m) buldu. ⚠️ Aykırı ayıklama eklenmeden önce bir koşuda p95
    0,45 m'ye çıkmıştı: yeni gövde haritadaki komşusuyla eşleşip çözümü
    büküyordu.
  - ⚠️ **Gerçek dedektör yok.** Gerçekte bu tespitleri 360° kameranın canlı
    görüntüsünde çalışan bir gövde modeli üretecek (orchadOS'un ofis
    tıklamalarıyla eğitilecek) ve kayıt sürerken canlı görüntünün alınabildiği
    SDK'da doğrulanmadı. Pi 4'te süre: masaüstünde kare başına ~30 ms;
    Pi'da ölçülecek (`max_rate_hz` 2).
- ✅ **K2d — RTK'sız bahçe taraması, gövdelerle** (`test_orchard_stack[trunks]`):
  gövde düzeltmeleri tazeyken (son 30 sn, σ ≤ 0,1 m) satırlar RTK'daki gibi
  `map`'te sürülüyor. Ölçülen (3×2 ağaç, F9P tek nokta, NTRIP yok):
  koridordan sapma p95 **0,23 m** (RTK'da 0,095), konum hatası ortanca
  0,06 / p95 0,33 m, 24 ağaç yüzünün 23–24'ü görüldü, temas yok.
  ⚠️ **Kararlı değil:** sonraki koşularda 0,61–0,79 m, CI'da 0,72 m (konum
  p95 0,50 m, 20/24 yüz). Bu yüzden testte bu durum yalnız ölçülüyor; kapı
  (assert) yalnız güvenlik için (görev biter, temas yok). Kök neden aşağıda,
  açık.
  ⚠️ **Yolda bulunanlar:**
  - Düzeltme bahçenin kenarında kesilince (4'ten az gövde) tahmin 1–2 sn
    içinde tek noktalı F9P'nin 1–2 m'lik sapmasına kayıyordu (EKF onun
    yavaş sapmasını beyaz gürültü sanıyor). En az eşleşme 4 → **3** (3 yön +
    eğim = 6 denklem, 3 bilinmeyen): sapma p95 0,6–0,8 m'den 0,23 m'ye.
    Sapan düzeltme sıfır kaldı (birim testleri ve 420 karelik kontrol).
  - `ekf_global`'da gövde pozuna konan **red eşiği (5) kaldırıldı**: sıkı
    kovaryansla F9P'nin sürüklediği tahmin bir sonraki doğru düzeltmeyi
    "uzak" sayabiliyordu. Yanlış düzeltmeye karşı koruma yerelleştiricinin
    kendi kontrolleri (belirsizlik, artık, aykırı ayıklama).
  - Kenarda düzeltme yokken F9P'nin çekişi: çözüm tek noktalı F9P'nin
    kovaryansını gövde düzeltmeleri varken büyütmek olurdu. **Kullanıcı
    kararı (2026-10-26): gerek yok**, kenarlardaki kayma önemli değil; EKF
    ayarı yapılmıyor. Gerçek bahçeler bu 3×2'den büyük, kenarın payı küçük.

## Açık kalanlar

- ✅ ~~Kitli modda ivme limiti~~ — yapıldı (hızlanma rampası, yukarıda).

- **Kayıt sürerken canlı görüntü alınabiliyor mu?** SDK belgesi söylemiyor.
  Kitin kamerasıyla algının tamamı buna bağlı; K3'te denenir.
- **Ağırlıklar tahmin.** Robotun kütlesi URDF'te bile `TODO: weigh it`. Devrilme
  tablosu robot ve kit tartılınca yeniden hesaplanmalı.
