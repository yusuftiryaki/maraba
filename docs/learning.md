# Öğrenme modu: taklit ile öğrenme için veri kaydı

> Durum: **5c ve 5d yapıldı** (2026-10-18) — kaydedici, sayfa, dışa aktarma;
> ölçüldü (aşağıda). Sıradaki: algoritma seçimi ve politikanın twist_mux'a
> takılması. 5a/5b `docs/missions.md` "5. aşama"da.

## Ne istiyoruz
Operatör sayfada "şu görevi yapacağız, öğrenme modundasın" der, robotu sürer
(ya da otonom görevi izleyip gerektiğinde araya girer); robotun **tüm durumu ve
sensörleri** bir bölüm (episode) olarak kaydedilir. Bölümler sonra laptopa
alınıp bir taklit ile öğrenme (IL) algoritmasına verilir.

**Öğrenilecek şey (kullanıcı, 2026-10-18): tüm görev** — kenar dibinde biçme,
ağacın etrafından dolaşma, yabani ota yönelme gibi davranışlar. Yani politika
büyük ihtimalle **kamera + durum → hız komutu**. Bu yüzden kamera kaydın
merkezinde; algoritma bugün seçilmiyor, kayıt ona bağlı değil.

## Kararlar
1. **Ham kayıt rosbag2 + MCAP, eğitim formatı ayrı ve çevrimdışı.** MCAP
   container'da ve Pi'da apt'te var, ROS olmadan da okunuyor (`mcap` Python
   kütüphanesi). Bag kayıpsız ana kaynak: bugün hangi gözlemi, hangi frekansı
   isteyeceğimizi bilmiyoruz; dışa aktarma (5d) her seferinde bag'den yeniden
   üretilir. Pi'da dönüştürme yapılmaz.
2. **Kaydedici robotu asla etkilemez.** Yalnızca abone olan ayrı bir süreç
   (`ros2 bag record`, `episode_recorder` düğümü başlatır/durdurur, SIGINT ile
   düzgün kapatır). Disk eşiğin altına inerse kayıt durur — robot durmaz.
   Motor komut yolunda hiçbir şey değişmez.
3. **Aksiyon etiketi = kim, ne istedi, ne oldu — üçü ayrı.** IL verisinin en
   sık bozulduğu yer burası: "operatör ne istedi" ile "tekerleğe ne gitti"
   arasında mux, hız limitleri, tampon vetosu, batarya kilidi var.
   | topic | anlamı |
   |---|---|
   | `/cmd_vel_teleop` | operatörün niyeti (**birincil etiket**) |
   | `/cmd_vel_nav` | Nav2'nin önerisi (otonom bölümlerde etiket) |
   | `/cmd_vel` | mux'tan çıkan, köprüye giden |
   | `teleop/state` | o an kira var mı, teleop aktif mi (5a) |
   | `nav_lock`, `bumper`, `diagnostics` (E-stop) | komutu kim engelledi |
   | `odom`, `joint_states` | tekerleğin gerçekte yaptığı |
4. **Üç bölüm modu:** `demo` (operatör sürüyor), `intervention` (otonom görev
   koşuyor, operatör gerektiğinde araya giriyor — DAgger tarzı yöntemlerin en
   değerli verisi, araya girdiği anlar `teleop/state`'ten çıkar), `autonomous`
   (sadece izleniyor). Mod bölümün meta verisinde.
5. **Bağlam bölümle birlikte saklanır** (`episode.yaml`, bag dizininde). B4'te
   kalibrasyon sabitleri değişecek; hangi sabitlerle kaydedildiği bilinmeyen
   bölüm o gün çöpe döner. İçerik: görev etiketi, mod, operatörün notu, sonuç
   (başarılı / başarısız / iptal, bitişte sorulur), saha dosyasının kopyası, git
   sha, `robot_description`, köprü/EKF/kamera parametreleri, sim mi gerçek mi
   (sim'deyse dünya tohumu).
6. **Bölüm içi işaretler:** kayıt sırasında sayfadaki "işaret" düğmesi
   `learning/marker`'a etiketli bir an yazar ("ağaç başladı", "yabani ot").
   "Tüm görev" tek bir bölümde birçok beceri içeriyor; işaretler sonradan
   bölütlemeyi (segmentation) mümkün kılar.
7. **Sim de aynı kaydediciyi kullanır.** Orada uzman bedava: planlayıcı + Nav2
   istendiği kadar bölüm üretir, ground truth da kayda girer (`ground_truth`,
   `collision_truth`). Algoritma denemeleri gerçek veri beklemeden başlar.
   ⚠️ Kinematik dünyanın kamerası (`sim_camera.py`) gerçekçi değil; görüntüden
   öğrenen bir politika ancak Gazebo kamerasıyla ya da gerçek veriyle anlamlı.
8. **Kamera varsayılan açık**, gizlilik kısıtı yok (kullanıcı, 2026-10-18).

## Kaydedilen topic'ler
Profil bir yaml'da (`config/learning_topics.yaml`); yeni sensör = yeni satır.
- **Kamera:** `camera/image_raw/compressed` (JPEG). Ham görüntü kaydedilmez:
  640×480 rgb8 30 Hz ~27 MB/s. JPEG 640×480 ~50 KB × 15 Hz ≈ **0.75 MB/s ≈
  2.7 GB/saat**. Kaynağı: `compressed_image_transport` eklentisi (camera_ros
  kendiliğinden `/compressed` yayınlar) — sim kameraları (Gazebo,
  `sim_camera.py`) ham yayınlıyor, onlara `image_transport republish`.
- **Durum:** `imu/data_raw`, `imu/data`, `imu/mag`, `gps/fix`, `gps/filtered`,
  `odometry/filtered/local`, `odometry/filtered/global`, `odometry/gps`, `odom`, `joint_states`, `/tf`,
  `/tf_static`, `battery`, `bumper`, `diagnostics`.
- **Aksiyon ve bağlam:** karar 3'teki tablo + `mission_runner/status`,
  `battery_supervisor/state`, `obstacle_map` (latched), `learning/marker`.
- **Sim'de ek:** `ground_truth`, `collision_truth`.
- İleride: misina motoru komutu (karar 2'deki ikinci aktüatör) bir aksiyon
  olarak eklenecek — biçme açık/kapalı da öğrenilecek şeyin parçası.

## Depolama ve dayanıklılık
- **USB 3 SSD** (SATA SSD + UASP'li USB 3 kutu, ≥ 256 GB ≈ 90 saat kamera).
  SD kart hem yavaş hem yazmayla aşınıyor; kayıt dizini SSD'de, OS SD'de kalır.
  ⚠️ Pi 4'ün USB portları toplam ~1.2 A veriyor; SSD'nin çekişi ölçülmeli,
  gerekirse ayrı beslemeli hub. Pi powerbank'ten beslendiği için (karar 13)
  E-stop Pi'ı kapatmaz, kayıt E-stop'tan etkilenmez.
- MCAP `--storage-preset-profile zstd_fast` (parça içi sıkıştırma),
  `--max-bag-duration 60`: güç kesilirse en fazla son parça kaybolur. Bu bir
  testle kanıtlanacak (SIGKILL ortasında → önceki parçalar okunur).
- Boş alan eşiği (ör. 5 GB) altında yeni bölüm başlamaz, süren bölüm düzgün
  kapatılır; sayfada kalan süre gösterilir.

## Donanım notu (kullanıcı: "istediğimiz donanımı takabiliriz")
- Tek Pi Camera V2 (62° yatay) docking için 5° aşağı bakıyor; zemin 41 cm
  ileriden başlıyor. **Kenar dibinde biçme ve yabani ota yönelme yakın zemini
  ister** — muhtemelen ikinci, geniş açılı, aşağı bakan bir kamera gerekecek.
  Pi 4'te tek CSI var → ikincisi USB kamera. Karar ilk gerçek bölümlere bakarak
  verilmeli; kaydedici profili kamera sayısından bağımsız.
- Derinlik kamerası (missions.md, "Robotun kendi tespit ettiği engeller")
  gelirse aynı profile bir satır olarak girer.

## Sayfada (5c)
"Öğrenme" sekmesi: görev etiketi (serbest + son kullanılanlar), mod, not →
**Kaydı başlat**; kayıt sürerken kırmızı REC, süre, boyut, kalan disk,
**İşaret** düğmesi; **Bitir** → sonuç + not. Bölüm listesi: indir (zip), sil.

## ✅ 5c: ne yapıldı (2026-10-18)
- **`robot_learning` paketi:** `episode.py` (ROS'suz: bölüm dizini,
  `episode.yaml`, `ros2 bag record` sürecini başlatma/durdurma, disk koruması,
  çökme tespiti), `episode_recorder` düğümü (`~/command` =
  `robot_missions_interfaces/Episode` start/stop/mark, `~/status` latched,
  `learning/marker`), `image_compressor` (`camera/image_raw` → `camera/jpeg`,
  ham karenin damgasıyla, **yalnız biri abone olunca**, ≤ 15 Hz).
- Profil: `robot_bringup/config/learning_topics.yaml` (32 topic; `sim`
  grubunda ground truth). `robot.launch.py`: `use_learning` (varsayılan açık,
  boştayken iş yapmaz), `episodes_dir`, `episode_split_s` (60),
  `learning_sim`. Pi'da `episodes_dir` SSD'de (deployment.md adım 8).
- Sayfada **Öğrenme** sekmesi (görev, mod, not → başlat; işaret; sonuç + not →
  bitir; liste: indir zip / sil) ve her sekmede görünen **REC** rozeti.
- Bölüm dizini: `episode.yaml` (görev, mod, sonuç, işaretler, `ready_after_s`,
  yazılım sürümü: git sha + kirli mi), `bag/` (MCAP, 60 sn'de bölünmüş),
  `recorder.log`, `site.geojson` kopyası, `params/<düğüm>.yaml`
  (`ros2 param dump`: köprü, EKF'ler, navsat, twist_mux, batarya… — B4'ten
  sonra hangi sabitlerle kaydedildiği buradan okunur).

Yolda bulunanlar (hepsi testli):
- ⚠️⚠️ **Kaydedici saniyelerce kör.** `ros2 bag record` başladıktan sonra
  konulara abone olması bu makinede **3–4 sn** sürüyor; ilk denemede
  ground truth'un ilk kaydı robot 0.55 m gitmişken geldi — gösterimin başı
  yoktu. Artık düğüm ROS grafiğinden "o an yayıncısı olan her profil konusuna
  kayıt süreci abone mi" diye bakıyor; olana kadar sayfa **○ HAZIRLANIYOR —
  henüz sürme** (+ beklenen konular) diyor, sonra **● REC**. `ready_after_s`
  `episode.yaml`'da. Pi'da bu süre daha uzun olacak — ölçülmeli.
- ⚠️ **Kayıt sürecinin çıktısı boşaltılmayan bir PIPE'a gitseydi** (ilk
  taslak) rosbag2 her bölmede log yazdığı için 64 KB dolunca saatler sonra
  sessizce donardı. Çıktı `recorder.log`'a.
- Kayıt süreci düğümden bağımsız (kendi oturumu) ama sahipsiz kalmaz: düğüm
  kapanırken SIGINT (sonuç `shutdown`), düğüm öldürülürse `setpriv
  --pdeathsig INT` ile çekirdek SIGINT yolluyor. (`preexec_fn` çok iş
  parçacıklı süreçte güvenli değil.)
- `ros2 param dump` düğüm başına ~1–2 sn CLI açılışı → paralel (4), bölümden
  bağımsız iş parçacığında; bölüm o sırada biterse sonuç dosyaya sonradan
  yamanıyor.

**Ölçülen** (`test_learning_stack.py`, gerçek yığın + kinematik dünya):
- Sayfadan başlat → **hazır 3–4 sn**; hazır beklenince sürüşün tamamı bag'de
  (ilk ground truth x = 0.000).
- 10 sn'lik bölümde 23 topic: `/cmd_vel_teleop` 100 (tam 0.30 m/s ve 0.5
  rad/s), `/cmd_vel` 100, `/teleop/state` ~80, `/odom` ~300–440, kamera JPEG
  ~30 (5 Hz sim kamerası; damgalar artan, çözülebilir, varıştan < 0.5 sn eski),
  işaret 1 (hem bag'de hem `episode.yaml`'da), `/tf_static` ve
  `/robot_description` (latched) 1'er.
- **Çökme:** kayıt süreci 10 sn'de SIGKILL → bölüm `crashed`, `metadata.yaml`
  yok (beklenen); `Reindexer` 3 bölmeden **5.7 sn**'yi geri getirdi (kapanmış
  bölmeler), sonraki bölüm normal başladı. Elle: `ros2 bag reindex <bag> -s mcap`.
- Tarayıcıda uçtan uca: başlat → HAZIRLANIYOR → REC → Sür sekmesinden sür →
  işaret → bitir → listede → zip 556 KB → sil. JS hatası yok.

Bilinmeyenler / sahada ölçülecek:
- Pi 4'te hazır olma süresi, CPU yükü (kayıt + JPEG kodlama 15 Hz) ve SSD yazma
  hızı. Gerçek kamera 30 Hz ham + 15 Hz JPEG.
- `systemctl stop` SIGTERM'ü tüm cgroup'a yollar; rosbag2'nin SIGTERM'de de
  dosyalarını kapattığı doğrulanmadı (kapatmazsa `reindex`).

## ✅ 5d: dışa aktarma (2026-10-18)
```bash
# bölümleri laptopa al (sayfadan zip ya da rsync), sonra dev container'da:
ros2 run robot_learning episode_export ~/episodes -o ~/dataset --fps 10 --image-size 320x240
#   --include-aborted   "iptal" sonuçlu bölümleri de al (varsayılan: alma)
#   --reindex           çökmüş bölümün bag'ini önce onar (metadata.yaml yoksa)
```
Çıktı (`format: hoverbot-episodes-v1`):
| dosya | içerik |
|---|---|
| `meta.json` | fps, sütun adları (`state_names`, `action_names`, `action_sources`), aksiyon kuralı; bölüm başına görev, mod, sonuç, işaretler (**kare numarası**), teleop oranı, yazılım sürümü |
| `episodes/episode_NNNNNN.npz` | `timestamp`, `state` (N×15, eksik = NaN), `action` (N×2 etiket), `action_source` (yok/teleop/nav), `action_executed` (`/cmd_vel`), `teleop` (insan tutuyordu), `image_age` |
| `videos/episode_NNNNNN.mp4` | satır başına bir kare (o andaki en son kamera karesi), mp4v |

Kurallar (`dataset.py`, elle hesaplanmış testlerle):
- Kareler **READY'den** başlar (öncesi kısmi). ⚠️ Gazebo'da bag'in saati sim
  saati, `started_unix` duvar saati — saatler uyuşmuyorsa bag'in ilk mesajı.
- Durum: her serinin kareden önceki **son** değeri; hiç gelmemişse NaN (0 değil).
- **Aksiyon = (t, t+1/fps] aralığında verilen komut** — gözlem k'ya verilen
  cevap; o aralıkta komut yoksa hâlâ taze (0.5 s, twist_mux'ın süresi) olan
  son komut. Teleop Nav2'yi ezer (mux gibi).
- Kamera karesi: damgası kareden önceki en son kare (damga = ışığın sensöre
  geldiği an, ham karenin damgası).

Neden parquet/LeRobot/HDF5 değil: algoritma seçilmedi (karar 1) ve
hiçbiri burada yeni bağımlılık olmadan yazılamıyor (Noble apt'inde pyarrow
yok). npz + mp4 her yerde açılır; seçilen eğiticinin formatına dönüştürücü
eğitim makinesinde küçük bir adım, ve bag kaynak olarak kalır — başka
seçimlerle istendiği an yeniden dışa aktarılır.

**Ölçülen** (`test_learning_stack.py`, kaydedilen gerçek bölüm): 68 kare (~70
beklenen), `action` tam 0.30 m/s ve 0.5 rad/s, kaynak yalnız teleop/yok,
**etiket tekerlekten önce**: komutun geldiği kare ile odometrinin hızlandığı
kare arası 0–5 kare (asla negatif değil — hizalama ters olsaydı politika
geleceği "görerek" öğrenirdi), işaret doğru karede, video kare sayısı = satır
sayısı.

## 6. aşama: beceri öğrenme — önce kenar takibi (plan, 2026-10-18)
Kullanıcıyla kararlaştırıldı: "tüm görev" tek politikaya verilmiyor. **Rota
planlayıcı + Nav2'de kalır** (şeritler, eve dönüş); öğrenme **yerel
becerilere** gider: kenar dibi takibi, engel etrafı, yabani ota yönelme.
`mission_runner` beceriyi kısa süreliğine çağırır, sonra kontrolü geri alır.
İlk beceri **kenar dibi takibi**: ölçütü en net olan, ve NEO-6M'le planlayıcının
kenar turu metrelerce ıskalıyor — kamera kenarı GPS'ten bağımsız görür.
Eğitim **bulutta** (Colab / Kaggle, GPU); robot ve dev container yalnız
veri üretir, dışa aktarır, çıkarım yapar.

Algoritma: önce **basit BC** (boru hattı taban çizgisi), sonra **ACT**
(aksiyon blokları, Pi'da koşma şansı), sol/sağ ortalaması sorun olursa
**Diffusion Policy** (çok modluluk; çıkarımı pahalı). Hepsi LeRobot'ta hazır.

**Sim'in uzmanı Nav2 değil, ayrıcalıklı betik:** ground truth'u ve gerçek
sınırı bilen bir kontrolcü (politika bunları görmez; kamera + durum görür).
Faydası: sınırsız gösterim ve **otomatik DAgger** — politika sürerken uzman
her duruma etiket verir (gerçekte bunu insan yapar: "müdahale" modu).

| dilim | ne | ölçüt / kanıt |
|---|---|---|
| **6a** | Ayrıcalıklı kenar uzmanı + kenar ölçütü (ground truth) + uzman bölümü kaydı; dışa aktarmada `action_expert` sütunu | uzman kendi ölçütünde: yanal hata cm'ler |
| **6b** | Kenar **görünür**: kinematik sim kamerasında çim / çim dışı (hızlı); rastgele sahalarla toplu bölüm üretimi | kamerada kenar; N bölüm |
| **6c** | npz → LeRobot dönüştürücü + Colab/Kaggle defteri (BC, ACT) | defter uçtan uca koşar |
| **6d** | `policy_node` + `policy_gate` (twist_mux önceliği 20) + sim'de kapalı çevrim değerlendirme + otomatik DAgger | politika kendi ölçütüyle puanlanır |
| **6e** | Gazebo render'lı dünyada gerçekçilik; sonra gerçek robotta **gölge mod** (komut üretir, tekerleğe gitmez) | sim→gerçek açığı |

### ✅ 6a: ayrıcalıklı kenar uzmanı (2026-10-18)
- `robot_sim/edge_expert.py` (ROS'suz): yol = sınırın `offset_m` (0.4) içeri
  çekilmişi; pure pursuit, ama **hedef keskin köşeyi geçmiyor** → robot köşeye
  girip yerinde dönüyor. Yalnız pure pursuit köşeleri yuvarlıyordu: dikdörtgen
  kenarın %16'sı hiç ulaşılmıyordu. `label(x, y, yaw)` saf: politikanın
  ulaştığı HER duruma etiket (6d DAgger). Tur, yay uzunluğuyla işaretli
  sayılıyor (ileri-geri titreme sıfır).
- `edge_metrics`: işaretli sınır uzaklığının `offset`'ten hatası (ortalama,
  p95, max), tolerans içi oranı, dışarı çıkma oranı, **sınırın ulaşılan oranı**.
  Uzman ve politika aynı ölçütle puanlanır.
- `edge_expert` düğümü: `/ground_truth` + sahanın alanı (sim'in GPS formülünün
  tam tersiyle sim_world'e — sim'in kendi gerçeği), `expert/cmd_vel` (her
  zaman, etiket) + `drive:=true` ise `cmd_vel_teleop` (insan gibi sürer),
  `~/start`, `~/stop`, `~/status`. Profil ve dışa aktarma: `action_expert`
  sütunu (sustuğu yerde NaN).
- **Ölçülen** (`test_edge_expert_stack.py`, gerçek mux + köprü + ESP32 sim +
  kinematik dünya, 6 × 4 m çim, offset 0.4): ortalama hata **1.9 cm**, p95
  6.5 cm, max 10 cm, dışarı 0, sınırın **%94**'ü (köşelerin kalanı yuvarlak
  misinanın erişemeyeceği yer: 0.4·√2 = 0.57 m > 0.5). Kayıt dışa
  aktarıldı: 652 kare, %99'unda uzman etiketi.

### ✅ 6b: kenar kamerada + rastgele sahalarla toplu bölüm (2026-10-18)
- `sim_camera`: `lawns` verilirse çim poligonunun içi çim, dışı **toprak**;
  `palette(seed)` renkleri ve dama boyunu sahaya göre değiştiriyor (domain
  randomization; çim yeşil, toprak kahve kalıyor — testli). `sim_node`:
  `lawn_file` (sahanın alanları), `ground_seed`.
- `site_world.py`: lat/lon ↔ sim_world, sim'in GPS formülünün tam tersi —
  uzman, kamera ve üreteç aynı "sim gerçeğini" kullanıyor (`EARTH_RADIUS_M`
  artık tek yerde).
- `lawn_gen.py`: orijin etrafında 4–7 köşeli, dışbükey ya da değil, rastgele
  dönük sahalar; **uzman ideal modelde temiz tur atamıyorsa saha reddediliyor**
  (sivri köşe < 55°, dar boğaz, robot kenara < 1 m, alan 15–70 m²) — kötü
  gösterim veriye sessizce girmesin.
- `ros2 run robot_sim edge_batch --lawns 20 --out ~/.robot/sim_episodes`:
  her saha için taze sim + Pi'daki `robot.launch.py` (öğrenme modu açık),
  uzman her yöne bir tur, bölüm kaydı; notunda seed, yön, GPS, sahanın WKT'si.
  Tur ~1–2 dk gerçek zaman: 20 saha × 2 yön ≈ 1.5 saat (gece işi).
- **Ölçülen** (`test_edge_batch_stack.py`, seed 8): bölüm başarılı, 317 kare,
  **kenar karelerin 312'sinde kamerada**, uzman etiketi %100; araç arkasında
  süreç bırakmıyor.
- ⚠️ **Kamera kenar takibi için iyi yerde değil.** Yanaşma için 5° aşağı
  bakıyor: karenin üst ~%40'ı gök, misinanın çalıştığı yakın zemin alt
  şeritte. Kenar ilerideki haliyle görünüyor, yanındaki haliyle zor. İkinci,
  aşağı bakan geniş açılı kamera (donanım notu, yukarıda) burada ölçülebilir:
  aynı sim'de ikinci kamera + aynı uzman, iki politikanın ölçütü yan yana.

### ✅ 6c: LeRobot'a dönüştürme + Colab/Kaggle defteri (2026-10-18)
`learning/` (ROS'suz; bulutta koşar):
- `to_lerobot.py`: `episode_export` çıktısı → **LeRobot 0.6.1** veri seti
  (`LeRobotDataset.create` → `add_frame` → `save_episode` → `finalize`;
  kamera AV1 video). Durum varsayılanı **yalnız odometri + IMU** — harita/GPS
  konumu politikaya tek sahanın koordinatlarını ezberletir, kenarı kamera
  görmeli. Etiket: sim'de uzman (`action_expert`), gerçekte insan. Baştaki
  görüntüsüz/etiketsiz kareler kırpılır, aradaki boşluk son değerle dolar.
- `eval_offline.py`: ayrılmış bölümlerde `select_action` (robotun çağıracağı
  gibi, bloklar dahil) vs etiket: sütun başına MAE + çıkarım süresi.
  ⚠️ Açık çevrim: hata birikmesini göstermez (o 6d'de, sim'de).
- `train_edge.ipynb`: ortam kontrolü (Python ≥ 3.12, GPU) → kurulum → zip'i
  aç → dönüştür → eğitim/puanlama bölümlerini ayır → **ACT** eğit →
  puanla → (isteğe bağlı) Diffusion → robota götürülecek paketi zip'le.
  Betikleri GitHub'dan (`REPO_REF`) çeker. Veri: Colab'da Drive, Kaggle'da
  "Dataset". `dataset.zip`: `cd ~ && python3 -m zipfile -c dataset.zip dataset/`
  (container'da `zip` yok).

Sabitlenen sürümde **gerçekten koşuldu** (dev container, CPU, karalama
sanal ortamı; LeRobot CI'da değil, çok ağır):
- 2 uzman bölümü (6b, seed 7–8) → dönüştürme 22 sn → LeRobot'un kendisi
  yükledi (1288 kare, görüntü 3×240×320, durum 5, aksiyon 2) → `lerobot-train`
  ACT 20 adım (kayıp 59 → 8.3) → kontrol noktası → `eval_offline` →
  **defterin tamamı nbclient ile hücre hücre** (10 adım). Yani defterin
  kodu doğru; eğitim kalitesi GPU'da, gerçek adım sayısıyla görülecek.

**Kaggle'da gerçekten koşuldu** (2026-10-24, hesabın `hoverbot-edge-train-smoke`
özel defteri, 2 bölümlük veri, 200 adım): imaj **Python 3.13**, GPU **Tesla T4**
(16'lık grupla saniyede ~4.3 adım → 20 000 adım ≈ 80 dk), defterin tamamı
3.5 dk'da bitti, politika paketi (190 MB) ve çevrimdışı puan indirildi.
⚠️ Kaggle veri setine yüklenen zip'leri **kendisi açıyor** — defter artık
zip'i de açılmış dizini de tanıyor (Kaggle'da `meta.json`'dan buluyor).

Yolda bulunan tuzaklar (hepsi defterde/belgede önlendi):
- `lerobot[dataset]` eğitim için yetmiyor: `accelerate` ister →
  **`lerobot[dataset,training]`**. Python **≥ 3.12** şart; torch < 2.12.
- ⚠️ **ACT'nin varsayılan bloğu 100/100: 10 Hz'de 10 saniye kör sürüş.**
  Defter `chunk_size=20` (2 s ufuk), `n_action_steps=5` (0.5 s'de bir
  yeniden tahmin) kullanıyor.
- ⚠️ Diffusion'ın varsayılan `crop_shape`'i 84×84: 320×240 karemizden küçük
  bir parça. Defterde `[224, 288]`.
- GPU'suz ortamda `nvidia-smi` yok → ilk hücre çöküyordu (nbclient
  koşusunda yakalandı).
- **Model 52M parametre, paket 190 MB.** Çıkarım bu dizüstünün CPU'sunda:
  blok hesabı ~85–130 ms, kuyruktan okuma ~3 ms (`n_action_steps=5` ile
  çağrı ortalaması 18 ms). Pi 4 birkaç kat yavaş — 6d'de ölçülecek.

⚠️ Kinematik kameranın kenarı (6b) yapısal olarak doğru ama gerçekçi değil;
o veriden öğrenen politika boru hattını ve yöntemi kanıtlar, sahayı değil.
Gerçek beceri gerçek gösterimler + müdahale verisiyle gelir.

**Güvenlik (6d):** `policy_gate` = kendi hız tavanı (ilk sahada 0.3 m/s) ve
ivme sınırı, kendi ölü adamı (kamera/politika > 0.3 s eskiyse sıfır),
coğrafi çit (görev alanı dışı = dur), **yalnız `mission_runner` beceriyi
başlattığında açık** (varsayılan kapalı). twist_mux'ta 20: teleop (100) ve
batarya kilidi (50) onu ezer; altındaki köprü/ESP32/E-stop/tampon zinciri
aynen kalır.

### 6d-1 ✅: politika kapısı (`policy_gate`, 2026-10-24)
`robot_learning/gate.py` (ROS'suz) + `policy_gate` düğümü; twist_mux'ta yeni
girdi `policy` (`cmd_vel_policy`, öncelik **20**: Nav2'nin üstünde, batarya
kilidi 50 ve teleop 100'ün altında). Politika `policy/cmd`'ye `TwistStamped`
yayınlar — **damga, komutun hesaplandığı gözleminki**.
- **Varsayılan kapalı**, hiçbir şey yayınlamaz. `~/arm` (`PolicyArm.srv`:
  alan `map`'te JSON köşe listesi + süre bütçesi), `~/disarm`, `~/state`.
- Açıkken: tavan 0.3 m/s / 0.8 rad/s, ivme 0.5 m/s² / 2 rad/s²; komut
  > 0.3 s ya da gözlemi > 0.5 s eskiyse **sıfır** (twist_mux'ı tutar); politika
  2 s susarsa **susar** (Nav2'ye döner); alan + 1 m dışı, sıkışma
  (`jam_monitor`) ya da bütçe sonu → **kapanır** (nedeniyle).
- ⚠️ Çit `map`'te: NEO-6M'le robotun kendi tahmini metrelerce kayık olabilir —
  kaçan bir politikayı yakalar, 50 cm geniş biçeni değil.
- **Ölçülen** (`test_policy_gate_stack.py`, gerçek mux + köprü + ESP32 sim,
  hep tam gaz isteyen sahte politika): kapalıyken robot kıpırdamadı; açıkken
  `/cmd_vel` tam **0.30**; gözlem 1 s bayatlayınca durdu (kapı açık kaldı);
  çit x = 3.0'da **3.03 m**'de kapandı ve durdu. `test_gate.py`: kuralların
  her biri.

### 6d-2 ✅: `policy_node` + kapalı çevrim değerlendirme (2026-10-24)
- `robot_learning/observation.py` (ROS'suz): gözlem, eğitimdekinin birebir
  aynısı — durum sütunları checkpoint'in yanındaki `hoverbot_conversion.json`
  sırasıyla (6d-3; yoksa eski beş sütun), kamera
  `camera/jpeg` → RGB, 320×240, CHW, [0, 1]. Testler eğitim tarafının kendi
  tanımlarına bağlı (sessiz uyuşmazlık = yanlış süren politika).
- `policy_node`: checkpoint'i yükler, 10 Hz (veri setinin fps'i) `select_action`,
  `policy/cmd`'ye yayınlar (damga = kamera karesi), kapı açılınca kuyruğu
  sıfırlar. torch + LeRobot ister → `--system-site-packages` ile kurulmuş bir
  venv'de: `<venv>/bin/python -m robot_learning.policy_node -p checkpoint:=...`
- ⚠️ **Kaggle'da (GPU) eğitilmiş modelin ön-işlemesi cihazı "cuda" diye
  kaydediyor → CPU'da yüklenmiyor** (Pi'da da). Yüklerken `device_processor`
  seçilen cihaza ezdiriliyor (`policy_node`, `eval_offline.py`).
- `ros2 run robot_sim policy_eval --python <venv>/bin/python --checkpoint ...
  --seeds 20,21,22`: her saha için taze yığın, kapı sahayla açılır, politika
  `--seconds` boyunca sürer; uzmanla aynı ölçütle (`edge_metrics`) puanlanır.

**İlk kapalı çevrim sonucu** (18 uzman bölümü, Kaggle T4'te 5 000 adım ACT;
eğitimde hiç görülmemiş sahalar; 150 s):
| saha | | tur | ort. hata | p95 | kenarın ulaşılanı | dışarı |
|---|---|---|---|---|---|---|
| 20 | uzman | tam | 2.1 cm | 6.9 cm | %93 | 0 |
| 20 | politika | %38 | 5.5 cm | 18 cm | %40 | 0 |
| 21 | uzman | tam | 1.7 cm | 6.2 cm | %99 | 0 |
| 21 | politika | %69 | 7.8 cm | 22 cm | %72 | 0 |
Politika kenarı **gerçekten takip ediyor ve dışarı çıkmıyor**, ama hatası
uzmanın 2–3 katı ve **çok yavaş** (150 s'de turun %38–69'u; uzman ~80 s'de tam
tur) — sebebi incelenmedi (çekingen hız mı, köşede duraksama mı). Çevrimdışı
MAE (0.011 m/s) bunu göstermiyordu: kapalı çevrimin ayrı ölçülme sebebi bu.
Çıkarım (dizüstü CPU): ortanca 2 ms, blok hesabı 134–175 ms (her 5 adımda).
Saha 22 (sonradan): tur %22–27, ort. hata 13–32 cm, dışarı 0.

### 6d-3 ✅: yavaşlığın sebebi — "copycat" (2026-10-25)
`policy_eval` artık hızı üç yerde döküyor (`speed`): politikanın İSTEDİĞİ
(`/policy/cmd`), kapının GEÇİRDİĞİ (`/cmd_vel_policy`), robotun GERÇEK hızı;
ve ileri gitmezken `asked_stopped`: yerinde dönüş mü, boşta mı, dakikada kaç
kez dönüş yönü değiştirdi.
- **Saha 22, ilk politika:** uzman ort. 0.28 m/s (%5 duruyor); politika
  İSTEDİĞİ ort. 0.10 m/s, **zamanın %55'inde ~0** — kapı ve sürüş kısmıyor
  (geçen 0.11, gerçek 0.10). Çıkarım değil: kapının 0.3 s zaman aşımı hiç
  dolmadı.
- **Sebep — kendi hızını kopyalıyor.** Eğitim karelerinin AYNISINDA durumdaki
  `odom.v/w` sıfırlanınca politika karelerin **%51**'inde duruyor (gerçek
  durumla %4; `odom.v` 0.3 yapılınca %4). Uzmanın bir sonraki komutu ≈ robotun
  şimdiki hızı: durumda hız varsa kamera yerine o kestirme öğrenilir
  (taklit öğrenmede bilinen "copycat" sorunu). Bir kez yavaşlayan yavaş kalır.
- **Düzeltme:** durum artık yalnız ivmeölçer (`imu.ax, imu.ay` —
  `to_lerobot.DEFAULT_STATE`): duran ve sabit hızla giden robotta ikisi de ~0,
  hız bilgisi taşımaz. Tamamen durumsuz olmadı: LeRobot 0.6.1'in ACT'si
  `forward()`'da durum özelliği yokken de `observation.state` okuyor
  (Kaggle'da `KeyError`). ⚠️ Sütunlar artık TAHMİN edilmiyor: defter dönüşüm
  özetini ağırlıkların yanına koyuyor, `policy_node` oradan okuyor; sayı
  modelinkiyle uyuşmazsa ya da sütunu kuramıyorsa yüklemeyi reddediyor
  (`observation.checkpoint_state`).
- **Sonuç** (aynı 18 bölüm, aynı 5 000 adım; 150 s):

| saha | durum | tur | ort. hata | p95 | bantta | dışarı |
|---|---|---|---|---|---|---|
| 20 | hız + IMU | %38 | 5.5 cm | 18 cm | – | 0 |
| 20 | **ivme** | %45 | 3.9 cm | 8.2 cm | %96 | 0 |
| 21 | hız + IMU | %69 | 7.8 cm | 22 cm | – | 0 |
| 21 | **ivme** | %1 | – | – | – | kapı kapattı |
| 22 | hız + IMU | %22–27 | 13–32 cm | 30–80 cm | %30–41 | 0 |
| 22 | **ivme** | %40–41 | 2.9–3.0 cm | 4.8–5.4 cm | %98–99 | 0 |

  Kenarda **uzman kadar iyi** (uzman 2.5 cm), ama hâlâ yavaş. Saha 22'de
  ileri gitmediği zamanın dökümü: **%55 yerinde dönüş, %13 boşta**, dakikada
  **27 kez dönüş yönü değişiyor** — köşede sağa-sola titriyor. Uzman köşede
  (yön hatası > 30°) yerinde döner (etiketlerin %8'i); politika o sınırda
  kararsız kalıyor ve ACT'nin 5 adımlık bloğu (0.5 s × 0.8 rad/s ≈ 23°)
  düzeltmeyi taşırıyor. Saha 21'de başta yanlış yöne gitti, alanın 1 m
  dışında **kapı kapattı ve durdurdu** — kapının gerçek bir politikaya karşı
  ilk işi. İkisi de uzmanın hiç görmediği durumlar — ama önce bir sebep daha
  çıktı (6d-4).

### 6d-4 ✅: yön görevin parçası — `edge.dir` (2026-10-25)
**Titremenin sebebi: politika hangi yöne gideceğini bilmiyordu.** 18 uzman
bölümünün 9'u ccw (sınır sağda), 9'u cw; görev metni ikisinde aynı ve ACT
görev metnini zaten kullanmıyor. Kenar boyunca giderken sınırın hangi yanda
olduğu kareden okunur; ama köşede ya da başta kenara BAKARKEN tek kare "sağa
mı sola mı" diyemez → sağa-sola titreme, bir sahada yanlış yöne gitme.
- Durumda yeni sütun **`edge.dir`**: +1 ccw, −1 cw. Sensör değil, GÖREV:
  `to_lerobot.py` bölümün notundan alır (`edge_batch` yazar; `episode_export`
  artık notu `meta.json`'a taşıyor), `policy_node` `direction` parametresinden.
  Notunda yön olmayan bölüm atlanır. `policy_eval --directions ccw,cw`.
- **Sonuç** (aynı 18 bölüm, aynı 5 000 adım, 150 s):

| saha | yön | tur | istenen hız | ort. hata | bantta | yerinde dönüş | yön değişimi/dk | dışarı |
|---|---|---|---|---|---|---|---|---|
| 20 | ccw | **%182** | 0.27 m/s | 4.8 cm | %83 | %8 | 0.8 | 0 |
| 20 | cw | **%164** | 0.22 | 7.7 cm | %89 | %9 | 5.6 | 0 |
| 21 | ccw | 0 | 0.24 | – | – | %0 | 0.4 | kapı, (2.9, 0.3) |
| 21 | cw | 0 | 0.24 | – | – | %3 | 7.5 | kapı, (2.1, 4.5) |
| 22 | ccw | **%192** | 0.25 | 3.6 cm | %94 | %11 | 1.6 | 0 |
| 22 | cw | **%169** | 0.22 | 4.6 cm | %98 | %15 | 4.8 | 0 |

  Önce (6d-3): tur %40–45, istenen hız 0.07–0.09, yön değişimi 27/dk. Artık
  uzmanın hızında (0.27–0.28) ve 150 s'de ~1.7–1.9 tur. **Saha 21 ise iki
  yönde de dışarı:** sahanın doğu kenarı başlangıçtan yalnız 2.1 m önde
  (eğitim sahalarının neredeyse hepsinde ≥ 2.6 m) — politika düz gidip
  dönmüyor, kapı 1 m dışarıda durduruyor. Az sahadan (16) öğrenilmiş bir
  alışkanlık ("önce ~2.5 m düz git"); otomatik DAgger'ın ve daha çok sahanın
  işi (6d-5).

### 6d-5 ✅: otomatik DAgger — `dagger_batch` (2026-10-25)
**Politika sürer, uzman etiketler.** `ros2 run robot_sim dagger_batch --python
<venv>/bin/python --checkpoint ... --lawns 10 --seed 101`: her saha × yön için
taze yığın, `edge_expert drive:=false` (yalnız `expert/cmd_vel` etiketi —
`label()` pozun saf fonksiyonu), `policy_node` kapıdan. Bölüm biter: tur
tamam, kapı kapandı (alandan çıktı — son metreler en değerli etiketler) ya da
süre. Sonra uzman bölümleriyle birlikte `episode_export` → `to_lerobot`
(etiket: `action_expert`). Toplama sahaları 101–110: ne eğitimde (1–9) ne
değerlendirmede (20–22).

**1. tur — ve iki ders.** 18 uzman + 20 DAgger bölümüyle (5k ve 10k adım)
saha 21 **çözüldü** (iki yönde ~1.8–2 tur, eskiden dışarı), ama cw'de
politika yerinde dönüp kaldı (20 cw 0–%12, 22 cw 0; 10k adımda daha kötü —
az eğitim değil). Etiketlere bakınca:
- Başarısız DAgger bölümlerinin karelerinin **%87–91'i "yerinde dön"**,
  tek bir bölümde **124 s kesintisiz aynı etiket**: kaybolmuş politika aynı
  durumda dakikalarca kalıyor, uzman yalnız etiketlediği için düzeltmiyor.
- Robot dönerken hedef tam arkaya geçince uzmanın "yakın yön"ü 180°'de işaret
  değiştiriyor: o bölümlerde dönüşlerin %16–71'i sola — **çelişkili etiket**.

Düzeltmeler: (1) hedef 150°'den fazla arkadaysa uzman **hep sahanın içine
doğru** döner (`ExpertConfig.behind_deg`; hâlâ pozun saf fonksiyonu). (2)
**Devralma** (`--takeover-s 3`): etiket 3 s "yerinde dön" derse uzman
direksiyonu alır (`edge_expert drive:=true` → `cmd_vel_teleop`, twist_mux'ta
politikanın üstünde), 2 s ileri sürünce bırakır; bölüm kipi `intervention`.

**2. tur** (DAgger öncesi politikayla, aynı sahalar): 20/20 tur tamam, uzman
çoğunlukla bir kez 2–4 s devraldı (en zor sahada 6 kez, 24 s). Etiketlerde
"yerinde dön" %11–27, en uzun 3–6 s, yön tutarlı.

| saha | yön | DAgger öncesi (6d-4) | 1. tur, 10k | **2. tur, 10k** | 2. tur hata |
|---|---|---|---|---|---|
| 20 | ccw | %182 | %178 | **%183** | 4.0 cm |
| 20 | cw | %164 | 0 | %10 ⚠️ | 4.5 cm |
| 21 | ccw | dışarı | %191 | **%198** | 3.3 cm |
| 21 | cw | dışarı | %181 | **%179** | 7.5 cm |
| 22 | ccw | %192 | %69 | **%185** | 3.4 cm |
| 22 | cw | %169 | 0 | **%160** | 4.2 cm |

  (150 s'de tur; hiçbir koşuda dışarı yok.) 6'dan 5'i uzman hızında ~1.6–2
  tur, saha 21 çözülmüş. **20 cw hâlâ yerinde dönüyor** (%66 dönüş, 15 yön
  değişimi/dk) — her yapılandırma tek koşu, gürültü payı büyük. DAgger'ın
  olağan yolu: bir sonraki tur bu politikayla toplanır (o hâlâ takıldığı
  durumları ziyaret eder).
  3. tur toplandı (`~/.robot/sim_episodes_dagger3`, sahalar 111–120, 20/20
  tur tamam) ama eğitilip değerlendirilmedi: amaç boru hattıydı, model
  cilalamak değil (kullanıcı, 2026-10-25).

### 6e ✅: ikinci kamera deneyi — aşağı bakan geniş açı (2026-10-25)
Soru: kenar takibi için ikinci, aşağı bakan geniş açılı bir kamera almaya
değer mi? Hafif tutuldu: aday kamera Pi Kamerası'nın YERİNE (ikisi birden
değil), geri kalan her şey aynı.
- Aday (`robot.urdf.xacro` `edge_camera_link`, alınmadı): önde, 40 cm
  yükseklikte, **50° aşağı**, 110° yatay açı (sim'de iğne deliği; gerçeği
  balık gözü olur). `sim_node camera_frame:=edge_camera_optical_link
  camera_hfov:=1.92`; `edge_batch`/`policy_eval --camera edge`. Yanaşma
  hep Pi Kamerası'nı kullanır. Görüntüde Pi Kamerası'nın üst yarısı gök,
  kenar uzakta ince bir çizgi; adayda çim/toprak sınırı karenin çoğu.
- Aynı 9 saha × 2 yön uzman bölümü, aynı GPS, aynı 5 000 adım, DAgger yok —
  karşılaştırma 6d-4'ün Pi Kameralı politikası. 150 s'de tur / ort. hata:

| saha | yön | Pi Kamerası | aday kamera |
|---|---|---|---|
| 20 | ccw | %182 / 4.8 cm | %77 / 8.0 cm |
| 20 | cw | %164 / 7.7 cm | %24 / 17.3 cm |
| 21 | ccw | dışarı | **%200** / 6.1 cm |
| 21 | cw | dışarı | **%189** / 4.3 cm |
| 22 | ccw | %192 / 3.6 cm | %212 / 5.7 cm |
| 22 | cw | %169 / 4.6 cm | %197 / 6.1 cm |

- **Sonuç: belirgin bir kazanç yok.** Aday, saha 21'i DAgger'sız çözüyor
  (kenarı yakında görüyor) ve 21/22'de biraz hızlı; ama 20'de belirgin kötü
  ve ortalama hatası Pi Kamerası'ndan **iyi değil** (yakın görüşten beklenen
  buydu). Her hücre tek koşu; gürültü payı farklar kadar büyük. DAgger (6d-5)
  saha 21'i Pi Kamerası'yla zaten çözdü.
- **Karar önerisi:** kenar takibi için ikinci kamerayı **şimdi alma**. Asıl
  sınır sim değil gerçek veri (gerçek çim, ışık, balık gözü bozulması); ilk
  gerçek bölümlere bakınca yeniden değerlendir. İkisi birden (iki görüntü
  girişli politika) ayrı bir altyapı işi — kayıt/dışa aktarma/dönüştürme/
  `policy_node` tek kamera varsayıyor.

### 6f ✅: ikinci beceri — ağacın etrafından dolaşma (sim, 2026-10-25)
Kenar altyapısı olduğu gibi: uzman (`EdgeExpert`, `path=orbit_path(...)` —
gövdenin yüzeyinden `offset` dışarıda bir çember; ccw = ağaç SOLDA, kenardaki
"çim solda" ile aynı yön anlamı, `edge.dir` aynen), `lawn_gen.random_tree`
(rastgele sahaya bir gövde; uzman iki yönde de temiz dönebiliyorsa kabul),
`orbit_metrics` (gövde yüzeyine uzaklığın hatası, `too_close`, `reached`),
`edge_batch`/`policy_eval --task tree` (değerlendirme artık çarpışmaları da
sayıyor, `/collision_truth`).
- ⚠️ İlk denemede uzman gövdeye dayanıp kaldı: sim'in robotu 0.4 m'lik bir
  disk (`world.robot_radius`), ofset 0.45 yetmiyordu → **0.55**; üretici
  artık yaklaşma yolunda da gövdeye değmemeyi şart koşuyor.
- 18 uzman bölümü (sahalar 1–9 × 2 yön, hepsi başarılı, tur ~20 s), 5 000
  adım ACT, görülmemiş sahalar 20–22, 90 s:

| | uzman | politika |
|---|---|---|
| tur | %98–100 | **~0** (ccw %4–6, cw ters yöne) |
| ort. hata | 1.6–2.9 cm | 15 cm (ccw: gövdeye dayalı) / 64–82 cm (cw) |
| çarpışma | 0 | ccw **5–10**, cw 0 |
| sonu | — | ccw: **jam_monitor "stall" → kapı kapandı**; cw: uzaklaştı |

- **Sonuç: bu kamerayla öğrenilemiyor.** Robot dönerken ağaç YANDA kalıyor;
  62°'lik ön kamera onu yalnız yaklaşırken görüyor, turun kendisinde hiç.
  Politika kenar gibi "görerek düzeltemiyor": ccw'de gövdeye sürdü, cw'de
  uzaklaştı. Güvenlik zinciri çalıştı: gövdeye dayanınca sıkışma tespiti
  kapıyı kapattı (6d-1 + #34), robot durdu.
- **Ne gerekir (öneri, yapılmadı):** ağacı dönüş boyunca gören bir algı —
  yana bakan kamera ya da 360° (ikinci kamera sorusu burada yeniden açılır,
  6e'deki aşağı bakan aday da yanı görmüyor); ya da ağaç dönüşünü öğrenmeyle
  değil klasik yolla yapmak (haritadaki ağacın çevresine Nav2 yolu; NEO-6M'in
  2–5 m hatası buna yetmez, RTK ya da yakın mesafe sensörü ister).

### 6g ✅: ağaç etrafı, kitin 360° kamerasıyla (sim, 2026-10-26)
6f'nin önerisini dener: ağacı dönüş boyunca gören bir kamera. Kitin 360°
kamerası (docs/kit-soketi.md) plakada, yerden 2,0 m'de; geri kalan her şey
6f'nin aynısı (uzman, sahalar 1–9 × 2 yön = 18 bölüm, 5 000 adım ACT, durum
ivme + `edge.dir`, sahalar 20–22, 90 s).
- **Sim kamerası:** `sim_camera.render_equirect` — birleştirilmiş 320×160
  eşdikdörtgen panorama (sütun = açı, +180° solda, 0 ortada), robotun kendi
  gövdesi altta koyu. `sim_node camera_model:=equirect camera_frame:=kit_mount
  camera_z_offset:=1.82`; gövdeler `trunk_height_m:=1.5` ile gövde boyunda
  (6f'de diz boyu çizilmişti — 2 m'den bakan bir kamera onu tepeden görürdü).
  `edge_batch`/`policy_eval --camera pano`; dışa aktarma
  `episode_export --image-size 320x160`. `policy_node` görüntü boyutunu artık
  checkpoint'tan okuyor (panoramayı 4:3'e germek sessiz bir hata olurdu).
- Kaggle: `hoverbot-tree-pano` veri seti, `hoverbot-tree-pano-train` defteri
  (T4, ~11 dk); kontrol noktası `~/.robot/policies/act_tree_pano`.

| saha | ccw (ağaç solda) | cw |
|---|---|---|
| 20 | **5,9 tur**, hata 4,5 cm, çarpışma 0 | yerinde dönüyor, ağaca hiç varmadı |
| 21 | **5,1 tur**, hata 7,7 cm, çarpışma 0 | aynı |
| 22 | gövdeye dayandı (5 temas), sıkışma kapıyı kapattı | aynı |

(6f, ön kamera: ccw %4–6 tur ve 5–10 çarpışma, cw ters yöne.)
- **Sonuç: 360° kamerayla ağaç etrafı ccw'de öğrenilebiliyor** — 6f'de
  hiç olmayan şey: politika ağacı dönüş boyunca görüyor ve 90 s'de 5–6 tur
  atıyor, uzmana yakın hata ve çarpışmasız (iki sahada). Görmediği üçüncü
  sahada gövdeye dayandı; güvenlik zinciri (sıkışma → kapı) durdurdu.
- **cw öğrenilmedi:** robot başlangıçta yerinde dönüp ağaca yaklaşmıyor
  (%98 dönüş). Kenar takibindeki 6d-4'ün yön belirsizliğine benziyor
  (`edge.dir` durumda ama yaklaşma bölümü iki yönde farklı başlıyor). Sıradaki
  deneme 6d-5'in DAgger'ı (`dagger_batch --task tree --camera pano`).
- ⚠️ Kamera simde kusursuz bir birleştirme; gerçekte iki balıkgözü, dikiş,
  pozlama ve **kayıt sürerken canlı görüntünün SDK'dan alınabilmesi şartı**
  (doğrulanmadı) var.

## Sonra: politika robota nasıl takılır
Algoritma seçilince (adaylar: ACT, Diffusion Policy — ikisi de kamera +
durum → aksiyon dizisi) öğrenilen politika twist_mux'a teleop ile Nav2
arasında **ayrı bir girdi** olarak takılır — kendi hız tavanı, kendi ölü
adamı ve kendi kapısıyla; teleop onu her zaman ezer. Pi 4'te çıkarım hızı
ayrı bir ölçüm konusu (gerekirse laptopta çıkarım, Wi-Fi üzerinden — o
zaman gecikme ve kopma güvenlik zincirinin parçası olur). Bugünün işi değil.
