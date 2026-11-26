# 🛩️ SARTEK Savaşan İHA — Micro XRCE-DDS Workspace (`sartek_siha_ws`)

Teknofest Savaşan İHA yarışması için **ROS 2 Humble**, **PX4 Micro XRCE-DDS** ve **Gazebo Harmonic** tabanlı, modüler otonom uçuş ve simülasyon çalışma alanı.

> [!IMPORTANT]
> **Bu Workspace'in Temel Mimarisi:**
> - **MAVROS kullanılmaz:** PX4 ile doğrudan **Micro XRCE-DDS** (`MicroXRCEAgent` + `px4_msgs`) üzerinden haberleşir.
> - **İzole `px4_msgs`:** Harici bir `~/px4_ros_ws` underlay'i yerine PX4 sürümüyle kilitlenmiş `src/px4_msgs` doğrudan bu workspace içinde yer alır.
> - **Görev İzolasyonu (`sartek_msgs` + `sartek_px4_bridge`):** Görev algoritmaları (QR, Dogfight, Devriye, HSS/NFZ, Hakem/GCS) `px4_msgs`, NED↔ENU dönüşümü veya Offboard kurallarıyla uğraşmadan yalnızca [sartek_msgs](src/sartek_msgs) arayüzleri üzerinden geliştirilir.

---

## 📚 Doküman Haritası

| Doküman | Ne Zaman Kullanılır? |
|---|---|
| **[README.md](README.md)** *(bu dosya)* | İlk kurulum, çakışma kontrolleri, GitHub klon/push ve genel mimari |
| **[ws_rehber.md](ws_rehber.md)** | Günlük kullanım el kitabı: klasör yapısı, tekli simülasyon + köprü başlatma ve terminal test komutları |
| **[MULTI_SIM_REHBER.md](MULTI_SIM_REHBER.md)** | 6 uçaklı çoklu simülasyon (`multi_sim.launch.py`) adım adım çalıştırma, QGC ve doğrulama rehberi |
| **[INSTALLATION.md](INSTALLATION.md)** | Hiç ROS 2 / Agent kurulmamış sıfır Ubuntu 22.04 görev bilgisayarı kurulumu |
| **[PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md)** | Gazebo Harmonic, `PX4-Autopilot` SITL ve QGroundControl kurulumu |

---

## 📦 Workspace Paketleri ve Ekip İçi Standartlar

| Paket | Tür | Görevi |
|---|---|---|
| **[src/px4_msgs](src/px4_msgs)** | Arayüz | `PX4-Autopilot` (`main` / `v1.18+`, `67f5c57e9a`) ile birebir uyumlu uORB mesajları (`de6b70e`) |
| **[src/sartek_msgs](src/sartek_msgs)** | Arayüz | Tüm görev, algı, güvenlik ve GCS düğümlerinin ortak mesaj/aksiyon sözleşmesi (9 `.msg` + 1 `.action`) |
| **[src/sartek_px4_bridge](src/sartek_px4_bridge)** | Omurga | `px4_telemetry_bridge` (`/fmu/out/*` → `TelemetryStatus` + `SystemHealth`) ve `px4_control_bridge` (`MissionCommand` / `SafetyOverride` → `/fmu/in/*`) |
| **[src/sartek_sim](src/sartek_sim)** | Simülasyon | Gazebo Harmonic (`yunuseli.sdf`, `yarisma.sdf`), kameralı uçak (`advanced_plane_cam`), tekli (`sim.launch.py`) ve 6 uçaklı otonom rakip simülasyonu (`multi_sim.launch.py` + `virtual_uav_runner`) |

### Sabitlenen Geliştirici Standartları
- **Görev Koordinat Sistemi:** Tüm görev düğümlerinde ve `sartek_msgs` içinde **Local ENU** (`X=Doğu`, `Y=Kuzey`, `Z=Yukarı +`) kullanılır; NED↔ENU çevrimi yalnızca `sartek_px4_bridge` içinde yapılır.
- **Açılar:** `roll_deg` (sağ kanat aşağı `+`), `pitch_deg` (burun yukarı `+`, dalış `-`), `yaw_deg` (`0..360°`, Kuzey'den saat yönünde pusula yönelimi).
- **Kamera Hata & BBox Birimi:** `QrMissionStatus` ve `DogfightStatus` içinde `error_x / error_y` **`[-1.0, +1.0]` normalize**, `bbox_cx / cy / w / h` **`[0.0, 1.0]` normalize** kullanılır.
- **Yunuseli Saha Referansı ([worlds/yunuseli.sdf](src/sartek_sim/worlds/yunuseli.sdf)):**
  - Home GPS: `40.233243, 29.005583, 101.0 m`
  - QR Hedef Konumu (Gazebo ENU): `(86.463, -276.128, 0.1)` (`40.2307126, 29.0067600`)

---

## 1. İndirilmesi ve Kurulu Olması Gereken Sistem Bileşenleri

Bu workspace'i çekip çalıştıracak bilgisayarda (Ubuntu 22.04 LTS x86_64 veya Jetson/ARM64) aşağıdaki sistem geneli bileşenler kurulu olmalıdır:

| Bileşen | Beklenen Durum / Sürüm | Kurulum / Not |
|---|---|---|
| **Ubuntu** | `22.04 LTS` | Temel işletim sistemi |
| **ROS 2** | `Humble` (`/opt/ros/humble`) | `ros-humble-desktop`, `python3-colcon-common-extensions`, `python3-rosdep`, `python3-pip` |
| **RMW (DDS)** | `rmw_cyclonedds_cpp` | `sudo apt install -y ros-humble-rmw-cyclonedds-cpp` |
| **Micro-XRCE-DDS Agent** | `/usr/local/bin/MicroXRCEAgent` | `eProsima/Micro-XRCE-DDS-Agent` kaynaktan derlenmiş olmalı |
| **Gazebo** | **Harmonic** (`gz-harmonic`) | `gz-harmonic` + `ros-humble-ros-gzharmonic` (**Fortress `ros-humble-ros-gz` kurulmamalı!**) |
| **PX4-Autopilot** | `main` (`v1.18.0-beta1` / `67f5c57e9a`+) | `~/PX4-Autopilot` (`make px4_sitl gz_advanced_plane` ile en az 1 kez derlenmiş olmalı) |
| **Sistem Kütüphaneleri** | `libzbar0`, `python3-pyqt5` | QR (`pyzbar`) ve arayüz bağımlılıkları |
| **Python Sabitlemeleri** | `numpy<2`, `opencv-python<4.9.0` | ROS 2 Humble `cv_bridge` ve `colcon` uyumu için (Bölüm 2.5'e bakın) |
| **QGroundControl** *(Opsiyonel)* | `~/.local/bin/qgroundcontrol` | Harici GCS izleme/kalkış için ([PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md) Adım 4) |

> Sıfırdan kurulum gerekiyorsa [INSTALLATION.md](INSTALLATION.md) ve [PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md) dosyalarındaki adım adım yönergeleri takip edebilirsiniz.

---

## 2. Kritik Çakışma Kontrolleri (Kurulumdan ve Derlemeden Önce!)

Yeni workspace'in eski projelerle (`px4_ros_ws`, `drone_ws`, `sartek_ws`, `multi_ws`) veya Python/Gazebo paketleriyle çakışmaması için aşağıdaki **5 kontrolü mutlaka yapın**:

### 2.1. `~/.bashrc` Kontrolü (En Kritik Kontrol)
Her yeni terminal açıldığında `~/.bashrc` otomatik çalışır. Eğer `~/.bashrc` içinde eski bir workspace (özellikle `~/px4_ros_ws`) `source` ediliyorsa, `sartek_siha_ws` derlenirken eski workspace'in üzerine bağlanır ve yanlış `px4_msgs` mesajları yüklenir.

Kontrol edin:
```bash
grep -n "setup.bash" ~/.bashrc
env | grep -E "AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|COLCON_PREFIX_PATH|PYTHONPATH"
```

**Olması gereken durum:**
- `~/.bashrc` içinde **yalnızca** sistem ROS 2 kurulumu bulunmalı (veya tek aktif projeniz buysa altına `sartek_siha_ws` eklenebilir):
  ```bash
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
- `~/.bashrc` içinde **`px4_ros_ws`, `drone_ws`, `sartek_ws`, `multi_ws` satırları BULUNMAMALIDIR.**
- `env` çıktısında `AMENT_PREFIX_PATH` yalnızca `/opt/ros/humble` (ve source ettikten sonra `~/sartek_siha_ws/install/...`) göstermelidir.

> **Çoklu Proje Kuralı:** Bilgisayarınızda birden fazla ROS 2 workspace'i duruyorsa `~/.bashrc` içine hiçbir workspace satırı yazmayın; hangi projede çalışıyorsanız sadece o terminalde `source ~/sartek_siha_ws/install/setup.bash` çalıştırın.

---

### 2.2. `RMW_IMPLEMENTATION` Eşleşmesi
Tüm terminaller (Agent, Gazebo köprüsü, düğümler) aynı DDS katmanını kullanmazsa birbirini göremez:
```bash
echo "$RMW_IMPLEMENTATION"
# Beklenen çıktı: rmw_cyclonedds_cpp
```

---

### 2.3. `PX4-Autopilot` ↔ `px4_msgs` Sürüm Eşleşmesi
Micro XRCE-DDS'te PX4 uORB tanımı ile `src/px4_msgs` **birebir eşleşmek zorundadır**:
- **`~/PX4-Autopilot`:** `main` (`v1.18.0-beta1`, commit `67f5c57e9a` / `a6814cc7b6`)
- **`src/px4_msgs`:** `main`, commit `de6b70e54b57d8dd2198e3c8789907f0799486f8`

Kontrol edin:
```bash
git -C ~/PX4-Autopilot describe --tags --always
git -C ~/PX4-Autopilot log -n 3 --oneline
```

---

### 2.4. Gazebo Harmonic vs. Fortress Çakışması
ROS 2 Humble varsayılan deposundaki `ros-humble-ros-gz` **Gazebo Fortress** kurar ve Harmonic ile çakışır:
```bash
gz sim --version
dpkg -l | grep ros-humble-ros-gz
```
**Beklenen:** `gz-harmonic` ve `ros-humble-ros-gzharmonic` kurulu olmalı; `ros-humble-ros-gz` (Fortress) kurulu **olmamalı**.

---

### 2.5. Python / `cv_bridge` / `colcon` Sürüm Kontrolü
ROS 2 Humble `cv_bridge` paketi **NumPy 1.x** ile derlenmiştir (`numpy>=2.0`, `opencv-python>=4.9` veya `setuptools>=80` `cv_bridge` ve `colcon build` işlemlerini bozar). Özellikle `ultralytics` veya `PX4-Autopilot/Tools/setup/ubuntu.sh` kurulumundan sonra mutlaka doğrulayın:
```bash
source /opt/ros/humble/setup.bash
python3 -c "from cv_bridge import CvBridge; import numpy as np, cv2; print('OK:', np.__version__, cv2.__version__)"
```
Sorun varsa sabitleyin:
```bash
pip3 install --user "numpy>=1.23.0,<2.0.0" "opencv-python>=4.8.0,<4.9.0" "packaging>=24.0" "setuptools>=77.0.3,<80.0.0"
```

---

## 3. GitHub'dan Çeken Kişi İçin İlk Kurulum Adımları

### Adım 1: Depoyu Klonlayın ve `px4_msgs` İçeriğini Doğrulayın
```bash
cd ~
git clone --recursive <GITHUB_REPO_URL> sartek_siha_ws
cd ~/sartek_siha_ws
```

> [!NOTE]
> Eğer `src/px4_msgs` klasörü boş geldiyse (ör. `--recursive` unutulduysa):
> ```bash
> git submodule update --init --recursive
> # Veya submodule yoksa manuel olarak uyumlu commit'i çekin:
> # rm -rf ~/sartek_siha_ws/src/px4_msgs
> # git clone https://github.com/PX4/px4_msgs.git ~/sartek_siha_ws/src/px4_msgs
> # git -C ~/sartek_siha_ws/src/px4_msgs checkout de6b70e54b57d8dd2198e3c8789907f0799486f8
> ```

### Adım 2: Temiz ROS 2 Ortamında Bağımlılıkları Yükleyin ve Derleyin
```bash
source /opt/ros/humble/setup.bash
cd ~/sartek_siha_ws

# (İlk kez rosdep kullanıyorsanız: sudo rosdep init && rosdep update)
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source ~/sartek_siha_ws/install/setup.bash
```

### Adım 3: Derleme, Yol ve Birim Test Doğrulaması
```bash
ros2 pkg prefix px4_msgs sartek_msgs sartek_px4_bridge sartek_sim
# Tüm paket yolları ~/sartek_siha_ws/install/... altında görünmelidir

python3 -c "from px4_msgs.msg import VehicleStatus; from sartek_msgs.msg import TelemetryStatus, MissionCommand; print('ALL_IMPORTS_OK')"

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/sartek_px4_bridge/test/test_conversions.py -v
```

---

## 4. Simülasyon ve Uçuş Omurgasını Çalıştırma

### A. Tek Komutla Simülasyon + Köprü (Önerilen Akış)

1. **Terminal 1 — Simülasyon (`sartek_sim`):**  
   Gazebo Harmonic (`yunuseli.sdf`), kameralı uçak (`advanced_plane_cam`), PX4 SITL (`PX4_GZ_STANDALONE=1`), kamera köprüsü ve `MicroXRCEAgent`'ı tek seferde başlatır:
   ```bash
   pkill -9 -x px4 2>/dev/null || true
   pkill -9 -x MicroXRCEAgent 2>/dev/null || true
   pkill -9 -f "gz sim" 2>/dev/null || true

   source /opt/ros/humble/setup.bash
   source ~/sartek_siha_ws/install/setup.bash
   ros2 launch sartek_sim sim.launch.py
   # GUI olmadan (headless) çalıştırmak için: headless:=true
   ```

2. **Terminal 2 — PX4 Sınır Köprüsü (`sartek_px4_bridge`):**  
   PX4 `/fmu/*` topic'lerini ENU tabanlı `/gorev/telemetry_status`, `/gorev/system_health` ve `/gorev/mission_command` arayüzlerine çevirir:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/sartek_siha_ws/install/setup.bash
   ros2 launch sartek_px4_bridge px4_bridge.launch.py
   ```

3. **Terminal 3 — Hızlı Doğrulama:**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/sartek_siha_ws/install/setup.bash

   ros2 topic echo /gorev/telemetry_status --once
   ros2 topic echo /gorev/system_health --once
   ros2 topic echo /camera/image_raw --no-arr --once
   ```
   *(Terminalden `GOTO`, `ATTITUDE`, `ARM`, `TAKEOFF`, `OFFBOARD`, `SafetyOverride` test komutları ve tüm launch parametreleri için [ws_rehber.md](ws_rehber.md) dosyasına bakın.)*

### B. Yalın PX4 Testi (3 Terminalde Manuel)
`sartek_sim` olmadan yalnızca PX4 + Micro XRCE-DDS bağlantısını test etmek isterseniz:
- **Terminal 1:** `MicroXRCEAgent udp4 -p 8888` *(Gerçek uçuş seri port: `MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600`)*
- **Terminal 2:** `cd ~/PX4-Autopilot && make px4_sitl gz_advanced_plane`
- **Terminal 3:** `source /opt/ros/humble/setup.bash && source ~/sartek_siha_ws/install/setup.bash && ros2 topic list | grep /fmu/`

### ⚠️ Altyapı Geliştiricileri İçin `px4_msgs` Notları (`sartek_px4_bridge` İçi)
Görev algoritması yazanlar doğrudan `sartek_msgs` kullanır; ancak `sartek_px4_bridge` içinde geliştirme yaparken dikkat edilmesi gereken PX4 v1.18 kuralları şunlardır:
- **Versiyonlu Topic İsimleri (`MESSAGE_VERSION != 0`):** `/fmu/out/vehicle_status_v5`, `vehicle_local_position_v1`, `battery_status_v1`, `airspeed_validated_v1`, `home_position_v2`, `vehicle_command_ack_v2`, `/fmu/in/vehicle_command_v1`, `vehicle_attitude_setpoint_v1`.
- **Koordinat:** PX4 **NED** (`X=Kuzey, Y=Doğu, Z=Aşağı +`) kullanır.
- **QoS & Timestamp:** `/fmu/*` topic'lerinde `BEST_EFFORT` QoS ve mikrosaniye (`int(time.time_ns() // 1000)`) `timestamp` kullanılmalıdır.

---

## 5. GitHub'a İlk Push ve Ekip İçi Çalışma Kuralları

1. **`build/`, `install/`, `log/` ve `.pytest_cache/` klasörlerini asla commit'lemeyin** ([.gitignore](.gitignore) içinde tanımlıdır).
2. **İlk defa GitHub'a yüklerken (`src/px4_msgs` submodule olarak):**
   ```bash
   cd ~/sartek_siha_ws
   git init -b main
   git submodule add https://github.com/PX4/px4_msgs.git src/px4_msgs
   git add .
   git status   # build/, install/, log/ görünmediğini doğrulayın
   git commit -m "Initial workspace setup: px4_msgs, sartek_sim, sartek_msgs, sartek_px4_bridge"
   git remote add origin <GITHUB_REPO_URL>
   git push -u origin main
   ```
3. **Aynı ağda birden fazla kişi test yapıyorsa** her ekip/uçak için ayrı `ROS_DOMAIN_ID` (`export ROS_DOMAIN_ID=<id>`) kullanın; tüm terminallerin aynı `ROS_DOMAIN_ID` ve `RMW_IMPLEMENTATION` değerine sahip olduğundan emin olun.
