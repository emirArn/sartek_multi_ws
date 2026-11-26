# ✈️ Çoklu İHA Simülasyonu Çalıştırma Rehberi (`multi_sim`)

Bu rehber, Teknofest Savaşan İHA dogfight/takip senaryoları için geliştirilen **6 uçaklı Gazebo Harmonic + PX4 SITL + Micro XRCE-DDS** çoklu simülasyon ortamının adım adım nasıl başlatılacağını, kontrol edileceğini ve doğrulanacağını anlatır.

---

## 1. Sistem Mimarisi (6 Uçak)

`ros2 launch sartek_sim multi_sim.launch.py` tek komutla aşağıdaki bileşenleri sırayla ayağa kaldırır:

| Rol | Gazebo Modeli | PX4 Instance | DDS Namespace | Başlangıç Konumu `(x, y)` | Açıklama |
|---|---|---:|---|---|---|
| **Kendi Uçağımız** | `advanced_plane_cam` | `0` | `/` *(kök)* | `(0, 0)` | Ön kamera (`/camera/image_raw`) + `sartek_px4_bridge` ile kontrol edilir |
| **Rakip İHA 1** | `gz_advanced_plane_1` | `1` | `/uav1` | `(0, 12)` | `virtual_uav_runner` ile otonom kalkış + rastgele waypoint uçuşu |
| **Rakip İHA 2** | `gz_advanced_plane_2` | `2` | `/uav2` | `(0, -12)` | `virtual_uav_runner` ile otonom kalkış + rastgele waypoint uçuşu |
| **Rakip İHA 3** | `gz_advanced_plane_3` | `3` | `/uav3` | `(-18, 12)` | `virtual_uav_runner` ile otonom kalkış + rastgele waypoint uçuşu |
| **Rakip İHA 4** | `gz_advanced_plane_4` | `4` | `/uav4` | `(-18, -12)` | `virtual_uav_runner` ile otonom kalkış + rastgele waypoint uçuşu |
| **Rakip İHA 5** | `gz_advanced_plane_5` | `5` | `/uav5` | `(-36, 0)` | `virtual_uav_runner` ile otonom kalkış + rastgele waypoint uçuşu |

- **Varsayılan Dünya:** [`src/sartek_sim/worlds/yarisma.sdf`](src/sartek_sim/worlds/yarisma.sdf)
- **Haberleşme:** Tüm 6 uçak tek bir `MicroXRCEAgent` (`udp4 -p 8888`) üzerinden ROS 2 ile konuşur.
- **Sanal Rakip Kontrolü:** [`virtual_uav_runner.py`](src/sartek_sim/sartek_sim/virtual_uav_runner.py) her rakip uçağı sırayla GPS bekler → `TAKEOFF` gönderir → `ARM` eder → belirlenen irtifaya tırmanır → `OFFBOARD` moduna geçip rastgele waypoint'lerde uçurur.

---

## 2. Adım Adım Çalıştırma

### Adım 1: Önce QGroundControl'ü Açın
6 uçağın tamamını haritada görebilmek için **simülasyondan önce** QGroundControl'ün açık olması gerekir. `multi_sim.launch.py` varsayılan olarak (`require_qgc:=true`) QGC'nin açık olup olmadığını kontrol eder; açık değilse uyarı verip başlamaz.

```bash
~/.local/bin/qgroundcontrol &
```

> [!TIP]
> QGroundControl kurulu değilse veya haritasız sadece terminal/Gazebo ile test yapacaksanız Adım 2'de `require_qgc:=false` parametresini kullanabilirsiniz. Bu durumda `virtual_uav_runner` sanal uçaklara otomatik olarak GCS heartbeat gönderir.

---

### Adım 2: Çoklu Simülasyonu Başlatın (Terminal 1)
Workspace'i derlemediyseniz önce bir kez derleyin, ardından ortamı yükleyip `multi_sim.launch.py` dosyasını çalıştırın:

```bash
cd ~/sartek_siha_ws
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# İlk kez veya kod değişikliği sonrası derleme:
colcon build --packages-select sartek_sim --symlink-install

# Workspace'i yükle ve 6 uçaklı simülasyonu başlat:
source install/setup.bash
ros2 launch sartek_sim multi_sim.launch.py
```

**Başlatma Sırası (Otomatik):**
- `t = 0s` → Gazebo Harmonic (`yarisma.sdf`) açılır.
- `t = 2s` → `MicroXRCEAgent udp4 -p 8888` başlar.
- `t = 5s` → Kendi uçağımız (`advanced_plane_cam`), PX4 instance `0` ve kamera köprüsü (`/camera/image_raw`) başlar.
- `t = 8s..20s` → 5 sanal rakip uçak 3'er saniye arayla spawn edilir.
- `t = 23s..35s` → `virtual_uav_runner_uav1..5` düğümleri devreye girer ve 5 rakip uçak otomatik olarak kalkış yapıp havada devriye gezmeye başlar.

---

### Adım 3: Kendi Uçağımızın Köprüsünü Başlatın (Terminal 2)
Kendi uçağımız (`advanced_plane_cam`) kök namespace (`/`) üzerinde çalıştığı için mevcut `sartek_px4_bridge` paketini hiçbir ekstra ayar yapmadan doğrudan başlatabilirsiniz:

```bash
cd ~/sartek_siha_ws
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash

ros2 launch sartek_px4_bridge px4_bridge.launch.py
```

Artık kendi uçağınızı:
- **QGroundControl** üzerinden **Vehicle 1**'i seçerek manuel / Takeoff / Mission modunda uçurabilir, veya
- **`/sartek/mission_command`** üzerinden otonom görev düğümleriyle kontrol edebilirsiniz (detaylar için [ws_rehber.md](ws_rehber.md)).

---

## 3. Launch Parametreleri

`multi_sim.launch.py` aşağıdaki opsiyonel parametreleri destekler:

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `require_qgc` | `true` | `true` ise QGC açık değilken launch durur. `false` ile QGC olmadan çalıştırılır. |
| `auto_fly` | `true` | `true` ise 5 rakip uçak otomatik kalkıp rastgele uçar. `false` ise yerde bekler. |
| `headless` | `false` | `true` ise Gazebo GUI açılmaz (`gz sim -r -s`), CPU/GPU yükü azalır. |
| `world_name` | `yarisma` | Yüklenecek SDF dünya adı (ör. `yunuseli` veya `yarisma`). |
| `agent_port` | `8888` | `MicroXRCEAgent` UDP portu. |
| `px4_dir` | `~/PX4-Autopilot` | Derlenmiş `PX4-Autopilot` dizini. |

**Örnek kullanımlar:**

```bash
# QGroundControl olmadan çalıştırma:
ros2 launch sartek_sim multi_sim.launch.py require_qgc:=false

# Rakip uçaklar yerde beklesin (manuel test için):
ros2 launch sartek_sim multi_sim.launch.py auto_fly:=false

# Headless (ekransız) + Yunuseli haritasında çalıştırma:
ros2 launch sartek_sim multi_sim.launch.py headless:=true world_name:=yunuseli require_qgc:=false
```

---

## 4. Kontrol ve Doğrulama Komutları (Terminal 3)

Simülasyon ayaktayken yeni bir terminalde (`source install/setup.bash` ve `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` sonrası) aşağıdaki komutlarla her şeyin sağlıklı çalıştığını doğrulayabilirsiniz:

### Tüm Uçakların Topic'lerini Kontrol Etme
```bash
ros2 topic list | grep -E "^(/fmu|/uav[1-5]/fmu|/camera|/sartek)"
```
Beklenen:
- Kendi uçağımız: `/fmu/out/...`, `/fmu/in/...`, `/camera/image_raw`
- 5 rakip uçak: `/uav1/fmu/out/...` → `/uav5/fmu/out/...`
- Köprü açıksa: `/sartek/telemetry_status`, `/sartek/system_health`

### Kendi Uçağımızın Telemetrisini İzleme
```bash
ros2 topic echo /sartek/telemetry_status
```

### Kamera Görüntüsünü Kontrol Etme
```bash
# Yayın hızını kontrol et:
ros2 topic hz /camera/image_raw

# Canlı görüntüyü aç:
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### Rakip Uçaklardan Birinin Konumunu İzleme
```bash
ros2 topic echo /uav1/fmu/out/vehicle_local_position_v1 --once
```

---

## 5. Temiz Kapatma ve Sorun Giderme

Simülasyonu kapatmak için Terminal 1'de `Ctrl+C` yapın. Arka planda asılı kalan PX4, Gazebo veya Agent süreçleri olursa hepsini tek seferde temizleyebilirsiniz:

```bash
pkill -f "px4" || true
pkill -f "gz sim" || true
pkill -f "MicroXRCEAgent" || true
```

### Sık Karşılaşılan Durumlar
- **"QGroundControl açık değil!" hatası:** Önce `~/.local/bin/qgroundcontrol &` ile QGC'yi açın ya da `require_qgc:=false` parametresi ekleyin.
- **Topic'ler görünmüyor:** Tüm terminallerde `echo $RMW_IMPLEMENTATION` çıktısının `rmw_cyclonedds_cpp` olduğunu doğrulayın.
- **Bilgisayar yavaşlıyor / FPS düşük:** 6 PX4 instance + Gazebo GUI yüksek işlemci/ekran kartı kullanır. Görsel gerekmiyorsa `headless:=true` ile başlatabilirsiniz.
