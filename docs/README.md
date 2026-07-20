# CAM message publikálása L2-ben 


## Előkészítés 

-    hasznalt docker image: `ghcr.io/autowarefoundation/autoware:universe-devel-cuda-humble`
-    docker network készítése (a dokcer eth0jara csatlakoznak a message ek amiket wireshark al el ehet kapni a host rol ) <br> 
    `docker network create --driver=bridge --subnet=10.0.0.0/24 v2x_net -o com.docker.network.bridge.name="v2x_net"`

## Contianer futtatása: 

A scripts folderbe talalhato a `run_cont.sh` shell file aminek sehitsegevel el lehet inditani a container t 
### FONTOS a shell file ba at irni az elereseket a vanetza source is kell az asn messsage dekodolasok miatt (nem tudom h nincs benne a python package be XD)

## Container-ben 

### Autoware launch a szokásos

1. A dokcerbe a szokasos helyen van az autoware install:
- `source /opt/autoware/setup.bash`

2. Ezután jöhet a planning-simulator launch:
- `ros2 launch autoware_launch  planning_simulator.launch.xml map_path:=$HOME/maps/highway`

NOTES: itt ha esetleg launch error az memory allocation miatt emelni kell a host on a memry size t (claude segít)


## CAM telepítése `colcon` -al


1. uj shell egy adott continer-be: <br>
`docker exec -it autoware_x bash`

2. Az ASN py modul feltelepítése: <br>
`pip install asn1tools`!!!!!!!!!!!!!!!!(Ezt aakkor be kéne tenni a saját docker image be h tegye fel default)!!!!!!!!!!!!!!!!!!!!!!

3. Autoware se source olasa (az rclpy es a többi message type miatt): <br>
`source /opt/autoware/setup.bash`
4. autowarev2x package build elése és telepitése <br>
`cd dev` <br>
`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
## CAM modul indítása: <br>

`ros2 launch autoware_v2x cam.launch.py`


NOTES: ezen commit alatt van a merged node ami egsyerre kuldi es fogajda a CAM message eket 


## Vizualizácó RVIZ ben: 

Ahogy lent is látható modon hozzá kell adni az rvizhez az adott topic ot csak nekünk most `cam` lesz nem `cpm` 

![alt text](image.png)



## Ellenőrzés wireshark segítségével: 

Futtatás : `sudo wireshark`

Ezután a programon belül a host gép `v2x_net` interface ét kell keresni azon beül lesz található az adatforgalom

---

# `tools/` mappa – fájlok leírása

A `tools/` mappában találhatók azok a scriptek és konfigurációs fájlok, amelyek a Vanetza-alapú V2X rendszer (CAM üzenetküldés/fogadás) indításához szükségesek a Docker konténeren belül.

---

## `launch.sh` – Fő indítószkript

Ez a legfontosabb belépési pont. Egy paranccsal elindítja az összes szükséges komponenst.

**Mit csinál lépésről lépésre:**

1. **`ROS_DOMAIN_ID` beolvasása** – Ha nincs beállítva környezeti változóként, alapértelmezetten `1`-et használ. Ez határozza meg az állomás azonosítóját is.
2. **`paho-mqtt` telepítése** – pip-en keresztül feltelepíti az MQTT Python kliens könyvtárat (ha még nincs meg).
3. **Hostnév felvétele** – Hozzáadja a saját hostnevét a `/etc/hosts` fájlhoz, hogy a hálózati kommunikáció ne akadjon meg.
4. **Mosquitto MQTT broker ellenőrzése és indítása** – Ha a `mosquitto` process még nem fut, elindítja háttérfolyamatként (`mosquitto -d`). Ez az MQTT közvetítő, amelyen keresztül a CAM üzenetek haladnak.
5. **Környezeti változók beállítása** – Beállítja a Vanetza működéséhez szükséges paramétereket:
   - `VANETZA_INTERFACE=eth0` – hálózati interfész
   - `VANETZA_STATION_ID` – az állomás azonosítója (= `ROS_DOMAIN_ID`)
   - `VANETZA_STATION_TYPE=5` – állomástípus (5 = személyautó)
   - `VANETZA_MAC_ADDRESS` – a station ID alapján generált MAC cím (`6e:06:e0:03:00:XX`)
   - `VANETZA_IGNORE_OWN_MESSAGES=true` – saját üzenetek figyelmen kívül hagyása
   - `VANETZA_USE_HARDCODED_GPS=true` – hardcoded GPS pozíció használata (gpsd nélkül)
   - `VANETZA_LATITUDE=47.5316`, `VANETZA_LONGITUDE=21.6273` – Debrecen közelének koordinátái (térkép origója)
   - MQTT engedélyezve, Zenoh kikapcsolva, periodicitás=0 (csak Python-ból érkező CAM megy ki)
6. **Autoware környezet betöltése** – `source /opt/autoware/setup.bash` – szükséges az ROS 2 node-okhoz.
7. **Python scriptek háttérben indítása:**
   - `cam_receive.py` – fogadja a más állomásoktól érkező CAM üzeneteket MQTT-n, és ROS 2 `TrackedObjects` üzenetként publikálja
   - `cam_send.py` – az Autoware lokalizációját olvassa és CAM üzenetként küldi ki MQTT-n
8. **`socktap` indítása előtérben** – Ez a Vanetza fő folyamata, amely a `config.ini` alapján kezeli a V2X kommunikációt az `eth0` interfészen.

**Használat (konténeren belül):**
```bash
bash /home/aw/tools/launch.sh
```

---

## `launch_autoware.sh` – Autoware Planning Simulator indítása

Egy egyszerű segédszkript, amely betölti az Autoware környezetet és háttérben elindítja a planning simulatort.

**Mit csinál:**
1. `source /opt/autoware/setup.bash` – Autoware ROS 2 workspace betöltése
2. `ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/maps/highway/` – Planning Simulator indítása háttérben a debreceni autópálya térképpel

**Mikor kell használni:** Ha az Autoware szimulátort külön akarod elindítani, nem a `launch.sh`-val együtt.

---

## `cam_send.py` – CAM üzenet küldő ROS 2 node

Ez a Python script egy ROS 2 node (`autoware_cam_mqtt`), amely az Autoware lokalizációs adatait CAM (Cooperative Awareness Message) üzenetekké alakítja és MQTT-n keresztül elküldi a Vanetza felé.

**Működése:**
- Feliratkozik az `/localization/kinematic_state` ROS 2 topicra (Odometry), amelyen az Autoware a jármű pozícióját és sebességét publikálja.
- Az ENU (East-North-Up) koordinátákat GPS koordinátákká (WGS84) konvertálja a `pyproj` könyvtár segítségével. A térkép origója: **Debrecen (47.5316°N, 21.6273°E)**.
- A CAM JSON sablont (`/vanetza/examples/in_cam.json`) tölti be, és feltölti az aktuális adatokkal:
  - pozíció (szélességi/hosszúsági fok)
  - menetirány (heading, fokokban)
  - sebesség (m/s)
  - szögsebség (yaw rate, fok/s)
  - jármű méretei
- Az elkészített CAM JSON-t **10 Hz**-en publikálja az MQTT broker `vanetza/in/cam` topicjára.
- A Vanetza (`socktap`) ezt az MQTT üzenetet átveszi és V2X rádión (vagy `eth0`-n) kisugározza.

**Fontos részletek:**
- A `lowFrequencyContainer`-t (amely `ExteriorLights`-ot tartalmaz) eltávolítja küldés előtt, mert ASN.1 méret hibát okoz.
- Ha nincs Odometry adat, visszaszámítja a sebességet az előző pozícióból.

---

## `cam_receive.py` – CAM üzenet fogadó ROS 2 node

Ez a Python script egy ROS 2 node (`cam_mqtt_to_tracked`), amely a más járművektől érkező CAM üzeneteket fogadja MQTT-n és Autoware-kompatibilis `TrackedObjects` üzenetekké alakítja, hogy az Autoware perception stackje megjelenítse és felhasználja azokat.

**Működése:**
- Feliratkozik az MQTT broker `vanetza/out/cam` topicjára. A Vanetza ide teszi ki a hálózatról érkező, dekódolt CAM üzeneteket.
- Minden beérkező CAM üzenetből kinyeri:
  - állomás azonosítóját (`stationID`) → ez lesz az objektum UUID-ja
  - GPS pozícióját → ENU koordinátává konvertálja (`pyproj` segítségével)
  - menetirányát → quaternionná alakítja
  - sebességét és szögsebségét
  - jármű hosszát és szélességét
- **50 Hz**-en publikál a `/perception/object_recognition/tracking/objects` ROS 2 topicra `TrackedObjects` formátumban.
- Ha egy objektumtól **3 másodpercig** nem érkezik friss CAM üzenet, eltávolítja a listából (timeout).
- Okos visszaskálázást végez: a CAM üzenetekben egyes értékek egész számban jönnek (pl. lat×10⁷), és automatikusan érzékeli, hogy kell-e osztani.

**Eredmény:** Az RViz-ben és az Autoware planning moduljában megjelennek a V2X-en keresztül érkező járművek, mint `TrackedObjects`.

---

## `config.ini` – Vanetza (`socktap`) konfigurációs fájl

A `socktap` folyamat konfigurációs fájlja, amely meghatározza a Vanetza V2X stack teljes működését.

**Főbb szekciók:**

### `[general]`
| Paraméter | Érték | Leírás |
|---|---|---|
| `interface` | `eth0` | V2X kommunikáció hálózati interfésze |
| `local_mqtt_broker` | `127.0.0.1` | Helyi Mosquitto broker címe |
| `local_mqtt_port` | `1883` | Broker portja |
| `ignore_own_messages` | `false` | Saját üzenetek is dekódolásra kerülnek (a `launch.sh`-ban env változóval felülírható) |
| `use_hardcoded_gps` | `true` | Nem vár gpsd-re, hardcoded koordinátákat használ |
| `rssi_enabled` | `false` | Nincs valódi 802.11p rádió, kikapcsolva |
| `enable_json_prints` | `true` | Kimeneti JSON logolás engedélyezve |
| `debug_enabled` | `true` | Részletes debug kimenet |

### `[station]`
| Paraméter | Érték | Leírás |
|---|---|---|
| `id` | `1` | Állomásazonosító (felülírható env-vel) |
| `type` | `5` | Állomástípus (5 = személyautó, ITS-AID szerint) |
| `mac_address` | `6e:06:e0:03:00:01` | Szimulált MAC cím |
| `use_hardcoded_gps` | `true` | Hardcoded GPS |
| `latitude` / `longitude` | `47.5316` / `21.6273` | Debreceni koordináták |

### `[cam]` – CAM üzenetek (aktív)
- `mqtt_enabled=true` – MQTT-n fogad/küld (`vanetza/in/cam` → rádió, rádió → `vanetza/out/cam`)
- `periodicity=0` – Vanetza saját maga nem generál CAM-et; csak a Python node által küldött üzeneteket továbbítja
- `zenoh_enabled=false`, `dds_enabled=false` – csak MQTT transport aktív

### `[denm]` – DENM üzenetek (aktív, MQTT)
Veszélyhelyzet-figyelmeztetések. MQTT-n engedélyezett, de jelenleg nem használt aktívan.

### Többi szekció (kikapcsolva)
A többi üzenettípus (`cpm`, `vam`, `spatem`, `mapem`, `mcm`, `ssem`, `srem`, `rtcmem`, `ivim`, `imzm`, `evcsnm`, `evrsrm`, `tistpgm`) jelenleg `enabled=false`. Zenoh-ra vannak konfigurálva, de inaktívak – jövőbeli bővítésre előkészítve.

---

## Összefoglalás – Adatfolyam

```
Autoware lokalizáció
(/localization/kinematic_state)
        |
        v
   cam_send.py
  (ENU → WGS84 konverzió,
   CAM JSON összeállítás)
        |
        v MQTT: vanetza/in/cam
   Mosquitto broker
        |
        v
     socktap
   (Vanetza stack,
    config.ini alapján)
        |
        v  V2X csomag (eth0 / v2x_net)
  [Hálózat / Másik jármű]
        |
        v
     socktap
   (beérkező csomag dekódolás)
        |
        v MQTT: vanetza/out/cam
   Mosquitto broker
        |
        v
  cam_receive.py
  (WGS84 → ENU konverzió,
   TrackedObjects összeállítás)
        |
        v
Autoware perception
(/perception/object_recognition/tracking/objects)
        |
        v
      RViz
```
