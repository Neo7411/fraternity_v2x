# DENM → EEBL vészfékezés lanelet2 HD térkép alapú sávdöntéssel

A [`tools/denm/denm_rec_lanelet2.py`](../tools/denm/denm_rec_lanelet2.py) egy ROS 2
node, ami MQTT-n érkező DENM üzeneteket fogad, és eldönti, hogy az esemény
**a saját sávunkban, előttünk, hatótávon belül** van-e. Ha igen, vészfékezést
kér az Autoware-től.

A korábbi `denm_receive.py` puszta GPS-távolságot nézett. Ez a verzió HD
térképet használ: az út valódi geometriája mentén számol, így az ívekben és a
szomszédos sávok elkülönítésében is helyes marad.

---

## 1. Hogyan dönt

A matek Frenet-koordinátákban megy, a lanelet2 hivatalos
`toArcCoordinates()` függvényével:

```
EGO lanelet  --following()-->  lánc  -->  LaneletSequence.centerline
                                             |
              toArcCoordinates(centerline, pont)  ->  (s, d)
                                             |
        delta_s = obj.s - ego.s     az ív mentén mért távolság
        d_rel   = obj.d - ego.d     oldalirányú eltérés
        offset  = floor(d_rel / sávszélesség + 0.5)      0 = saját sáv
```

- **`s`** — mennyit haladtunk az útvonal középvonala mentén. A különbsége a
  valódi, ív menti távolság, nem légvonal.
- **`d`** — előjeles oldaltávolság a középvonaltól (bal = pozitív).
- **`offset`** — hány sávval odébb van az esemény. A kerekítés
  `floor(x + 0.5)`, nem `round()`: a Python `round()` bankári kerekítést
  használ (`round(-0.5) == 0`), ami pont a sávhatáron lenne megjósolhatatlan.

### A döntés szűrői, ebben a sorrendben

| # | Szűrő | Elutasítás oka a naplóban |
|---|---|---|
| 1 | `same_lane` — `offset == 0`? | `1 savval balra (+4.4 m)` |
| 2 | `on_route` — az esemény lanelet-je a láncon vagy szomszédján van? | `nem a sajat utvonalunkon (lanelet 110)` |
| 3 | `delta_s >= 0` — előttünk van? | `mogottunk (-99.9 m)` |
| 4 | `min_distance_m <= delta_s <= max_distance_m` | `tavolsag hataron kivul (210.3 m)` |
| 5 | `require_eebl_cause` esetén a DENM cause code | `nem EEBL cause code` |

A sávdöntés szándékosan **megelőzi** az `on_route`-ot: olyan térképen, aminek
nincs `lefts`/`rights` relációja, az `on_route` gyakorlatilag „ugyanaz a
lanelet"-et jelent, és a szomszéd sávra félrevezető indoklást adna.

Egy eseményre (`originatingStationId`, `sequenceNumber` páros) **csak egyszer**
fékezünk. Ha az esemény `event_timeout_s`-ig nem jön újra, lezárjuk — ha később
mégis felbukkan, az már új eseménynek számít.

A GNSS-zaj ellen a node az utolsó `sample_size` darab pozíciót átlagolja, és
legfeljebb `report_period_s`-onként dönt.

---

## 2. Projekció — ez a rész buktatós

A DENM WGS84 szélesség/hosszúságot hordoz, az Autoware és a lanelet2 térkép
viszont lokális méteres koordinátákban dolgozik. **A kettőnek ugyanabba a
koordinátarendszerbe kell esnie**, különben az EGO és az esemény több száz
kilométerre kerül egymástól.

A helyes beállítást a térkép saját leírója mondja meg:

```bash
cat $HOME/maps/highway/map_projector_info.yaml
```

| A yaml `projector_type` értéke | `lanelet2_projector` paraméter |
|---|---|
| `Local` | `local` |
| `LocalCartesian` | `local_cartesian` |
| `MGRS` | `mgrs` |
| `UTM` | `utm` |

### Az Autoware `Local` formátum

Ez a jelenlegi `highway` térkép formátuma, és **a lanelet2 egyik saját
projektora sem kezeli**. Ilyen térképben a node-oknak nincs lat/lon-juk — a
koordináta a `local_x` / `local_y` tag-ben áll:

```xml
<node id="36" lat="" lon="">
  <tag k="local_x" v="-0.2872"/>
  <tag k="local_y" v="-5.7959"/>
  <tag k="ele" v="0"/>
</node>
```

Ha ezt `LocalCartesianProjector`-ral töltöd be, az az üres `lat=""`/`lon=""`
értéket 0-nak olvassa, a `local_x`/`local_y` tag-eket pedig figyelmen kívül
hagyja, így **minden pont a (0,0)-ba kerül**:

```
x range: 0.000 .. 0.000
lanelet 135 first centerline pt 0.0 0.0
```

Ekkor az EGO (aki x ≈ 2200 körül jár) sosem talál sávot, és a node csak ennyit
mond: `EGO nincs egyetlen savon sem`.

Ezért a [`load_local_tag_map()`](../tools/denm/denm_rec_lanelet2.py) betöltés
után visszaírja a pontok koordinátáit az OSM tag-jeiből. A módosítás
átvezetődik a lanelet-ek középvonalára és a `findNearest` térbeli indexére is,
tehát nem kell újratölteni a térképet.

### Origó valódi lat/lon-os térképnél

`local_cartesian` / `mgrs` / `utm` esetén a `map_origin_lat` / `map_origin_lon`
paraméternek **egyeznie kell azzal, amivel a térkép készült**. A `local`
projekció ezt nem használja — ott a térkép már eleve méterben van.

A DENM adó ([`denm_send.py`](../tools/denm/denm_send.py)) és a vevő ugyanazzal
az origóval konvertál ENU ↔ WGS84 oda-vissza, tehát a kettőnek mindig
ugyanazt az értéket kell kapnia.

---

## 3. Indítás

```bash
# a konténerben
source /opt/autoware/setup.bash
python3 /home/aw/tools/denm/denm_rec_lanelet2.py
```

Paraméterekkel (figyelj a `--ros-args`-ra, enélkül a ROS remap szabálynak
nézi és figyelmen kívül hagyja):

```bash
python3 /home/aw/tools/denm/denm_rec_lanelet2.py --ros-args \
    -p lanelet2_map_path:=/home/aw/maps/highway/lanelet2_map.osm \
    -p lanelet2_projector:=local \
    -p max_distance_m:=200.0
```

### Paraméterek

| Paraméter | Alapérték | Jelentés |
|---|---|---|
| `lanelet2_map_path` | `/home/aw/maps/highway/lanelet2_map.osm` | a HD térkép |
| `lanelet2_projector` | `local` | lásd a projekciós táblázatot fent |
| `lane_width_m` | `3.5` | sávszélesség a sávoffset kerekítéséhez |
| `lane_search_radius_m` | `5.0` | ekkora sugárban keres lanelet-et |
| `max_distance_m` | `150.0` | efölött nem releváns az esemény |
| `min_distance_m` | `1.0` | ez alatt sem (saját pozíció zaja) |
| `map_origin_lat` / `_lon` / `_alt` | `47.5316` / `21.6273` / `0.0` | csak lat/lon-os projekciónál |
| `mqtt_broker` / `mqtt_port` | `127.0.0.1` / `1883` | a Vanetza brókere |
| `mqtt_denm_topic` | `vanetza/out/denm` | bejövő DENM topic |
| `own_station_id` | `1` | a saját üzeneteinket eldobjuk |
| `sample_size` | `3` | ennyi pozíciót átlagol (GNSS-zaj) |
| `report_period_s` | `0.5` | legfeljebb ennyiszer dönt másodpercenként |
| `event_timeout_s` | `5.0` | ennyi csend után lezárja az eseményt |
| `require_eebl_cause` | `False` | csak EEBL cause code-ra fékezzen-e |
| `emergency_service` | `/api/autoware/set/emergency` | Autoware vészfék szolgáltatás |

---

## 4. A napló olvasása

Induláskor:

```
terkep: /home/aw/maps/highway/lanelet2_map.osm (4 lanelet, local, origo 47.5316,21.6273)
terkep kiterjedes: x -0.3..4999.4  y -10.9..11.1 m
```

**A kiterjedést mindig nézd meg.** Ha itt `x 0.0..0.0` áll, vagy a számok
nagyságrendje nem stimmel az EGO pozíciójához, akkor rossz a projekció — nem a
sávdöntés a hibás.

Egy döntés:

```
DENM (2001, 1) #3 | lanelet 135->135 (lanc 1) | s: 2797.3->2897.4 delta_s=+100.1 m
                  | d: +0.61->+0.02 d_rel=-0.59 m off=+0 | relevans=True (sajat savban, 100.1 m elottunk)
```

- `(2001, 1)` — az esemény azonosítója (station id, sequence number), `#3` a
  beérkezett üzenetek száma
- `lanelet 135->135` — az EGO és az esemény lanelet-je, `(lanc 1)` a összefűzött
  lanelet-ek száma
- `s: 2797.3->2897.4` — a két pont ív menti pozíciója, a különbség `delta_s`
- `off=+0` — saját sáv

Fékezéskor:

```
EEBL -> VESZFEKEZES kerese elkuldve
VESZFEKEZES AKTIV (...)
```

---

## 5. Hibakeresés

### `EGO nincs egyetlen savon sem`

A warning megmondja az okot:

**a) `legkozelebbi lanelet 110 250.4 m-re (sugar 5.0 m)`**

Az EGO messze van minden sávtól → **projekciós hiba**. Ellenőrizd a
`lanelet2_projector` értékét a `map_projector_info.yaml` ellen, és nézd meg a
naplóban a térkép kiterjedését.

**b) `lanelet 42 0.0 m-re van, de az iranya nem egyezik`**

Az EGO pontosan a sávon áll, de **szemben megy vele**. A `HEADING_TOL = 90°`
szűrő védi meg attól, hogy az ellenirányú sávot saját sávnak vegye.

> **Fontos a jelenlegi `highway` térképnél:** mind a négy lanelet **ugyanabba
> az irányba fut** (−180°, nyugat felé) — nincs benne ellenirányú pálya, hiába
> néz ki négysávos autópályának. Ha az EGO-t kelet felé indítod, egyik sáv
> iránya sem fog egyezni, és mindig ezt a warningot kapod. Amíg a térkép így
> van tájolva, a járműveket a sávok irányába (nyugat felé) kell állítani.

Ellenőrzés:

```bash
python3 -c "
import importlib.util, math
spec = importlib.util.spec_from_file_location('m', '/home/aw/tools/denm/denm_rec_lanelet2.py')
mm = importlib.util.module_from_spec(spec); spec.loader.exec_module(mm)
m = mm.load_local_tag_map('/home/aw/maps/highway/lanelet2_map.osm')
for l in m.laneletLayer:
    c = list(l.centerline)
    print('lanelet %3d  y=%+6.1f  heading=%+4.0f deg'
          % (l.id, c[0].y, math.degrees(math.atan2(c[-1].y-c[0].y, c[-1].x-c[0].x))))
"
```

### `Nincs sajat pozicio`

Nem jön `/localization/kinematic_state` — nem fut a szimulátor, vagy nincs
inicializálva a lokalizáció.

### A node elindul, de egy DENM-re sem reagál

- Megy-e egyáltalán üzenet a topicra: `mosquitto_sub -t 'vanetza/out/denm' -v`
- Nem a saját `own_station_id`-nkkal jön-e (azt eldobjuk)
- Kell-e legalább `sample_size` (alapból 3) üzenet ugyanahhoz az eseményhez

---

## 6. Validáció

A [`tools/denm/lanelet2_gps.py`](../tools/denm/lanelet2_gps.py) ugyanazt a
matekot számolja parancssorból, kézzel megadott koordinátákkal. Ezzel lehet
ellenőrizni, hogy a node döntése helyes-e:

```bash
# milyen sávok vannak
python3 lanelet2_gps.py --osm /home/aw/maps/highway/lanelet2_map.osm --list

# egy konkrét eset
python3 lanelet2_gps.py --osm /home/aw/maps/highway/lanelet2_map.osm \
    --lane 135 --ego-x 2202.07 --ego-y 7.93 --pt-x 2102.0 --pt-y 8.5
```

```
EGO Frenet: s= 2797.33 m  d= +0.607 m
OBJ Frenet: s= 2897.40 m  d= +0.025 m
delta_s =  +100.07 m  (elottunk)
d_rel   =    -0.58 m
lane_offset = +0   maradek -0.58 m
dontes  = UGYANAZ A SAV
```

Ennek egyeznie kell a node naplójával ugyanarra a helyzetre. A script a
`--projector local|local_cartesian` kapcsolóval ugyanazt a betöltést használja,
mint a node.

---

## 7. A jelenlegi `highway` térkép korlátai

Mérésből, nem feltételezésből:

| Tulajdonság | Érték | Következmény |
|---|---|---|
| lanelet-ek száma | 4 | mind ~5 km hosszú |
| kiterjedés | x −0.3…4999.4, y −10.9…11.1 m | egyenes szakasz |
| `following` / `previous` | **nincs** | a lánc mindig 1 elemű; elágazó térképen a láncépítés lesz a következő szűk keresztmetszet |
| `lefts` / `rights` | **nincs** | az `on_route` gyakorlatilag „ugyanaz a lanelet"; sávváltást nem tud követni |
| sávirányok | mind −180° | **ellenirányú forgalmat nem lehet vele tesztelni** |

A sávok középvonala y = +8.5, +3.5, −3.3, −8.3 méteren fut (lanelet 135, 110,
42, 85).
