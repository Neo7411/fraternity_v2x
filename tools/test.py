import paho.mqtt.client as mqtt

# Beállítások
BROKER_ADDRESS = "localhost"  # Írd át a bróker IP címére, ha nem a saját gépeden fut
PORT = 1883                   # Az MQTT alapértelmezett portja
TOPIC = "vanetza/out/cam"

# Ez a függvény fut le, amikor sikeresen csatlakozunk a brókerhez
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Sikeresen csatlakozva a brókerhez! Feliratkozás: {TOPIC} ...")
        client.subscribe(TOPIC)
    else:
        print(f"Hiba a csatlakozáskor! Hibakód: {rc}")

# Ez a függvény fut le, amikor új üzenet érkezik
def on_message(client, userdata, msg):
    try:
        # Megpróbáljuk UTF-8 szövegként dekódolni a csomagot
        payload = msg.payload.decode('utf-8')
    except UnicodeDecodeError:
        # Ha bináris adat (pl. nyers bájtok), akkor így írjuk ki
        payload = msg.payload
        
    print(f"[{msg.topic}] -> {payload}")

# Kliens példányosítása
client = mqtt.Client()

# Eseménykezelők (callbackek) hozzárendelése
client.on_connect = on_connect
client.on_message = on_message

print(f"Csatlakozás a brókerhez: {BROKER_ADDRESS}:{PORT}")

# Csatlakozás
try:
    client.connect(BROKER_ADDRESS, PORT, 60)
except Exception as e:
    print(f"Nem sikerült csatlakozni a brókerhez: {e}")
    exit(1)

# Végtelen ciklus, ami figyeli a bejövő forgalmat (CTRL+C-vel megszakítható)
try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nKilépés...")
    client.disconnect()