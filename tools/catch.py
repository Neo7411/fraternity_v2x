from scapy.all import sniff, Raw

# A Wireshark képed alapján a pontos interfész neve
INTERFACE = "v2x_net"
GEONET_ETHERTYPE = 0x8947

def packet_callback(packet):
    # Ellenőrizzük, hogy Ethernet keret-e és GeoNetworking-e az EtherType
    if packet.haslayer("Ether") and packet["Ether"].type == GEONET_ETHERTYPE:
        print("\n" + "="*50)
        print(" [!] GEONETWORKING CSOMAG ELFOGVA")
        print("="*50)
        print(f" L2 Forrás MAC: {packet['Ether'].src}")
        print(f" L2 Cél MAC:    {packet['Ether'].dst} (Broadcast)")

        # Kinyerjük a nyers bájtokat az Ethernet fejléc után (L3 és felette)
        raw_data = bytes(packet.payload)
        
        # Minimális ellenőrzés, hogy a fejléc hossza megfelelő-e
        if len(raw_data) >= 12:
            # --- Basic Header (Első 4 bájt) ---
            # Az 1. bájt alsó 4 bitje a Next Header (1 = Common Header)
            bh_next_header = raw_data[0] & 0x0F
            hop_limit = raw_data[3]
            
            # --- Common Header (Következő 8 bájt, a 4. bájttól kezdődik) ---
            # Az 5. bájt felső 4 bitje a Next Header (2 = BTP-B)
            ch_next_header = (raw_data[4] >> 4) & 0x0F
            # A 6. bájt a Header Type (0x50 = Single-hop Broadcast / SHB)
            header_type = raw_data[5]
            
            # Kiértékelés a Wireshark adatai alapján
            print(f" L3 Protokoll:  GeoNetworking (Verzió: {raw_data[0] >> 4})")
            
            if header_type == 0x50:
                print(" -> Átvitel típusa: Single-Hop Broadcast (SHB) [0x50]")
            else:
                print(f" -> Átvitel típusa: Egyéb GeoNet (Típus kód: {hex(header_type)})")
                
            print(f" -> Hop Limit:      {hop_limit}")
            
            if ch_next_header == 2:
                print(" L4 Protokoll:  BTP-B (Basic Transport Protocol)")
            else:
                print(f" L4 Protokoll:  Egyéb (Kód: {ch_next_header})")
            
            # --- Hasznos teher (BTP-B + ITS + CPMv1) ---
            # A GeoNet fejlécek összesen 12 bájt után érnek véget az SHB-nál
            payload = raw_data[12:]
            print(f" L7 Adat hossza: {len(payload)} bájt (CPMv1 észlelések)")
            print(f" Nyers adat (hex): {payload.hex()[:60]}...")
        else:
            print(" [!] A kapott csomag túl rövid a GeoNet fejlécek elemzéséhez.")

if __name__ == "__main__":
    print(f"[*] Fülelés indítása a(z) '{INTERFACE}' interfészen...")
    print("[*] Kilépéshez nyomj Ctrl+C-t.")
    
    try:
        # A sniff funkció indítása a megadott interfészen és szűrővel
        sniff(iface=INTERFACE, prn=packet_callback, store=False)
    except PermissionError:
        print("\n [Hiba] Nyers socketek olvasásához root/sudo jogosultság szükséges!")
        print(" Futtasd újra így: sudo python3 <fájlnév>.py")