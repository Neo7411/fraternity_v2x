#!/usr/bin/env python3
# Szerző: Angi Dávid

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

import socket
import struct
import time
import math
import asn1tools

class AutowareRealCamBroadcaster(Node):
    def __init__(self):
        super().__init__('real_cam_broadcaster')
        
        # 1. ASN.1 Séma betöltése
        try:
            self.get_logger().info("ASN.1 sémák fordítása (UPER)...")
            self.cam_asn1 = asn1tools.compile_files([
                '/home/aw/vanetza_build/asn1/TS102894-2v131-CDD.asn', 
                '/home/aw/vanetza_build/asn1/EN302637-2v141-CAM.asn'  
            ], 'uper')
            self.get_logger().info("ASN.1 sémák sikeresen betöltve!")
        except Exception as e:
            self.get_logger().error(f"Hiba az ASN.1 betöltésekor: {e}")
            raise

        # 2. TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Bázis koordináták (Debrecen, WGS84)
        self.map_origin_lat = 47.5518  
        self.map_origin_lon = 21.6262

        # 3. Hálózati Socket (Raw Ethernet)
        self.interface = "eth0"
        self.ethertype = 0x8947
        self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
        self.sock.bind((self.interface, 0))

        # MAC Címek
        self.dst_mac = bytes.fromhex('ffffffffffff')
        self.src_mac = bytes.fromhex('aaae7cb625b0') # Konténer MAC címe

        # 10 Hz-es frissítés
        self.timer = self.create_timer(0.1, self.broadcast_cam)

    def xy_to_lat_lon(self, x, y):
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.map_origin_lat))
        
        lat = self.map_origin_lat + (y / meters_per_deg_lat)
        lon = self.map_origin_lon + (x / meters_per_deg_lon)
        return lat, lon

    def broadcast_cam(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # WGS84 Konverzió
            lat, lon = self.xy_to_lat_lon(x, y)
            
            # ASN.1 payload generálása
            cam_payload = self.create_real_asn1_cam(lat, lon)
            
            # L2/L3 csomagolás és küldés
            geonet_packet = self.build_geonet_frame(cam_payload, lat, lon)
            self.sock.send(geonet_packet)
            
            self.get_logger().debug(f"Szabványos CAM elküldve! Hossz: {len(geonet_packet)} byte")

        except TransformException:
            pass # Várjuk a TF fát

    def create_real_asn1_cam(self, lat, lon):
        lat_microdeg = int(lat * 10_000_000)
        lon_microdeg = int(lon * 10_000_000)
        gen_delta_time = int((time.time() * 1000) % 65536)

        cam_dict = {
            'header': {
                'protocolVersion': 2,
                'messageID': 2,       # 2 = CAM
                'stationID': 12345    
            },
            'cam': {
                'generationDeltaTime': gen_delta_time,
                'camParameters': {
                    'basicContainer': {
                        'stationType': 5, # passengerCar
                        'referencePosition': {
                            'latitude': lat_microdeg,
                            'longitude': lon_microdeg,
                            'positionConfidenceEllipse': {
                                'semiMajorConfidence': 10,
                                'semiMinorConfidence': 10,
                                'semiMajorOrientation': 0
                            },
                            'altitude': {
                                'altitudeValue': 800001,   # unavailable
                                'altitudeConfidence': 'unavailable'
                            }
                        }
                    },
                    'highFrequencyContainer': ('basicVehicleContainerHighFrequency', {
                        'heading': {
                            'headingValue': 3601, # unavailable
                            'headingConfidence': 127
                        },
                        'speed': {
                            'speedValue': 0,
                            'speedConfidence': 127
                        },
                        'driveDirection': 'forward',
                        'vehicleLength': {
                            'vehicleLengthValue': 40,
                            'vehicleLengthConfidenceIndication': 'noTrailerPresent'
                        },
                        'vehicleWidth': 20, 
                        'longitudinalAcceleration': {
                            'longitudinalAccelerationValue': 161, # unavailable
                            'longitudinalAccelerationConfidence': 102
                        },
                        'curvature': {
                            'curvatureValue': 1023, # unavailable
                            'curvatureConfidence': 'unavailable'
                        },
                        'curvatureCalculationMode': 'yawRateUsed',
                        'yawRate': {
                            'yawRateValue': 32767, # unavailable
                            'yawRateConfidence': 'unavailable'
                        }
                    })
                }
            }
        }

        return self.cam_asn1.encode('CAM', cam_dict)

    def build_geonet_frame(self, payload, lat, lon):
        # 1. Ethernet Fejléc
        eth_header = struct.pack('!6s6sH', self.dst_mac, self.src_mac, self.ethertype)
        
        # 2. GeoNetworking Basic Header
        gn_basic = struct.pack('!BBBB', 0x11, 0x00, 0x1a, 0x01)
        
        # 3. GeoNetworking Common Header
        payload_len = 4 + len(payload)
        gn_common = struct.pack('!BBBBHBB', 0x20, 0x50, 0x00, 0x00, payload_len, 0x01, 0x00)
        
        # 4. GeoNetworking SHB Extended Header (JAVÍTVA: Pontosan 24 bájt)
        gn_addr_prefix = 0x8000
        gn_addr = struct.pack('!H6s', gn_addr_prefix, self.src_mac)
        
        gn_timestamp = 0
        lat_gn = int(lat * 10_000_000)
        lon_gn = int(lon * 10_000_000)
        pai_speed_heading = 0x80000000 # PAI=1 (Érvényes pozíció)
        
        gn_extended = gn_addr + struct.pack('!IiiI', gn_timestamp, lat_gn, lon_gn, pai_speed_heading)
        
        # 5. BTP-B fejléc - Cél Port: 2001 (CAM)
        btp_b = struct.pack('!HH', 2001, 0x0000)

        return eth_header + gn_basic + gn_common + gn_extended + btp_b + payload

def main(args=None):
    rclpy.init(args=args)
    node = AutowareRealCamBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()