import socket
import time
import struct
from dataclasses import dataclass
import numpy as np
import math
import enum

MULTICAST_IP = "236.6.7.9"
PORT = 6679
DATA_IP="236.6.7.8"
DATA_PORT = 6678
NAVICO_SPOKE_LEN = 1024
NAVICO_SPOKES = 2048
RADAR_IP = "192.168.1.142"


test_image = np.zeros((NAVICO_SPOKES, NAVICO_SPOKE_LEN), dtype=np.uint8)
test_image[:256, :] = 255
test_image[-256:, :] = 255
test_image[:, :256] = 255
test_image[:, -256:] = 255

class Mode(enum.IntEnum):
    CUSTOM = 0
    HARBOR = 1
    OFFSHORE = 2
    BIRD = 4
    WEATHER = 5

class SeaAuto(enum.IntEnum):
    OFF = 0
    HARBOR = 1
    OFFSHORE = 2
    
@dataclass
class RadarReport_02C4_99:
    range: int = 0
    field4: int = 0
    mode: Mode = Mode.OFFSHORE
    field8: int = 0
    gain: int = 0
    sea_auto: SeaAuto = SeaAuto.OFFSHORE
    field14: int = 0
    field15: int = 0
    sea: int = 0
    field21: int = 0
    rain: int = 0
    field23: int = 0
    field24: int = 0
    field28: int = 0
    field32: int = 0
    field33: int = 0
    interference_rejection: int = 0
    field35: int = 0
    field36: int = 0
    field37: int = 0
    target_expansion: int = 0
    field39: int = 0
    field40: int = 0
    field41: int = 0
    target_boost: int = 0
    
    def to_bytes(self) -> bytes:
        res = struct.pack(
            '<BBIBBIBBBHIBBBIIBBBBBBBBBBB',
            0x02,
            0xC4,
            self.range,
            self.field4,
            self.mode,
            self.field8,
            self.gain,
            self.sea_auto,
            self.field14,
            self.field15,
            self.sea,
            self.field21,
            self.rain,
            self.field23,
            self.field24,
            self.field28,
            self.field32,
            self.field33,
            self.interference_rejection,
            self.field35,
            self.field36,
            self.field37,
            self.target_expansion,
            self.field39,
            self.field40,
            self.field41,
            self.target_boost
        )
        return res + (b'\0' * (99 - len(res)))
    
radar_settings = RadarReport_02C4_99(range=1000)
radar_settings_bytes = radar_settings.to_bytes()
                    
@dataclass
class RadarReport_01C4_18:
    what: int = 0x01
    command: int = 0xC4
    radar_status: int = 0x02  # RADAR_TRANSMIT
    field3: int = 0
    field4: int = 0
    field5: int = 0
    field6: int = 0
    field8: int = 0
    field10: int = 0

    def to_bytes(self):
        return struct.pack('<BBBBBBHHHBBBBBB', self.what, self.command, self.radar_status,
                           self.field3, self.field4, self.field5, self.field6, self.field8, self.field10, 0,0,0,0,0,0)

    
status_report = RadarReport_01C4_18(radar_status=0x02)
status_report_bytes = status_report.to_bytes()

def create_radar_report_01B2():
    packet_id = struct.pack(">H", 0x01B2)  # 2 bytes: Packet ID (0x01B2)
    serialno = b"BR24SIM000001\x00\x00\x00\x00"  # 16-byte serial number, null-terminated

    # Generate the repeated radar IP addresses
    RADAR_IP_PACKED = socket.inet_aton(RADAR_IP) + b"\x00\x00"  # 4 bytes IP + 2 bytes padding
    packed_addresses = RADAR_IP_PACKED * 16  # 16 occurrences of the same radar IP

    # Fill the fixed unknown bytes from the C++ struct
    unknown_blocks = (
        b"\x11\x00\x00\x00" +  # u1
        b"\x11\x00\x00\x00" +  # u2
        b"\x1F\x00\x20\x01\x02\x00\x10\x00\x00\x00" +  # u3
        b"\x11\x00\x00\x00" +  # u4
        b"\x10\x00\x20\x01\x03\x00\x10\x00\x00\x00" +  # u5
        b"\x11\x00\x00\x00" +  # u6
        b"\x12\x00\x00\x00" +  # u7
        b"\x10\x00\x20\x02\x03\x00\x10\x00\x00\x00" +  # u8
        b"\x11\x00\x00\x00" +  # u9
        b"\x12\x00\x00\x00" +  # u10
        b"\x12\x00\x20\x01\x03\x00\x10\x00\x00\x00" +  # u11
        b"\x11\x00\x00\x00" +  # u12
        b"\x12\x00\x00\x00" +  # u13
        b"\x12\x00\x20\x02\x03\x00\x10\x00\x00\x00" +  # u14
        b"\x11\x00\x00\x00" +  # u15
        b"\x12\x00\x00\x00"    # u16
    )

    return packet_id + serialno + packed_addresses + unknown_blocks

radar_report_01B2_bytes = create_radar_report_01B2()

lookupNibbleToByte = [
    0x00, 0x32, 0x40, 0x4E, 0x5C, 0x6A, 0x78, 0x86,
    0x94, 0xA2, 0xB0, 0xBE, 0xCC, 0xDA, 0xE8, 0xF4
]
lookup_low = np.array([lookupNibbleToByte[b & 0x0F] for b in range(256)], dtype=np.uint8)
lookup_high = np.array([lookupNibbleToByte[b >> 4] for b in range(256)], dtype=np.uint8)

def pack_data(data):
    """Convert the spoke data to a packed binary format."""
    # Pack two pixels per byte using lookup tables
    packed_data = np.zeros(NAVICO_SPOKE_LEN // 2, dtype=np.uint8)
    packed_data[:] = (lookup_high[data[::2]] << 4) | (lookup_low[data[1::2]])
    return packed_data
    
@dataclass
class RadarSpoke:
    status: int
    scan_number: int  # 2 bytes
    mark: int  # 4 bytes
    angle: int  # 2 bytes
    heading: int  # 2 bytes
    range: int  # 4 bytes
    u01: int = 0  # 2 bytes
    u02: int = 0  # 2 bytes
    u03: int = 0  # 4 bytes
    data: np.ndarray = None  # 1024 values per spoke

    def to_bytes(self) -> bytes:
        return struct.pack(
            "<BBH I H H I H H I",
            24, #self.headerLen,
            self.status,
            self.scan_number,
            self.mark,
            self.angle,
            self.heading,
            int(self.range * math.sqrt(2.0) / 10.0),
            self.u01,
            self.u02,
            self.u03) + pack_data(self.data).tobytes()

class RadarImage(object):
    """Convert an image array (2048x1024) to radar spoke packets and send via UDP."""

    header = struct.pack("<BBBBBBBB", 0,0,0,0,0,0,0,0)
    
    def __init__(self, image):
        self.image = image
        self.spoke_idx = 0

    def send(self, sock):
        packet = self.header
        for spoke_idx2 in range(self.spoke_idx, self.spoke_idx+32):
            angle_raw = spoke_idx2 * 2  # Reverse the MOD_SPOKES(angle_raw / 2) calculation
            heading_raw = 0
            range_meters = 1000
            spoke_data = self.image[spoke_idx2, :].astype(np.uint8)

            # Create and send a radar spoke packet
            packet += RadarSpoke(0x02, spoke_idx2, 0, angle_raw, heading_raw, range_meters, data=spoke_data).to_bytes()
            
        sock.sendto(packet, (DATA_IP, DATA_PORT))
        self.spoke_idx = (self.spoke_idx + 32) % NAVICO_SPOKES

class Timers(object):
    def __init__(self):
        self.timers = []
        for name in dir(self):
            fn = getattr(self, name, None)
            if hasattr(fn, "__often__"):
                self.timers.append([fn, fn.__often__, 0])

    @classmethod
    def timer(cls, often):
        def timer(fn):
            fn.__often__ = often
            return fn
        return timer

    def run(self):
        while True:
            now = time.time()
            nxt = now + 10
            for idx in range(len(self.timers)):
                fn, often, when = self.timers[idx]
                if now > when:
                    print("Running %s..." % (fn.__name__,))
                    fn()
                    self.timers[idx][2] = now + often
                if self.timers[idx][2] < nxt:
                    nxt = self.timers[idx][2]
            now = time.time()
            sleeptime = nxt - now
            if sleeptime > 0: time.sleep(sleeptime)

class Sender(Timers):
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(RADAR_IP))
        self.sock.bind((RADAR_IP, 0))
        self.image = RadarImage(test_image)
        Timers.__init__(self)
 
    @Timers.timer(0.1)
    def send_status_report(self):
        self.sock.sendto(status_report_bytes, (MULTICAST_IP, PORT))

    @Timers.timer(0.1)
    def send_radar_settings(self):
        self.sock.sendto(radar_settings_bytes, (MULTICAST_IP, PORT))

    @Timers.timer(0.1)
    def send_radar_report_01B2(self):
        self.sock.sendto(radar_report_01B2_bytes, (MULTICAST_IP, PORT))

    @Timers.timer(0.015)
    def send_image(self):
        self.image.send(self.sock)
    
        
if __name__ == "__main__":
    Sender().run()
