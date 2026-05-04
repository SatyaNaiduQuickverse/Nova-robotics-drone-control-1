"""CRSF (Crossfire) protocol codec. Pure Python, no deps."""

import struct

ADDR_BROADCAST   = 0x00
ADDR_FLIGHT_CTRL = 0xC8
ADDR_REMOTE_CTRL = 0xEC
ADDR_TX_MODULE   = 0xEE
ADDR_RX_MODULE   = 0xEA

FRAME_GPS                = 0x02
FRAME_VARIO              = 0x07
FRAME_BATTERY_SENSOR     = 0x08
FRAME_HEARTBEAT          = 0x0B
FRAME_LINK_STATS         = 0x14
FRAME_RC_CHANNELS_PACKED = 0x16
FRAME_ATTITUDE           = 0x1E
FRAME_FLIGHT_MODE        = 0x21
FRAME_DEVICE_PING        = 0x28
FRAME_DEVICE_INFO        = 0x29

FRAME_NAMES = {
    0x02: 'GPS', 0x07: 'VARIO', 0x08: 'BATTERY', 0x0B: 'HEARTBEAT',
    0x14: 'LINK_STATS', 0x16: 'RC_CHANNELS', 0x1E: 'ATTITUDE',
    0x21: 'FLIGHT_MODE', 0x28: 'DEVICE_PING', 0x29: 'DEVICE_INFO',
}


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def make_frame(addr, frame_type, payload):
    body = bytes([frame_type]) + bytes(payload)
    crc = crc8_dvbs2(body)
    return bytes([addr, len(body) + 1]) + body + bytes([crc])


def pack_channels(channels):
    if len(channels) != 16:
        channels = (list(channels) + [992] * 16)[:16]
    bits, val = 0, 0
    out = bytearray()
    for ch in channels:
        val |= (ch & 0x7FF) << bits
        bits += 11
        while bits >= 8:
            out.append(val & 0xFF)
            val >>= 8
            bits -= 8
    return bytes(out)


def unpack_channels(payload):
    channels, bits, val = [], 0, 0
    for b in payload:
        val |= b << bits
        bits += 8
        while bits >= 11 and len(channels) < 16:
            channels.append(val & 0x7FF)
            val >>= 11
            bits -= 11
    return channels[:16]


def crsf_to_us(ch):
    return int((ch - 992) * 5 / 8 + 1500)


def us_to_crsf(us):
    return int((us - 1500) * 8 / 5 + 992)


def make_rc_channels(channels_us, source=ADDR_FLIGHT_CTRL):
    crsf_vals = [us_to_crsf(us) for us in channels_us]
    return make_frame(source, FRAME_RC_CHANNELS_PACKED, pack_channels(crsf_vals))


def make_battery(voltage_v, current_a=0, mah=0, percent=0, source=ADDR_FLIGHT_CTRL):
    p  = (int(voltage_v * 10)).to_bytes(2, 'big')
    p += (int(current_a * 10)).to_bytes(2, 'big')
    p += bytes([(mah >> 16) & 0xFF, (mah >> 8) & 0xFF, mah & 0xFF])
    p += bytes([percent & 0xFF])
    return make_frame(source, FRAME_BATTERY_SENSOR, p)


def make_flight_mode(text, source=ADDR_FLIGHT_CTRL):
    return make_frame(source, FRAME_FLIGHT_MODE, text.encode('ascii') + b'\x00')


def parse_link_stats(payload):
    if len(payload) < 10:
        return None
    return {
        'uplink_rssi_ant1': -payload[0],
        'uplink_rssi_ant2': -payload[1],
        'uplink_lq':         payload[2],
        'uplink_snr':        struct.unpack('b', bytes([payload[3]]))[0],
        'active_antenna':    payload[4],
        'rf_mode':           payload[5],
        'uplink_tx_power':   payload[6],
        'downlink_rssi':    -payload[7],
        'downlink_lq':       payload[8],
        'downlink_snr':      struct.unpack('b', bytes([payload[9]]))[0],
    }


RF_MODE_NAMES = {
    21: '50Hz', 23: '100Hz Full', 24: '150Hz', 27: '250Hz',
    28: '333Hz Full', 29: '500Hz', 32: 'F500', 33: 'F1000', 36: 'K1000',
}
TX_POWER_NAMES = {
    0: '10mW', 1: '25mW', 2: '50mW', 3: '100mW', 4: '250mW',
    5: '500mW', 6: '1000mW', 7: '2000mW',
}


class CRSFParser:
    """Stateful byte-stream CRSF frame parser."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        while len(self.buf) >= 3:
            if self.buf[0] not in (0xC8, 0xEA, 0xEC, 0xEE, 0x00):
                self.buf.pop(0)
                continue
            length = self.buf[1]
            total = length + 2
            if length < 2 or length > 62:
                self.buf.pop(0)
                continue
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            if crc8_dvbs2(frame[2:-1]) != frame[-1]:
                self.buf.pop(0)
                continue
            self.buf = self.buf[total:]
            yield frame[0], frame[2], frame[3:-1]
