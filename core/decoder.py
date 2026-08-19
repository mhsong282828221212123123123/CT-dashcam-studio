import os
import struct
import math


class RealTeslaSEIDecoder:
    def __init__(self, mp4_path):
        self.mp4_path = mp4_path
        self.telemetry_cache = {}
        self.has_sei_data = False
        self.max_speed_kmh = 0
        self.parse_mp4_sei()

    @classmethod
    def quick_check_is_parking(cls, mp4_path):
        if not os.path.exists(mp4_path):
            return False
        try:
            inst = cls.__new__(cls)
            with open(mp4_path, 'rb') as fp:
                offset, size = inst.find_mdat(fp)
                if offset is None:
                    return False

                file_size = os.path.getsize(mp4_path)
                mdat_len = size if size > 0 else (file_size - offset)
                if mdat_len <= 0:
                    return True

                chunk_size = 65536
                sample_ratios = [0.0, 0.25, 0.50, 0.75, 0.90]
                sample_offsets = [offset + int(mdat_len * r) for r in sample_ratios]

                max_speed = 0
                has_gps = False
                found_sei = False

                for s_off in sample_offsets:
                    fp.seek(s_off)
                    buf = fp.read(chunk_size)
                    if not buf:
                        continue

                    pos = 0
                    buf_len = len(buf)
                    while pos < buf_len - 4:
                        nal_size = struct.unpack(">I", buf[pos:pos+4])[0]
                        pos += 4
                        if nal_size < 2 or pos + nal_size > buf_len:
                            break

                        if (buf[pos] & 0x1F) == 6:
                            peek = buf[pos:pos+min(nal_size, 256)]
                            payload = inst.extract_proto_payload(peek)
                            if payload:
                                telemetry = inst.parse_protobuf_wire(payload)
                                if telemetry:
                                    found_sei = True
                                    spd = telemetry.get("speed_kmh", 0)
                                    ap_mode = telemetry.get("ap_mode", "MANUAL")
                                    if spd > max_speed:
                                        max_speed = spd
                                    lat = telemetry.get("lat", 0.0)
                                    lon = telemetry.get("lon", 0.0)
                                    if lat != 0.0 or lon != 0.0:
                                        has_gps = True

                                    if spd > 5 or ap_mode != "MANUAL":
                                        return False
                        pos += nal_size

                if not found_sei or not has_gps:
                    return True
                return max_speed <= 3
        except Exception:
            pass
        return False

    def find_mdat(self, fp):
        fp.seek(0)
        while True:
            header = fp.read(8)
            if len(header) < 8:
                return None, None
            size32, atom_type = struct.unpack(">I4s", header)
            if size32 == 1:
                large = fp.read(8)
                if len(large) < 8:
                    return None, None
                size = struct.unpack(">Q", large)[0] - 16
            else:
                size = size32 - 8
            if atom_type == b'mdat':
                return fp.tell(), size
            else:
                if size > 0:
                    fp.seek(size, 1)

    def strip_emulation_prevention_bytes(self, data: bytes) -> bytes:
        stripped = bytearray()
        zero_count = 0
        for byte in data:
            if zero_count >= 2 and byte == 0x03:
                zero_count = 0
                continue
            stripped.append(byte)
            zero_count = 0 if byte != 0 else zero_count + 1
        return bytes(stripped)

    def extract_proto_payload(self, nal: bytes):
        if not isinstance(nal, bytes) or len(nal) < 2:
            return None
        for i in range(3, len(nal) - 1):
            byte = nal[i]
            if byte == 0x42:
                continue
            if byte == 0x69:
                if i > 2:
                    return self.strip_emulation_prevention_bytes(nal[i + 1:-1])
                break
            break
        return None

    def iter_nals(self, fp, offset: int, size: int):
        NAL_ID_SEI = 6
        fp.seek(offset)
        consumed = 0
        while size == 0 or consumed < size:
            header = fp.read(4)
            if len(header) < 4:
                break
            nal_size = struct.unpack(">I", header)[0]
            consumed += 4
            if nal_size < 2:
                fp.seek(nal_size, 1)
                consumed += nal_size
                continue
            
            nal = fp.read(nal_size)
            consumed += nal_size
            if len(nal) < nal_size:
                break
                
            nal_type = nal[0] & 0x1F
            if nal_type == NAL_ID_SEI:
                yield nal

    def parse_protobuf_wire(self, payload: bytes):
        pos = 0
        length = len(payload)
        
        speed_mps = 0.0
        accel_pct = 0.0
        steering_deg = 0.0
        brake_applied = False
        autopilot_state = 0
        fsd_profile_raw = 0
        left_blinker = False
        right_blinker = False
        lat_deg = 0.0
        lon_deg = 0.0
        heading_deg = 0.0
        
        while pos < length:
            try:
                shift = 0
                tag = 0
                while True:
                    if pos >= length: break
                    b = payload[pos]
                    pos += 1
                    tag |= (b & 0x7f) << shift
                    if not (b & 0x80):
                        break
                    shift += 7
                
                field_number = tag >> 3
                wire_type = tag & 0x07
                
                if wire_type == 0:
                    shift = 0
                    val = 0
                    while True:
                        if pos >= length: break
                        b = payload[pos]
                        pos += 1
                        val |= (b & 0x7f) << shift
                        if not (b & 0x80):
                            break
                        shift += 7
                    if field_number == 7:
                        left_blinker = bool(val)
                    elif field_number == 8:
                        right_blinker = bool(val)
                    elif field_number == 9: 
                        brake_applied = bool(val)
                    elif field_number == 10: 
                        autopilot_state = val
                    elif field_number in (17, 18, 19, 20):
                        fsd_profile_raw = val

                elif wire_type == 5:
                    if pos + 4 > length: break
                    val_float = struct.unpack('<f', payload[pos:pos+4])[0]
                    pos += 4
                    if field_number == 4:
                        speed_mps = val_float
                    elif field_number == 5:
                        accel_pct = val_float
                    elif field_number == 6:
                        steering_deg = val_float

                elif wire_type == 1:
                    if pos + 8 > length: break
                    val_double = struct.unpack('<d', payload[pos:pos+8])[0]
                    pos += 8
                    if field_number == 11 and -90.0 <= val_double <= 90.0:
                        lat_deg = val_double
                    elif field_number == 12 and -180.0 <= val_double <= 180.0:
                        lon_deg = val_double
                    elif field_number == 13 and 0.0 <= val_double <= 360.0:
                        heading_deg = val_double

                elif wire_type == 2:
                    shift = 0
                    sub_len = 0
                    while True:
                        if pos >= length: break
                        b = payload[pos]
                        pos += 1
                        sub_len |= (b & 0x7f) << shift
                        if not (b & 0x80):
                            break
                        shift += 7
                    pos += sub_len

                else:
                    pos += 1
            except Exception:
                pos += 1

        if math.isnan(speed_mps):
            speed_mps = 0.0

        speed_kmh = int(round(speed_mps * 3.6))
        
        ap_modes = {
            0: "MANUAL",
            1: "FSD ACTIVE",
            2: "AUTOPILOT",
            3: "TACC"
        }
        ap_mode_str = ap_modes.get(autopilot_state, "MANUAL")

        fsd_profiles = {
            1: "Chill",
            2: "Standard",
            3: "Assertive",
            4: "Hurry"
        }
        fsd_profile_str = fsd_profiles.get(fsd_profile_raw, None)

        return {
            "speed_kmh": max(0, speed_kmh),
            "steering_deg": int(round(steering_deg)),
            "accel_pct": min(100, max(0, int(round(accel_pct)))),
            "brake_pct": 100 if brake_applied else 0,
            "ap_mode": ap_mode_str,
            "fsd_profile": fsd_profile_str,
            "left_blinker": left_blinker,
            "right_blinker": right_blinker,
            "lat": lat_deg,
            "lon": lon_deg,
            "heading": heading_deg
        }

    def parse_mp4_sei(self):
        if not os.path.exists(self.mp4_path):
            return

        try:
            with open(self.mp4_path, 'rb') as fp:
                offset, size = self.find_mdat(fp)
                if offset is None:
                    return

                frame_no = 0
                last_valid = None

                for nal in self.iter_nals(fp, offset, size):
                    payload = self.extract_proto_payload(nal)
                    if not payload:
                        continue

                    parsed = self.parse_protobuf_wire(payload)
                    if parsed:
                        last_valid = parsed
                        self.telemetry_cache[frame_no] = parsed
                        self.has_sei_data = True
                        if parsed["speed_kmh"] > self.max_speed_kmh:
                            self.max_speed_kmh = parsed["speed_kmh"]
                    elif last_valid:
                        self.telemetry_cache[frame_no] = dict(last_valid)

                    frame_no += 1

        except Exception as e:
            print(f"[!] SEI Parse Notice: {e}")

    def get_frame_telemetry(self, frame_idx, fps=30.0):
        if self.has_sei_data and frame_idx in self.telemetry_cache:
            return self.telemetry_cache[frame_idx]

        if self.has_sei_data and self.telemetry_cache:
            closest_idx = min(self.telemetry_cache.keys(), key=lambda k: abs(k - frame_idx))
            return self.telemetry_cache[closest_idx]

        return {
            "speed_kmh": 0,
            "steering_deg": 0,
            "accel_pct": 0,
            "brake_pct": 0,
            "ap_mode": "MANUAL",
            "fsd_profile": None,
            "left_blinker": False,
            "right_blinker": False,
            "lat": 0.0,
            "lon": 0.0,
            "heading": 0.0
        }
