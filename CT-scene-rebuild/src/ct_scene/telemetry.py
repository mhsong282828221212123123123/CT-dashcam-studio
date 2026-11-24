"""Strict H.264 Tesla SEI extraction; packet PTS is not exposure synchronization.

Wire field meanings: https://github.com/teslamotors/dashcam/blob/master/dashcam.proto
Tesla vendor prefix: official dashcam-mp4.js. No legacy decoder dependencies.
"""
import math
import struct
from .artifacts import canonical, publish, digest
from .ingest import file_hash
from pathlib import Path

FIELDS = {
    1: ('version', 0), 2: ('gear_state', 0), 3: ('frame_seq_no', 0),
    4: ('vehicle_speed_mps', 5), 5: ('accelerator_pedal_position', 5),
    6: ('steering_wheel_angle_raw', 5), 7: ('blinker_on_left', 0),
    8: ('blinker_on_right', 0), 9: ('brake_applied', 0),
    10: ('autopilot_state', 0), 11: ('latitude_deg', 1),
    12: ('longitude_deg', 1), 13: ('heading_deg', 1),
    14: ('linear_acceleration_mps2_x', 1), 15: ('linear_acceleration_mps2_y', 1),
    16: ('linear_acceleration_mps2_z', 1),
}


def varint(data, pos):
    value = 0
    for shift in range(0, 70, 7):
        if pos >= len(data):
            raise ValueError('Truncated varint')
        b = data[pos]
        pos += 1
        if shift == 63 and b > 1:
            raise ValueError('Varint overflow')
        value |= (b & 127) << shift
        if not b & 128:
            return value, pos
    raise ValueError('Varint too long')


def decode_metadata(data):
    parsed, pos, unknown = {}, 0, []
    while pos < len(data):
        key, pos = varint(data, pos)
        number, wire = key >> 3, key & 7
        if number == 0:
            raise ValueError('Protobuf field zero')
        if wire == 0:
            value, pos = varint(data, pos)
        elif wire in (1, 5):
            size = 8 if wire == 1 else 4
            if pos+size > len(data):
                raise ValueError('Truncated fixed field')
            value = struct.unpack_from('<d' if size == 8 else '<f', data, pos)[0]
            pos += size
        elif wire == 2:
            size, pos = varint(data, pos)
            if pos+size > len(data):
                raise ValueError('Truncated length field')
            value = None
            pos += size
        else:
            raise ValueError('Unsupported wire type')
        if number in FIELDS:
            name, expected = FIELDS[number]
            if wire != expected:
                raise ValueError('Wrong field wire type: '+name)
            parsed[name] = value
        else:
            unknown.append(number)
    if parsed.get('version') != 1:
        return dict(status='unsupported_version', version=parsed.get('version'), fields={}, unknown_fields=unknown)
    fields = {}
    for number, (name, _) in FIELDS.items():
        value = parsed.get(name, 0)
        valid = math.isfinite(value)
        if number in (2, 10):
            valid &= value in range(4)
        if number in (7, 8, 9):
            valid &= value in (0, 1)
            value = bool(value) if valid else None
        if number == 11:
            valid &= -90 <= value <= 90
        if number == 12:
            valid &= -180 <= value <= 180
        if number == 13:
            valid &= 0 <= value <= 360
        fields[name] = dict(value=value if valid else None, valid=bool(valid),
                            presence='wire' if name in parsed else 'schema_default')
    speed=fields['vehicle_speed_mps']['value']
    flags=['negative_speed_requires_motion_interpretation'] if speed is not None and speed<0 else []
    return dict(status='decoded_v1', fields=fields, unknown_fields=unknown, quality_flags=flags,
                steering_unit='unverified_raw', steering_wheel_rad=None)


def rbsp(data):
    out, zeros = bytearray(), 0
    for b in data:
        if zeros >= 2 and b == 3:
            zeros = 0
            continue
        out.append(b)
        zeros = zeros+1 if b == 0 else 0
    return bytes(out)


def sei_messages(nal):
    if not nal or nal[0] & 31 != 6:
        return []
    data, pos, results = rbsp(nal[1:]), 0, []
    while pos < len(data) and data[pos:] != b'\x80':
        values = []
        for _ in range(2):
            total = 0
            while pos < len(data) and data[pos] == 255:
                total += 255
                pos += 1
            if pos >= len(data):
                raise ValueError('Truncated SEI header')
            total += data[pos]
            pos += 1
            values.append(total)
        kind, size = values
        if pos+size > len(data):
            raise ValueError('Truncated SEI payload')
        payload = data[pos:pos+size]
        pos += size
        if kind != 5:
            continue
        prefix = 0
        while prefix < len(payload) and payload[prefix] == 0x42:
            prefix += 1
        if prefix and prefix < len(payload) and payload[prefix] == 0x69:
            results.append(decode_metadata(payload[prefix+1:]))
    return results


def nal_units(packet, length_size):
    pos = 0
    while pos < len(packet):
        if pos+length_size > len(packet):
            raise ValueError('Truncated NAL length')
        size = int.from_bytes(packet[pos:pos+length_size], 'big')
        pos += length_size
        if size <= 0 or pos+size > len(packet):
            raise ValueError('Invalid NAL length')
        yield packet[pos:pos+size]
        pos += size


def audit_sei(source, destination, duration_s=35):
    import av
    source = Path(source).resolve()
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('Invalid duration')
    source_hash, rows, errors = file_hash(source), [], []
    with av.open(str(source)) as container:
        stream = container.streams.video[0]
        extra = stream.codec_context.extradata
        if stream.codec_context.name != 'h264' or not extra or len(extra) < 5 or extra[0] != 1:
            raise ValueError('Only avcC H.264 supported; no fabricated telemetry')
        length_size = (extra[4] & 3) + 1
        first_time = None
        for packet_index, packet in enumerate(container.demux(stream)):
            if packet.pts is None:
                continue
            t = float(packet.pts * packet.time_base)
            if first_time is None:
                first_time = t
            if t-first_time > duration_s:
                # Packet order is decode order, not necessarily PTS order.
                continue
            try:
                messages = [m for nal in nal_units(bytes(packet), length_size) for m in sei_messages(nal)]
                rows.extend(dict(source_sha256=source_hash, packet_index=packet_index,
                                 packet_pts=packet.pts, camera_time_s=t, session_time_s=None,
                                 clock_status='unverified', **m) for m in messages)
            except ValueError as exc:
                errors.append(dict(packet_index=packet_index, camera_time_s=t, reason=str(exc)))
    summary = dict(source=str(source), source_sha256=source_hash, messages=len(rows),
                   decoded_v1=sum(r['status']=='decoded_v1' for r in rows), errors=errors,
                   no_telemetry=not rows, exposure_sync='NOT_EVALUATED',
                   schema_reference='https://github.com/teslamotors/dashcam/blob/master/dashcam.proto')
    return publish(destination, stage='sei_audit',
                   payloads={'telemetry.jsonl': b''.join(canonical(r)+b'\n' for r in rows),
                             'sei-report.json': canonical(summary)}, inputs=[],
                   config={'source_sha256':source_hash,'duration_s':duration_s},
                   code_revision=digest(Path(__file__).read_bytes()), runtime={'av':av.__version__})
