import binascii
import struct

from terrascout_hardware.tattu_bms import (
    Battery,
    BatteryManager,
    parse_tail_byte,
)

LEFT_BATTERY_CAN_ID = 0x01109216


def test_parse_tail_byte_start():
    tail = 0b10100000
    result = parse_tail_byte(tail)
    assert result["start"] is True
    assert result["end"] is False
    assert result["toggle"] is True


def test_parse_tail_byte_end():
    tail = 0b01000000
    result = parse_tail_byte(tail)
    assert result["start"] is False
    assert result["end"] is True


def test_battery_allow_drive_nominal():
    bat = Battery()
    bat.pack_voltage_mv = 48000
    bat.temperature_c = 25
    bat.current_ma = 5000
    bat.cell_voltages_mv = [3800] * 12
    bat.capacity_pct = 80
    bat.health_pct = 90
    bat.error_flags = 0
    bat.last_update = 1000.0
    assert bat.allow_drive(now=1000.5) is True


def test_battery_unsafe_low_cell():
    bat = Battery()
    bat.pack_voltage_mv = 48000
    bat.temperature_c = 25
    bat.current_ma = 5000
    bat.cell_voltages_mv = [3800] * 11 + [2900]
    bat.capacity_pct = 80
    bat.health_pct = 90
    bat.error_flags = 0
    bat.last_update = 1000.0
    assert bat.allow_drive(now=1000.5) is False


def test_battery_unsafe_stale_data():
    bat = Battery()
    bat.pack_voltage_mv = 48000
    bat.temperature_c = 25
    bat.current_ma = 5000
    bat.cell_voltages_mv = [3800] * 12
    bat.capacity_pct = 80
    bat.health_pct = 90
    bat.error_flags = 0
    bat.last_update = 1000.0
    assert bat.allow_drive(now=1002.0) is False


def test_battery_unsafe_high_voltage():
    bat = Battery()
    bat.pack_voltage_mv = 53000
    bat.temperature_c = 25
    bat.current_ma = 5000
    bat.cell_voltages_mv = [3800] * 12
    bat.capacity_pct = 80
    bat.health_pct = 90
    bat.error_flags = 0
    bat.last_update = 1000.0
    assert bat.allow_drive(now=1000.5) is False


def _build_payload(
    voltage_mv=48000,
    current_ma=5000,
    temp_c=25,
    capacity_pct=80,
    health_pct=90,
    cell_mv=3800,
    error_flags=0,
):
    buf = bytearray(50)
    struct.pack_into("<H", buf, 6, voltage_mv)
    struct.pack_into("<h", buf, 8, current_ma)
    struct.pack_into("<H", buf, 10, temp_c)
    struct.pack_into("<H", buf, 12, capacity_pct)
    struct.pack_into("<H", buf, 16, health_pct)
    for i in range(12):
        struct.pack_into("<H", buf, 18 + i * 2, cell_mv)
    struct.pack_into("<I", buf, 46, error_flags)
    return bytes(buf)


def test_update_from_payload_offsets():
    bat = Battery()
    payload = _build_payload(
        voltage_mv=48000,
        current_ma=-3000,
        temp_c=30,
        capacity_pct=75,
        health_pct=95,
        cell_mv=3900,
        error_flags=0,
    )
    bat.update_from_payload(payload, now=1.0)
    assert bat.pack_voltage_mv == 48000
    assert bat.current_ma == -30000
    assert bat.temperature_c == 30
    assert bat.capacity_pct == 75
    assert bat.health_pct == 95
    assert bat.cell_voltages_mv == [3900] * 12
    assert bat.error_flags == 0


def test_update_from_payload_error_flags():
    bat = Battery()
    payload = _build_payload(error_flags=0x0005)
    bat.update_from_payload(payload, now=1.0)
    assert bat.error_flags == 0x0005
    assert bat.allow_drive(now=1.0) is False


def test_process_frame_ignores_short_data():
    mgr = BatteryManager(installed_can_ids={LEFT_BATTERY_CAN_ID})
    mgr.process_frame(LEFT_BATTERY_CAN_ID, b"\x00\x01")
    assert len(mgr.batteries) == 0


def test_process_frame_ignores_empty_data():
    mgr = BatteryManager(installed_can_ids={LEFT_BATTERY_CAN_ID})
    mgr.process_frame(LEFT_BATTERY_CAN_ID, b"")
    assert len(mgr.batteries) == 0


def test_battery_manager_reassembly():
    mgr = BatteryManager(installed_can_ids={LEFT_BATTERY_CAN_ID})
    can_id = 0x01109216
    payload = bytearray(_build_payload())
    struct.pack_into("<H", payload, 0, binascii.crc_hqx(bytes(payload[2:]), 0xFFFF))
    frames = []
    for i in range(8):
        chunk = bytes(payload[i * 7 : (i + 1) * 7])
        start = 1 if i == 0 else 0
        end = 1 if i == 7 else 0
        toggle = i % 2
        tail = (start << 7) | (end << 6) | (toggle << 5)
        frames.append((can_id, chunk + bytes([tail])))
    for can_id_frame, data in frames:
        mgr.process_frame(can_id_frame, data)
    assert can_id in mgr.batteries
    bat = mgr.batteries[can_id]
    assert bat.pack_voltage_mv == 48000
