import binascii
import struct
import time
from collections.abc import Iterable
from dataclasses import dataclass, field

TRANSFER_BYTES = 50
TRANSFER_CRC_INIT = 0xFFFF
CURRENT_MA_PER_UNIT = 10


def parse_tail_byte(tail: int) -> dict[str, bool | int]:
    return {
        "start": bool(tail & 0x80),
        "end": bool(tail & 0x40),
        "toggle": bool(tail & 0x20),
        "transfer_id": tail & 0x1F,
    }


class Battery:
    def __init__(
        self,
        *,
        stale_timeout: float = 1.0,
        max_pack_voltage_mv: int = 51000,
        min_temp_c: int = 10,
        max_temp_c: int = 50,
        max_current_ma: int = 100000,
        min_cell_mv: int = 3300,
        max_cell_mv: int = 4250,
        min_capacity_pct: int = 10,
        min_health_pct: int = 20,
    ):
        self._stale_timeout = stale_timeout
        self._max_pack_voltage_mv = max_pack_voltage_mv
        self._min_temp_c = min_temp_c
        self._max_temp_c = max_temp_c
        self._max_current_ma = max_current_ma
        self._min_cell_mv = min_cell_mv
        self._max_cell_mv = max_cell_mv
        self._min_capacity_pct = min_capacity_pct
        self._min_health_pct = min_health_pct

        self.manufacturer_id = 0
        self.sku_code = 0
        self.pack_voltage_mv = 0
        self.current_ma = 0
        self.temperature_c = 0
        self.capacity_pct = 0
        self.cycle_life = 0
        self.health_pct = 0
        self.cell_voltages_mv: list[int] = [0] * 12
        self.standard_capacity_mah = 0
        self.remaining_capacity_mah = 0
        self.error_flags = 0
        self.last_update = 0.0

    def update_from_payload(self, payload: bytes, now: float):
        if len(payload) != TRANSFER_BYTES:
            raise ValueError(
                f"update battery failed: operation: unpack payload; cause: expected {TRANSFER_BYTES} bytes, got {len(payload)}"
            )
        self.manufacturer_id = struct.unpack_from("<H", payload, 2)[0]
        self.sku_code = struct.unpack_from("<H", payload, 4)[0]
        self.pack_voltage_mv = struct.unpack_from("<H", payload, 6)[0]
        self.current_ma = struct.unpack_from("<h", payload, 8)[0] * CURRENT_MA_PER_UNIT
        self.temperature_c = struct.unpack_from("<h", payload, 10)[0]
        self.capacity_pct = struct.unpack_from("<H", payload, 12)[0]
        self.cycle_life = struct.unpack_from("<H", payload, 14)[0]
        self.health_pct = struct.unpack_from("<H", payload, 16)[0]
        for i in range(12):
            self.cell_voltages_mv[i] = struct.unpack_from("<H", payload, 18 + i * 2)[0]
        self.standard_capacity_mah = struct.unpack_from("<H", payload, 42)[0]
        self.remaining_capacity_mah = struct.unpack_from("<H", payload, 44)[0]
        self.error_flags = struct.unpack_from("<I", payload, 46)[0]
        self.last_update = now

    def is_stale(self, now: float) -> bool:
        return now - self.last_update > self._stale_timeout

    def allow_drive(self, now: float) -> bool:
        if self.is_stale(now):
            return False
        if self.pack_voltage_mv > self._max_pack_voltage_mv:
            return False
        if self.temperature_c < self._min_temp_c or self.temperature_c > self._max_temp_c:
            return False
        if abs(self.current_ma) > self._max_current_ma:
            return False
        for cell_mv in self.cell_voltages_mv:
            if cell_mv < self._min_cell_mv or cell_mv > self._max_cell_mv:
                return False
        if self.capacity_pct < self._min_capacity_pct:
            return False
        if self.health_pct < self._min_health_pct:
            return False
        if self.error_flags != 0:
            return False
        return True


@dataclass
class _Transfer:
    transfer_id: int
    payload: bytearray = field(default_factory=bytearray)
    toggle: bool = False


class BatteryManager:
    def __init__(
        self,
        *,
        installed_can_ids: Iterable[int],
        stale_timeout: float = 1.0,
        max_pack_voltage_mv: int = 51000,
        min_temp_c: int = 10,
        max_temp_c: int = 50,
        max_current_ma: int = 100000,
        min_cell_mv: int = 3300,
        max_cell_mv: int = 4250,
        min_capacity_pct: int = 10,
        min_health_pct: int = 20,
    ):
        self._valid_ids = frozenset(installed_can_ids)
        if not self._valid_ids:
            raise ValueError("configure battery manager failed: cause: no installed packs")
        self._stale_timeout = stale_timeout
        self._max_pack_voltage_mv = max_pack_voltage_mv
        self._min_temp_c = min_temp_c
        self._max_temp_c = max_temp_c
        self._max_current_ma = max_current_ma
        self._min_cell_mv = min_cell_mv
        self._max_cell_mv = max_cell_mv
        self._min_capacity_pct = min_capacity_pct
        self._min_health_pct = min_health_pct
        self.batteries: dict[int, Battery] = {}
        self._transfers: dict[int, _Transfer] = {}

    def process_frame(self, can_id: int, data: bytes | bytearray) -> bool:
        if can_id not in self._valid_ids:
            return False
        if len(data) < 2:
            return False
        tail = parse_tail_byte(data[-1])
        if tail["start"]:
            self._transfers[can_id] = _Transfer(tail["transfer_id"])
        transfer = self._transfers.get(can_id)
        if transfer is None:
            return False
        if tail["transfer_id"] != transfer.transfer_id:
            raise ValueError(
                f"reassemble battery transfer failed: operation: check transfer ID of CAN ID 0x{can_id:08X}; "
                f"cause: expected {transfer.transfer_id}, got {tail['transfer_id']}"
            )
        if tail["toggle"] != transfer.toggle:
            raise ValueError(
                f"reassemble battery transfer failed: operation: check toggle bit of CAN ID 0x{can_id:08X}; "
                f"cause: expected {int(transfer.toggle)}, got {int(tail['toggle'])}"
            )
        if len(transfer.payload) + len(data) - 1 > TRANSFER_BYTES:
            raise ValueError(
                f"reassemble battery transfer failed: operation: append frame of CAN ID 0x{can_id:08X}; "
                f"cause: transfer exceeds {TRANSFER_BYTES} bytes"
            )
        transfer.payload += data[:-1]
        transfer.toggle = not transfer.toggle
        if not tail["end"]:
            return False

        del self._transfers[can_id]
        payload = bytes(transfer.payload)
        if len(payload) != TRANSFER_BYTES:
            raise ValueError(
                f"reassemble battery transfer failed: operation: complete transfer of CAN ID 0x{can_id:08X}; "
                f"cause: expected {TRANSFER_BYTES} bytes, got {len(payload)}"
            )
        received_crc = struct.unpack_from("<H", payload, 0)[0]
        computed_crc = binascii.crc_hqx(payload[2:], TRANSFER_CRC_INIT)
        if received_crc != computed_crc:
            raise ValueError(
                f"reassemble battery transfer failed: operation: check CRC of CAN ID 0x{can_id:08X}; "
                f"cause: received 0x{received_crc:04X}, computed 0x{computed_crc:04X}"
            )
        if can_id not in self.batteries:
            self.batteries[can_id] = Battery(
                stale_timeout=self._stale_timeout,
                max_pack_voltage_mv=self._max_pack_voltage_mv,
                min_temp_c=self._min_temp_c,
                max_temp_c=self._max_temp_c,
                max_current_ma=self._max_current_ma,
                min_cell_mv=self._min_cell_mv,
                max_cell_mv=self._max_cell_mv,
                min_capacity_pct=self._min_capacity_pct,
                min_health_pct=self._min_health_pct,
            )
        self.batteries[can_id].update_from_payload(payload, time.monotonic())
        return True

    def all_safe(self, now: float) -> bool:
        if set(self.batteries) != self._valid_ids:
            return False
        return all(self.batteries[can_id].allow_drive(now) for can_id in self._valid_ids)
