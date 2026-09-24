#!/usr/bin/env python3
"""Read ESCape32 Link Wi-Fi startup diagnostics over native USB."""

from __future__ import annotations

import argparse
import struct
import sys

from escape32_programmer import (
    Escape32Serial,
    ProgrammerError,
    ensure_link_adapter,
    print_adapter_info,
)

ADAPTER_CMD_WIFI_DIAG = 16
WIFI_DIAG_STRUCT = struct.Struct("<BBBBiII")

WIFI_STATES = {
    0: "DISABLED",
    1: "STARTING",
    2: "READY",
    3: "ERROR",
}

WIFI_STAGES = {
    0: "NONE",
    1: "NETIF_INIT",
    2: "EVENT_LOOP",
    3: "AP_NETIF",
    4: "WIFI_INIT",
    5: "SET_MODE",
    6: "SET_CONFIG",
    7: "WIFI_START",
    8: "MDNS_INIT",
    9: "MDNS_HOSTNAME",
    10: "HTTP_START",
    11: "HTTP_404",
    12: "HTTP_ROOT",
    13: "HTTP_WS",
    14: "HTTP_EVENT_CONNECTED",
    15: "HTTP_EVENT_DISCONNECTED",
    16: "DNS_TASK",
    17: "READY",
}

RESET_REASONS = {
    0: "UNKNOWN",
    1: "POWERON",
    2: "EXT",
    3: "SW",
    4: "PANIC",
    5: "INT_WDT",
    6: "TASK_WDT",
    7: "WDT",
    8: "DEEPSLEEP",
    9: "BROWNOUT",
    10: "SDIO",
    11: "USB",
    12: "JTAG",
    13: "EFUSE",
    14: "PWR_GLITCH",
    15: "CPU_LOCKUP",
}

ESP_ERRORS = {
    0: "ESP_OK",
    -1: "ESP_FAIL",
    0x101: "ESP_ERR_NO_MEM",
    0x102: "ESP_ERR_INVALID_ARG",
    0x103: "ESP_ERR_INVALID_STATE",
    0x104: "ESP_ERR_INVALID_SIZE",
    0x105: "ESP_ERR_NOT_FOUND",
    0x106: "ESP_ERR_NOT_SUPPORTED",
    0x107: "ESP_ERR_TIMEOUT",
    0x108: "ESP_ERR_INVALID_RESPONSE",
    0x109: "ESP_ERR_INVALID_CRC",
    0x10A: "ESP_ERR_INVALID_VERSION",
    0x10B: "ESP_ERR_INVALID_MAC",
    0x10C: "ESP_ERR_NOT_FINISHED",
}


def error_text(value: int) -> str:
    name = ESP_ERRORS.get(value)
    raw = value & 0xFFFFFFFF
    if name:
        return f"{name} (0x{raw:08X})"
    return f"0x{raw:08X} ({value})"


def main() -> int:
    parser = argparse.ArgumentParser(description="ESCape32 Link Wi-Fi startup diagnostic reader")
    parser.add_argument("--port", required=True, help="Native USB serial port, e.g. COM7")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        with Escape32Serial(args.port, verbose=args.verbose) as transport:
            print(f"Port.................... {args.port}")
            adapter, _, _ = ensure_link_adapter(transport)
            info = adapter.wifi_status()
            print_adapter_info(info)

            payload = adapter.request_raw(ADAPTER_CMD_WIFI_DIAG)
            if len(payload) != WIFI_DIAG_STRUCT.size:
                raise ProgrammerError(
                    f"Unexpected Wi-Fi diagnostic length: {len(payload)} "
                    f"(expected {WIFI_DIAG_STRUCT.size})"
                )

            status, state, stage, reset_reason, error, free_heap, min_free_heap = WIFI_DIAG_STRUCT.unpack(payload)
            if status != 0:
                raise ProgrammerError(f"Wi-Fi diagnostic status is {status}")

            state_name = WIFI_STATES.get(state, f"UNKNOWN({state})")
            stage_name = WIFI_STAGES.get(stage, f"UNKNOWN({stage})")
            reset_name = RESET_REASONS.get(reset_reason, f"UNKNOWN({reset_reason})")

            print(f"Wi-Fi state............. {state_name}")
            print(f"Wi-Fi stage............. {stage_name}")
            print(f"Wi-Fi error............. {error_text(error)}")
            print(f"Free heap............... {free_heap} bytes")
            print(f"Minimum free heap....... {min_free_heap} bytes")
            print(f"Reset reason............ {reset_name} ({reset_reason})")
            print(f"Wi-Fi health............ {'PASS' if state == 2 else state_name}")
            print("\nRESULT: PASS")
            return 0
    except ProgrammerError as exc:
        print("\nRESULT: FAIL")
        print(f"ERROR : {exc}")
        return 2
    except KeyboardInterrupt:
        print("\nRESULT: ABORTED")
        return 130


if __name__ == "__main__":
    sys.exit(main())
