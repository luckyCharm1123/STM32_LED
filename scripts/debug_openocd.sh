#!/usr/bin/env bash
set -euo pipefail

# Start OpenOCD for STM32F1 + ST-Link without semihosting.
# Optional env:
#   OPENOCD_ADAPTER_KHZ=400
#   OPENOCD_CONNECT_UNDER_RESET=1
adapter_khz="${OPENOCD_ADAPTER_KHZ:-1000}"
connect_under_reset="${OPENOCD_CONNECT_UNDER_RESET:-0}"

reset_config_cmd=()
if [[ "${connect_under_reset}" == "1" ]]; then
  reset_config_cmd=(-c "reset_config srst_only srst_nogate connect_assert_srst")
fi

exec openocd \
  -f interface/stlink.cfg \
  -f target/stm32f1x.cfg \
  -c "transport select hla_swd" \
  -c "adapter speed ${adapter_khz}" \
  "${reset_config_cmd[@]}" \
  -c "init" \
  -c "reset halt"
