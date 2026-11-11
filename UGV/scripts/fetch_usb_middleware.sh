#!/usr/bin/env bash
# Copy ST USB Device Library (core + CDC) tu goi framework cua PlatformIO vao project.
# Neu khong co san, dat USB_MW_DIR tro toi thu muc STM32_USB_Device_Library
# (repo: https://github.com/STMicroelectronics/stm32-mw-usb-device).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$(cd "$HERE/.." && pwd)"
MW="${USB_MW_DIR:-${PLATFORMIO_CORE_DIR:-$HOME/.platformio}/packages/framework-stm32cubeh7/Middlewares/ST/STM32_USB_Device_Library}"

[ -d "$MW/Core" ] || { echo "Khong thay $MW/Core - dat USB_MW_DIR"; exit 1; }

mkdir -p "$PROJECT/src/usb/mw" "$PROJECT/include/usb_mw"
cp "$MW"/Core/Src/{usbd_core,usbd_ctlreq,usbd_ioreq}.c "$PROJECT/src/usb/mw/"
cp "$MW"/Class/CDC/Src/usbd_cdc.c                       "$PROJECT/src/usb/mw/"
cp "$MW"/Core/Inc/{usbd_core,usbd_ctlreq,usbd_def,usbd_ioreq}.h "$PROJECT/include/usb_mw/"
cp "$MW"/Class/CDC/Inc/usbd_cdc.h                       "$PROJECT/include/usb_mw/"
echo ">> Da copy USB middleware tu: $MW"
