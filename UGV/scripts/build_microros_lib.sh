#!/usr/bin/env bash
# Build libmicroros.a cho STM32H750 (Cortex-M7, soft-float, newlib-nano).
# Chay tren may PC/laptop co Docker (KHONG chay tren Orange Pi).
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"      # phai khop distro ROS 2 tren Orange Pi
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$(cd "$HERE/.." && pwd)"
WORK="$PROJECT/.uros_build"
OUT_DIR="$PROJECT/third_party/microros"

mkdir -p "$WORK"
cp "$HERE/Makefile.cflags" "$WORK/Makefile"
cd "$WORK"

if [ ! -d micro_ros_stm32cubemx_utils ]; then
  git clone -b "$ROS_DISTRO" https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
fi

META="$(find micro_ros_stm32cubemx_utils -name colcon.meta | head -1)"
echo ">> File cau hinh bo nho: $WORK/$META"
echo ">> Hay sua (giam so node/publisher/subscription...) neu can, roi nhan Enter."
read -rp ""

docker pull "microros/micro_ros_static_library_builder:$ROS_DISTRO"
docker run -it --rm -v "$WORK":/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  "microros/micro_ros_static_library_builder:$ROS_DISTRO"

LIB="$WORK/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros"
mkdir -p "$OUT_DIR/include"
cp "$LIB/libmicroros.a" "$OUT_DIR/"
cp -r "$LIB/microros_include/." "$OUT_DIR/include/"
echo ">> Xong: $OUT_DIR"
