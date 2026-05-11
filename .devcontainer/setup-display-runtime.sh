#!/usr/bin/env bash
set -euo pipefail

TARGET_RUNTIME="/tmp/host-runtime-dir"
TARGET_X11="/tmp/.X11-unix"

uid_num="$(id -u)"

wslg_runtime="/host/mnt/wslg/runtime-dir"
wslg_runtime_alt="/host/mnt/wslg/run/user/${uid_num}"
linux_runtime="/host/run/user/${uid_num}"
wslg_x11="/host/mnt/wslg/.X11-unix"
linux_x11="/host/tmp/.X11-unix"

runtime_source=""
if [[ -S "${wslg_runtime}/wayland-0" ]]; then
  runtime_source="${wslg_runtime}"
elif [[ -S "${wslg_runtime_alt}/wayland-0" ]]; then
  runtime_source="${wslg_runtime_alt}"
elif [[ -S "${linux_runtime}/wayland-0" ]]; then
  runtime_source="${linux_runtime}"
elif [[ -d "${wslg_runtime}" ]]; then
  runtime_source="${wslg_runtime}"
elif [[ -d "${wslg_runtime_alt}" ]]; then
  runtime_source="${wslg_runtime_alt}"
elif [[ -d "${linux_runtime}" ]]; then
  runtime_source="${linux_runtime}"
fi

x11_source=""
if [[ -d "${wslg_x11}" ]]; then
  x11_source="${wslg_x11}"
elif [[ -d "${linux_x11}" ]]; then
  x11_source="${linux_x11}"
fi

if [[ -z "${runtime_source}" || -z "${x11_source}" ]]; then
  echo "Warning: GUI paths were not fully detected in this session."
  echo "  runtime: ${runtime_source:-none}"
  echo "  x11: ${x11_source:-none}"
  rm -rf "${TARGET_RUNTIME}" "${TARGET_X11}"
  mkdir -p "${TARGET_RUNTIME}" "${TARGET_X11}"
  exit 0
fi

rm -rf "${TARGET_RUNTIME}" "${TARGET_X11}"
ln -s "${runtime_source}" "${TARGET_RUNTIME}"
ln -s "${x11_source}" "${TARGET_X11}"

echo "Display runtime configured"
echo "  runtime: ${runtime_source} -> ${TARGET_RUNTIME}"
echo "  x11: ${x11_source} -> ${TARGET_X11}"
