#!/usr/bin/env bash
# Create /dev/hidraw* nodes for the Logitech wheel inside this (privileged) dev
# container and grant the dev user access.
#
# Why this is needed: the container's /dev is a private tmpfs, so the wheel's
# hidraw nodes that exist on the host (visible under /sys/class/hidraw) are not
# created here. wheel_node.py needs hidraw for the preferred input path and for
# force feedback; without these nodes it silently falls back to /dev/input/js*.
#
# Safe to re-run. Run via sudo (or as root). Re-run after replugging the wheel
# or restarting the container.
set -euo pipefail

# Keywords matching the wheel HID_NAME (mirrors WHEEL_NAME_KEYWORDS in wheel_node.py).
KEYWORDS_REGEX='g923|g29|g920|racing wheel|driving force|thrustmaster|t598|t300|t248|t150|tmx|ts-xw|t-gt'

# User who should own the nodes (the dev container user).
TARGET_USER="${SUDO_USER:-${USER:-$(id -un)}}"

created=0
for dev_dir in /sys/class/hidraw/hidraw*; do
    [ -e "$dev_dir/device/uevent" ] || continue

    name="$(grep -m1 '^HID_NAME=' "$dev_dir/device/uevent" | cut -d= -f2-)"
    shopt -s nocasematch
    if [[ ! "$name" =~ $KEYWORDS_REGEX ]]; then
        shopt -u nocasematch
        continue
    fi
    shopt -u nocasematch

    node="/dev/$(basename "$dev_dir")"        # e.g. /dev/hidraw1
    majmin="$(cat "$dev_dir/dev")"            # e.g. 240:1
    major="${majmin%%:*}"
    minor="${majmin##*:}"

    if [ ! -e "$node" ]; then
        mknod "$node" c "$major" "$minor"
    fi
    chown "$TARGET_USER" "$node"
    chmod 0660 "$node"
    echo "ready: $node ($majmin) -> $name [owner $TARGET_USER]"
    created=$((created + 1))
done

# Also create the matching /dev/input/event* node(s) for the wheel. Force feedback on
# Thrustmaster wheels (via hid-tmff2) goes through evdev FF_CONSTANT, which needs the
# event node — and the container's /dev/input is a private tmpfs like /dev/hidraw.
for ev_dir in /sys/class/input/event*; do
    [ -e "$ev_dir/device/name" ] || continue

    name="$(cat "$ev_dir/device/name")"
    shopt -s nocasematch
    if [[ ! "$name" =~ $KEYWORDS_REGEX ]]; then
        shopt -u nocasematch
        continue
    fi
    shopt -u nocasematch

    node="/dev/input/$(basename "$ev_dir")"   # e.g. /dev/input/event13
    majmin="$(cat "$ev_dir/dev")"
    major="${majmin%%:*}"
    minor="${majmin##*:}"

    mkdir -p /dev/input
    if [ ! -e "$node" ]; then
        mknod "$node" c "$major" "$minor"
    fi
    chown "$TARGET_USER" "$node"
    chmod 0660 "$node"
    echo "ready: $node ($majmin) -> $name [owner $TARGET_USER]"
    created=$((created + 1))
done

if [ "$created" -eq 0 ]; then
    echo "No matching wheel hidraw device found under /sys/class/hidraw." >&2
    echo "Is the wheel plugged in and seen by the host kernel?" >&2
    exit 1
fi
