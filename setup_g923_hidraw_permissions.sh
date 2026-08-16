#!/usr/bin/env sh
set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "Run this with sudo: sudo $0" >&2
    exit 1
fi

RULE_FILE="/etc/udev/rules.d/99-logitech-g923-hidraw.rules"

cat > "$RULE_FILE" <<'EOF'
# Logitech G923 wheel hidraw access for direct input and force feedback.
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ENV{HID_NAME}=="*Logitech G923*", MODE="0666", TAG+="uaccess"
EOF

if command -v udevadm >/dev/null 2>&1; then
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=hidraw || true
fi

for sysfs_dev in /sys/class/hidraw/hidraw*/device/uevent; do
    [ -e "$sysfs_dev" ] || continue

    if ! grep -q '^HID_NAME=.*Logitech G923' "$sysfs_dev"; then
        continue
    fi

    hidraw_name=$(basename "$(dirname "$(dirname "$(dirname "$sysfs_dev")")")")
    dev_path="/dev/$hidraw_name"
    major_minor=$(cat "/sys/class/hidraw/$hidraw_name/dev")
    major=${major_minor%:*}
    minor=${major_minor#*:}

    if [ ! -e "$dev_path" ]; then
        mknod "$dev_path" c "$major" "$minor"
    fi

    chmod 666 "$dev_path"
done

echo "Installed $RULE_FILE"
echo "Reconnect the wheel, then run wheel_node.py again."
