#!/usr/bin/env sh
set -eu

# Logitech G923, both variants: PlayStation/PC (c266) and Xbox/PC (c26e).
VENDOR="046d"
PRODUCTS="c266 c26e"

if [ "$(id -u)" -ne 0 ]; then
    echo "Run this with sudo: sudo $0" >&2
    exit 1
fi

RULE_FILE="/etc/udev/rules.d/99-logitech-g923-hidraw.rules"

# Match on the USB parent: a hidraw device's own properties are only MAJOR/MINOR/DEVNAME, so
# ENV{HID_NAME} never matches here. ATTRS walks up to the USB device, where the ids live.
{
    echo "# Logitech G923 wheel hidraw access for direct input and force feedback."
    for product in $PRODUCTS; do
        echo "KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"$VENDOR\", ATTRS{idProduct}==\"$product\", MODE=\"0666\", TAG+=\"uaccess\""
    done
} > "$RULE_FILE"

if command -v udevadm >/dev/null 2>&1; then
    udevadm control --reload-rules || true
    udevadm trigger --subsystem-match=hidraw || true
fi

# Apply to whatever is already plugged in, so this takes effect without a reconnect.
matched=0
for sysfs_dev in /sys/class/hidraw/hidraw*; do
    [ -e "$sysfs_dev/device/uevent" ] || continue

    # HID_ID is "bus:VVVVVVVV:PPPPPPPP" with hex in upper case.
    hid_id=$(grep '^HID_ID=' "$sysfs_dev/device/uevent" 2>/dev/null || true)
    case "$hid_id" in
        *:0000"$(echo "$VENDOR" | tr 'a-f' 'A-F')":0000*) ;;
        *) continue ;;
    esac

    is_g923=0
    for product in $PRODUCTS; do
        upper=$(echo "$product" | tr 'a-f' 'A-F')
        case "$hid_id" in *:0000"$upper") is_g923=1 ;; esac
    done
    [ "$is_g923" -eq 1 ] || continue

    hidraw_name=$(basename "$sysfs_dev")
    dev_path="/dev/$hidraw_name"

    if [ ! -e "$dev_path" ]; then
        major_minor=$(cat "$sysfs_dev/dev")
        mknod "$dev_path" c "${major_minor%:*}" "${major_minor#*:}"
    fi

    chmod 0666 "$dev_path"
    echo "Granted access to $dev_path"
    matched=$((matched + 1))
done

echo "Installed $RULE_FILE"

if [ "$matched" -eq 0 ]; then
    echo "No G923 found. Plug the wheel in and re-run, or check: grep HID_ID /sys/class/hidraw/*/device/uevent" >&2
fi

if [ -f /.dockerenv ]; then
    echo >&2
    echo "WARNING: running inside a container. The chmod above works because /dev is shared with" >&2
    echo "the host, but it is lost when the wheel is reconnected: udev runs on the host and never" >&2
    echo "reads this container's $RULE_FILE. Run this script on the host to make it persist." >&2
fi
