#!/usr/bin/env bash
set -Eeuo pipefail
if [[ $EUID -ne 0 || $# -lt 1 ]]; then
    echo "Usage: sudo $0 <wireless-interface> [wireless-interface ...]" >&2
    exit 1
fi
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
for interface in "$@"; do
    [[ $interface =~ ^[a-zA-Z0-9_.:-]+$ ]] || { echo "Invalid interface: $interface" >&2; exit 1; }
done
install -d -m 0755 /etc/surf /etc/NetworkManager/dispatcher.d /usr/local/sbin
printf 'SURF_RADIO_INTERFACES=%q\n' "$*" > /etc/surf/radio-policy.env
install -m 0755 "$project_dir/surf-radio-policy" /usr/local/sbin/
install -m 0755 "$project_dir/90-surf-radio-policy" /etc/NetworkManager/dispatcher.d/
install -m 0644 "$project_dir/surf-radio-policy.service" /etc/systemd/system/
install -m 0644 "$project_dir/surf-radio-policy.timer" /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now surf-radio-policy.timer
systemctl start surf-radio-policy.service
for interface in "$@"; do
    profile=$(nmcli -g GENERAL.CONNECTION device show "$interface" 2>/dev/null || true)
    if [[ -n $profile && $profile != -- ]]; then
        nmcli connection modify "$profile" 802-11-wireless.powersave 2 || true
    fi
done
echo "Installed persistent power-save-off policy for: $*"
