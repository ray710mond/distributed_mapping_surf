#!/usr/bin/env bash
# One-time cleanup, also called by the network installers.
set -Eeuo pipefail
if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo bash $0" >&2
    exit 1
fi
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
legacy_profile=SURF-Robot-Net
if [[ -r /etc/surf/jetson-network.env ]]; then
    legacy_profile=$(source /etc/surf/jetson-network.env; printf '%s' "${SURF_JETSON_TEST_PROFILE:-SURF-Robot-Net}")
fi
# The old helper tears down only the dedicated virtual AP and SURF daemons.
systemctl disable --now surf-robot-hotspot.service 2>/dev/null || true
if [[ -x /usr/local/sbin/stop-surf-hotspot && -r /etc/surf/local.env ]]; then
    /usr/local/sbin/stop-surf-hotspot
fi
for profile in "$legacy_profile" SURF-Robot-Net SURF-Robot-Net-hotspot; do
    if nmcli connection show "$profile" >/dev/null 2>&1; then
        nmcli connection delete "$profile"
    fi
done
# Older laptop setup also left a generic NetworkManager AP profile.
if [[ $(nmcli -g 802-11-wireless.mode connection show Hotspot 2>/dev/null || true) == ap ]]; then
    nmcli connection delete Hotspot
fi
rm -f /etc/systemd/system/surf-robot-hotspot.service \
    /usr/local/sbin/start-surf-hotspot /usr/local/sbin/stop-surf-hotspot \
    /run/surf-robot-hostapd.conf /run/surf-robot-dnsmasq.conf \
    /run/surf-robot-hostapd.pid /run/surf-robot-dnsmasq.pid \
    /run/NetworkManager/conf.d/90-surf-hotspot-unmanaged.conf
# Remove retired settings, including the hotspot password, without displaying
# configuration contents or touching internet credentials in NetworkManager.
python3 - <<'PY'
from pathlib import Path
import re
pattern = re.compile(r'^\s*(?:export\s+)?(?:SURF_WIFI5_\w+|SURF_AP_IF|SURF_JETSON_TEST_PROFILE|SURF_JETSON_TEST_ADDR)=')
for path in (Path('/etc/surf/local.env'), Path('/etc/surf/jetson-network.env')):
    if path.exists():
        lines = path.read_text().splitlines(keepends=True)
        path.write_text(''.join(line for line in lines if not pattern.match(line)))
PY
# Replace the old internet reconnect command before deleting its dependency.
if [[ -f /etc/surf/local.env ]]; then
    install -m 0755 "$project_dir/../local/surf-normal-mode" /usr/local/sbin/surf-normal-mode
fi
systemctl daemon-reload
nmcli general reload conf
echo "Removed legacy SURF 5 GHz peer profiles and hotspot files."
