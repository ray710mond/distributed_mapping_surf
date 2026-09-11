#!/usr/bin/env bash
set -Eeuo pipefail
if [[ $EUID -ne 0 || $# != 1 ]]; then
    echo "Usage: sudo $0 <peer-HaLow-IPv4>" >&2
    exit 1
fi
peer=$1
python3 - "$peer" <<'PY'
import ipaddress, sys
ipaddress.IPv4Address(sys.argv[1])
PY
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# This installer targets Ubuntu's packaged Chrony layout. Fail before changing
# anything if drop-ins would be ignored.
if ! grep -Eq '^confdir[[:space:]]+/etc/chrony/conf.d([[:space:]]|$)' /etc/chrony/chrony.conf; then
    echo "Chrony must include: confdir /etc/chrony/conf.d" >&2
    exit 1
fi
install -d -m 0755 /etc/chrony/conf.d /etc/surf /usr/local/lib/surf
# Remove only configuration owned by previous SURF installers, including the
# old local-clock authority. Never manufacture synchronized time when offline.
rm -f /etc/chrony/conf.d/surf-halow-client.conf /etc/chrony/conf.d/surf-halow-server.conf
cat > /etc/chrony/conf.d/surf-time.conf <<CONFIG
# Internet sources win whenever selectable. The peer is a reciprocal fallback.
pool time.cloudflare.com iburst prefer maxsources 4 minpoll 4 maxpoll 6
server $peer iburst minpoll 2 maxpoll 4
allow $peer/32
CONFIG
# Validate the complete configuration before restarting the daemon.
chronyd -p -f /etc/chrony/chrony.conf >/dev/null
install -m 0644 "$project_dir/../../surf_data_tracker/surf_data_tracker/clock.py" /usr/local/lib/surf/clock.py
install -m 0755 "$project_dir/surf-clock-sync-exporter" /usr/local/sbin/
install -m 0644 "$project_dir/surf-clock-sync-exporter.service" /etc/systemd/system/
printf 'SURF_CLOCK_PEER=%q\n' "$peer" > /etc/surf/clock-sync.env
systemctl restart chrony.service
systemctl daemon-reload
systemctl enable surf-clock-sync-exporter.service
systemctl restart surf-clock-sync-exporter.service
