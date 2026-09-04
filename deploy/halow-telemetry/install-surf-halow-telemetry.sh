#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "Usage: sudo $0 <telemetry-user> <halow-interface>" >&2
    exit 1
fi
if [[ $# -ne 2 ]]; then
    echo "Usage: sudo $0 <telemetry-user> <halow-interface>" >&2
    exit 1
fi

telemetry_user=$1
halow_interface=$2
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if ! getent passwd "$telemetry_user" >/dev/null; then
    echo "Unknown telemetry user: $telemetry_user" >&2
    exit 1
fi
if [[ ! "$halow_interface" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
    echo "Invalid interface name: $halow_interface" >&2
    exit 1
fi
for command_name in install systemctl; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "Required command not found: $command_name" >&2
        exit 1
    fi
done

install -d -m 0755 /etc/surf /usr/local/sbin
install -m 0755 "$project_dir/surf-halow-debugfs-exporter" \
    /usr/local/sbin/surf-halow-debugfs-exporter
install -m 0644 "$project_dir/surf-halow-telemetry-exporter.service" \
    /etc/systemd/system/surf-halow-telemetry-exporter.service
printf 'SURF_TELEMETRY_USER=%q\nSURF_HALOW_INTERFACE=%q\nSURF_HALOW_SAMPLE_INTERVAL_SECONDS=1\n' \
    "$telemetry_user" "$halow_interface" > /etc/surf/halow-telemetry.env
chmod 0644 /etc/surf/halow-telemetry.env

systemctl disable --now surf-halow-telemetry-acl.timer 2>/dev/null || true
systemctl stop surf-halow-telemetry-acl.service 2>/dev/null || true
systemctl reset-failed surf-halow-telemetry-acl.service 2>/dev/null || true
systemctl daemon-reload
systemctl enable --now surf-halow-telemetry-exporter.service

snapshot_dir="/run/surf-halow-telemetry/$halow_interface"
for ((attempt = 0; attempt < 50; attempt++)); do
    if [[ -s "$snapshot_dir/mmrc_table_csv" && -e "$snapshot_dir/page_stats" ]]; then
        break
    fi
    sleep 0.1
done

echo "Installed persistent HaLow telemetry export for $telemetry_user on $halow_interface."
echo "Check with: systemctl status surf-halow-telemetry-exporter.service"
if [[ -s "$snapshot_dir/mmrc_table_csv" && -e "$snapshot_dir/page_stats" ]]; then
    echo "First telemetry snapshot is available under $snapshot_dir."
else
    echo "The service is running but the Morse source files are not available yet." >&2
    echo "Check with: journalctl -u surf-halow-telemetry-exporter.service -b" >&2
fi
