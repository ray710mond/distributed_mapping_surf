#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "Run this installer as root: sudo $0" >&2
    exit 1
fi

project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
config_file=${SURF_JETSON_NETWORK_CONFIG:-$project_dir/jetson-network.env}

if [[ ! -r "$config_file" ]]; then
    echo "Copy jetson-network.env.example to jetson-network.env first." >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$config_file"

required_settings=(
    SURF_JETSON_WIFI_IF
    SURF_JETSON_INTERNET_PROFILE
    SURF_JETSON_TEST_PROFILE
    SURF_JETSON_HALOW_IF
    SURF_JETSON_HALOW_PEER
    SURF_JETSON_TEST_ADDR
    SURF_TELEMETRY_USER
)

for setting in "${required_settings[@]}"; do
    if [[ -z ${!setting:-} ]]; then
        echo "$setting is required in $config_file" >&2
        exit 1
    fi
done

for command_name in nmcli ip chronyc; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "Required command not found: $command_name" >&2
        exit 1
    fi
done

nmcli device show "$SURF_JETSON_WIFI_IF" >/dev/null
nmcli device show "$SURF_JETSON_HALOW_IF" >/dev/null
nmcli connection show "$SURF_JETSON_INTERNET_PROFILE" >/dev/null
nmcli connection show "$SURF_JETSON_TEST_PROFILE" >/dev/null

# The robot profile is local-only and remains the default after reboot. Campus
# Internet is entered explicitly and must not displace the robot link later.
nmcli connection modify "$SURF_JETSON_INTERNET_PROFILE" \
    connection.autoconnect no
nmcli connection modify "$SURF_JETSON_TEST_PROFILE" \
    connection.autoconnect yes \
    connection.autoconnect-priority 100 \
    802-11-wireless.powersave 2 \
    ipv4.never-default yes \
    ipv4.gateway "" \
    ipv6.never-default yes
nmcli connection modify "$SURF_JETSON_INTERNET_PROFILE" \
    802-11-wireless.powersave 2

install -d -m 0755 /etc/surf
install -m 0600 "$config_file" /etc/surf/jetson-network.env
install -m 0755 "$project_dir/surf-jetson-internet-mode" /usr/local/sbin/
install -m 0755 "$project_dir/surf-jetson-test-mode" /usr/local/sbin/
install -m 0755 \
    "$project_dir/../../surf_data_tracker/surf_data_tracker/clock.py" \
    /usr/local/bin/clock-sync-status
install -m 0755 "$project_dir/../network/surf-preflight" /usr/local/bin/surf-preflight

# Persist the laptop as the preferred offline source across Chrony and host
# reboots. Internet sources may coexist but cannot replace this preferred peer
# while the field link is reachable.
install -d -m 0755 /etc/chrony/conf.d
printf 'server %s iburst prefer minpoll 2 maxpoll 4\n' "$SURF_JETSON_HALOW_PEER" \
    > /etc/chrony/conf.d/surf-halow-client.conf

install -m 0755 "$project_dir/../clock-sync/surf-clock-sync-exporter" /usr/local/sbin/
install -m 0644 "$project_dir/../clock-sync/surf-clock-sync-exporter.service" \
    /etc/systemd/system/
printf 'SURF_CLOCK_PEER=%q\n' "$SURF_JETSON_HALOW_PEER" > /etc/surf/clock-sync.env

"$project_dir/../network/install-surf-radio-policy.sh" \
    "$SURF_JETSON_WIFI_IF" "$SURF_JETSON_HALOW_IF"
"$project_dir/../halow-telemetry/install-surf-halow-telemetry.sh" \
    "$SURF_TELEMETRY_USER" "$SURF_JETSON_HALOW_IF"
systemctl restart chrony.service
systemctl daemon-reload
systemctl enable --now surf-clock-sync-exporter.service

echo "Jetson headless network commands installed:"
echo "  sudo surf-jetson-internet-mode"
echo "  sudo surf-jetson-test-mode"
echo "  clock-sync-status"
echo "  surf-preflight"
