#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "Run this installer as root: sudo $0" >&2
    exit 1
fi

project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
config_file=${SURF_LOCAL_CONFIG:-$project_dir/.env}

if [[ ! -f "$config_file" ]]; then
    echo "Create $project_dir/.env from .env.example before installing." >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$config_file"

required_settings=(
    SURF_UPLINK_IF
    SURF_AP_IF
    SURF_WIFI5_SSID
    SURF_WIFI5_PASSPHRASE
    SURF_WIFI5_CHANNEL
    SURF_WIFI5_AP_ADDR
    SURF_WIFI5_AP_PREFIX
    SURF_WIFI5_NETMASK
    SURF_WIFI5_AP_NET
    SURF_WIFI5_PEER_ADDR
    SURF_WIFI5_DHCP_START
    SURF_WIFI5_DHCP_END
    SURF_COUNTRY
    SURF_NORMAL_WIFI_PROFILE
    SURF_HALOW_IF
    SURF_HALOW_SUBNET
    SURF_TELEMETRY_USER
)

for setting in "${required_settings[@]}"; do
    if [[ -z ${!setting:-} ]]; then
        echo "$setting is required in $config_file" >&2
        exit 1
    fi
done

required_commands=(
    hostapd
    dnsmasq
    iw
    nmcli
    systemctl
    chronyc
)

for command_name in "${required_commands[@]}"; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "Required command not found: $command_name" >&2
        exit 1
    fi
done

install -d -m 0755 /etc/surf

install -m 0600 \
    "$config_file" \
    /etc/surf/local.env

install -m 0755 \
    "$project_dir/start-surf-hotspot.sh" \
    /usr/local/sbin/start-surf-hotspot

install -m 0755 \
    "$project_dir/stop-surf-hotspot.sh" \
    /usr/local/sbin/stop-surf-hotspot

install -m 0755 \
    "$project_dir/surf-test-mode" \
    /usr/local/sbin/surf-test-mode

install -m 0755 \
    "$project_dir/surf-normal-mode" \
    /usr/local/sbin/surf-normal-mode

install -m 0755 \
    "$project_dir/surf-collate-run" \
    /usr/local/bin/surf-collate-run

install -m 0755 \
    "$project_dir/../../surf_data_tracker/surf_data_tracker/clock.py" \
    /usr/local/bin/clock-sync-status
install -m 0755 "$project_dir/../network/surf-preflight" /usr/local/bin/surf-preflight

# Make the laptop a usable offline time authority and permit only the dedicated
# HaLow subnet. `local` keeps serving a stable common clock without Internet.
install -d -m 0755 /etc/chrony/conf.d
printf 'allow %s\nlocal stratum 10 orphan\n' "$SURF_HALOW_SUBNET" \
    > /etc/chrony/conf.d/surf-halow-server.conf
systemctl restart chrony.service

nmcli connection modify "$SURF_NORMAL_WIFI_PROFILE" 802-11-wireless.powersave 2

"$project_dir/../network/install-surf-radio-policy.sh" \
    "$SURF_UPLINK_IF" "$SURF_HALOW_IF"
"$project_dir/../halow-telemetry/install-surf-halow-telemetry.sh" \
    "$SURF_TELEMETRY_USER" "$SURF_HALOW_IF"

install -m 0644 \
    "$project_dir/surf-robot-hotspot.service" \
    /etc/systemd/system/surf-robot-hotspot.service

systemctl daemon-reload

# The hotspot is intentionally manual.
#
# Installation must not alter the user's current Wi-Fi connection and must
# not cause the robot hotspot to start automatically at boot.
systemctl disable surf-robot-hotspot.service >/dev/null 2>&1 || true

echo
echo "SURF Wi-Fi mode tools installed."
echo
echo "The robot hotspot is NOT enabled at boot."
echo
echo "Commands:"
echo "  sudo surf-test-mode"
echo "      Disconnect normal Wi-Fi and start SURF-Robot-Net."
echo
echo "  sudo surf-normal-mode"
echo "      Stop SURF-Robot-Net and reconnect normal Wi-Fi."
echo "  surf-collate-run <tracker_run_id>"
echo "      Collect both hosts and regenerate analysis outputs."
echo "  clock-sync-status"
echo "      Print verified local/offline clock status as JSON."
echo "  surf-preflight"
echo "      Validate persistent radio, telemetry, and clock prerequisites."
