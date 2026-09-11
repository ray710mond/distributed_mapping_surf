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
SURF_HALOW_PEER=${SURF_HALOW_PEER:-10.43.30.1}

required_settings=(
    SURF_UPLINK_IF
    SURF_NORMAL_WIFI_PROFILE
    SURF_HALOW_IF
    SURF_HALOW_PEER
    SURF_TELEMETRY_USER
)

for setting in "${required_settings[@]}"; do
    if [[ -z ${!setting:-} ]]; then
        echo "$setting is required in $config_file" >&2
        exit 1
    fi
done

required_commands=(
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

if [[ $(readlink -f "$config_file") != /etc/surf/local.env ]]; then
    install -m 0600 "$config_file" /etc/surf/local.env
fi
printf '\nSURF_HALOW_PEER=%q\n' "$SURF_HALOW_PEER" >> /etc/surf/local.env

"$project_dir/../network/remove-surf-wifi-peer-config.sh"

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

"$project_dir/../clock-sync/install-surf-clock-sync.sh" "$SURF_HALOW_PEER"
nmcli connection modify "$SURF_NORMAL_WIFI_PROFILE" \
    connection.autoconnect yes connection.autoconnect-priority 100 \
    802-11-wireless.powersave 2

"$project_dir/../network/install-surf-radio-policy.sh" \
    "$SURF_UPLINK_IF" "$SURF_HALOW_IF"
"$project_dir/../halow-telemetry/install-surf-halow-telemetry.sh" \
    "$SURF_TELEMETRY_USER" "$SURF_HALOW_IF"

/usr/local/sbin/surf-normal-mode
echo "Installed HaLow peer networking and internet Wi-Fi tools."
echo "Use surf-test-mode to check clock readiness without changing Wi-Fi."
