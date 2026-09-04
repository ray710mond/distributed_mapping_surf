#!/usr/bin/env bash
set -u

CONFIG_FILE=${SURF_LOCAL_CONFIG:-/etc/surf/local.env}

if [[ ! -r "$CONFIG_FILE" ]]; then
    echo "Local deployment configuration is not readable: $CONFIG_FILE" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$CONFIG_FILE"

AP_IF=${SURF_AP_IF:?SURF_AP_IF is required}

HOSTAPD_PID=/run/surf-robot-hostapd.pid
DNSMASQ_PID=/run/surf-robot-dnsmasq.pid
HOSTAPD_CONF=/run/surf-robot-hostapd.conf
DNSMASQ_CONF=/run/surf-robot-dnsmasq.conf
NM_UNMANAGED_CONF=/run/NetworkManager/conf.d/90-surf-hotspot-unmanaged.conf

if [[ ${EUID} -ne 0 ]]; then
    echo "Run this script with sudo." >&2
    exit 1
fi

if [[ -f "$DNSMASQ_PID" ]]; then
    kill "$(cat "$DNSMASQ_PID")" 2>/dev/null || true
fi

if [[ -f "$HOSTAPD_PID" ]]; then
    kill "$(cat "$HOSTAPD_PID")" 2>/dev/null || true
fi

ip link set "$AP_IF" down 2>/dev/null || true
iw dev "$AP_IF" del 2>/dev/null || true

rm -f \
    "$HOSTAPD_PID" \
    "$DNSMASQ_PID" \
    "$HOSTAPD_CONF" \
    "$DNSMASQ_CONF" \
    "$NM_UNMANAGED_CONF"

nmcli general reload conf 2>/dev/null || true

echo "SURF robot hotspot stopped."