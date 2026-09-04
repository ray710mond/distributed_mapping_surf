#!/usr/bin/env bash
set -Eeuo pipefail

CONFIG_FILE=${SURF_LOCAL_CONFIG:-/etc/surf/local.env}

if [[ ! -r "$CONFIG_FILE" ]]; then
    echo "Local deployment configuration is not readable: $CONFIG_FILE" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$CONFIG_FILE"

RADIO_IF=${SURF_UPLINK_IF:?SURF_UPLINK_IF is required}
AP_IF=${SURF_AP_IF:?SURF_AP_IF is required}
SSID=${SURF_WIFI5_SSID:?SURF_WIFI5_SSID is required}
PASSPHRASE=${SURF_WIFI5_PASSPHRASE:?SURF_WIFI5_PASSPHRASE is required}
CHANNEL=${SURF_WIFI5_CHANNEL:?SURF_WIFI5_CHANNEL is required}
AP_ADDR=${SURF_WIFI5_AP_ADDR:?SURF_WIFI5_AP_ADDR is required}
AP_PREFIX=${SURF_WIFI5_AP_PREFIX:?SURF_WIFI5_AP_PREFIX is required}
DHCP_START=${SURF_WIFI5_DHCP_START:?SURF_WIFI5_DHCP_START is required}
DHCP_END=${SURF_WIFI5_DHCP_END:?SURF_WIFI5_DHCP_END is required}
NETMASK=${SURF_WIFI5_NETMASK:?SURF_WIFI5_NETMASK is required}
COUNTRY=${SURF_COUNTRY:?SURF_COUNTRY is required}

HOSTAPD_CONF=/run/surf-robot-hostapd.conf
HOSTAPD_PID=/run/surf-robot-hostapd.pid
DNSMASQ_CONF=/run/surf-robot-dnsmasq.conf
DNSMASQ_PID=/run/surf-robot-dnsmasq.pid
NM_UNMANAGED_CONF=/run/NetworkManager/conf.d/90-surf-hotspot-unmanaged.conf

if [[ ${EUID} -ne 0 ]]; then
    echo "Run this script with sudo." >&2
    exit 1
fi

cleanup_failed_start() {
    local status=${1:-1}

    set +e

    if [[ -f "$DNSMASQ_PID" ]]; then
        kill "$(cat "$DNSMASQ_PID")" 2>/dev/null
    fi

    if [[ -f "$HOSTAPD_PID" ]]; then
        kill "$(cat "$HOSTAPD_PID")" 2>/dev/null
    fi

    ip link set "$AP_IF" down 2>/dev/null
    iw dev "$AP_IF" del 2>/dev/null

    rm -f \
        "$HOSTAPD_PID" \
        "$DNSMASQ_PID" \
        "$HOSTAPD_CONF" \
        "$DNSMASQ_CONF" \
        "$NM_UNMANAGED_CONF"

    nmcli general reload conf 2>/dev/null || true

    return "$status"
}

trap 'status=$?; cleanup_failed_start "$status"; exit "$status"' ERR

echo "Disconnecting normal Wi-Fi from $RADIO_IF..."

# Intel radios can come up after a reboot with every 5 GHz channel marked
# NO-IR until the regulatory domain has been set and the radio has heard the
# local access points.  In that state hostapd is forbidden from initiating an
# AP, even though the same channel worked before the reboot.  Set the requested
# domain and scan while NetworkManager still owns the station interface.
if ! iw reg set "$COUNTRY"; then
    echo "Could not set the Wi-Fi regulatory country to $COUNTRY." >&2
    exit 1
fi

nmcli radio wifi on
nmcli device set "$RADIO_IF" managed yes
ip link set "$RADIO_IF" up

if ! nmcli device wifi rescan ifname "$RADIO_IF" >/dev/null 2>&1; then
    echo "Warning: regulatory Wi-Fi scan on $RADIO_IF failed; continuing." >&2
else
    # Regulatory information is applied asynchronously by the driver.
    sleep 2
fi

# Important:
# Keep the physical Wi-Fi interface managed by NetworkManager.
# `nmcli device disconnect` prevents NetworkManager from immediately
# auto-connecting it while the same radio is being used for the robot AP.
nmcli device set "$RADIO_IF" managed yes
nmcli device disconnect "$RADIO_IF" >/dev/null 2>&1 || true

sleep 1

# Clean up remnants from an earlier hotspot run.
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
    "$DNSMASQ_CONF"

# NetworkManager should ignore only the virtual AP interface.
# Do NOT mark the physical Intel interface unmanaged.
mkdir -p "$(dirname "$NM_UNMANAGED_CONF")"

cat >"$NM_UNMANAGED_CONF" <<EOF
[keyfile]
unmanaged-devices=interface-name:$AP_IF
EOF

nmcli general reload conf

# Create the AP interface on the same physical Intel radio.
iw dev "$RADIO_IF" interface add "$AP_IF" type __ap

nmcli device set "$AP_IF" managed no 2>/dev/null || true

ip address flush dev "$AP_IF"
ip address add "$AP_ADDR/$AP_PREFIX" dev "$AP_IF"
ip link set "$AP_IF" up
/usr/local/sbin/surf-radio-policy "$AP_IF" 2>/dev/null || true

cat >"$HOSTAPD_CONF" <<EOF
interface=$AP_IF
driver=nl80211
ssid=$SSID
country_code=$COUNTRY
ieee80211d=1
hw_mode=a
channel=$CHANNEL
wmm_enabled=1
ieee80211n=1
ieee80211ac=1
auth_algs=1
wpa=2
wpa_passphrase=$PASSPHRASE
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
EOF

chmod 600 "$HOSTAPD_CONF"

cat >"$DNSMASQ_CONF" <<EOF
interface=$AP_IF
bind-dynamic
dhcp-range=$DHCP_START,$DHCP_END,$NETMASK,12h
dhcp-option=3,$AP_ADDR
pid-file=$DNSMASQ_PID
leasefile-ro
EOF

if ! hostapd -B -P "$HOSTAPD_PID" "$HOSTAPD_CONF"; then
    echo "hostapd could not start on channel $CHANNEL." >&2
    echo "Check whether the kernel forbids AP operation there:" >&2
    echo "  sudo iw phy phy\$(iw dev $RADIO_IF info | awk '/wiphy/ {print \$2}') channels" >&2
    echo "If $CHANNEL is marked 'no IR', verify SURF_COUNTRY and retry after a Wi-Fi scan." >&2
    exit 1
fi

sleep 2

if [[ ! -s "$HOSTAPD_PID" ]] ||
   ! kill -0 "$(cat "$HOSTAPD_PID")" 2>/dev/null; then
    echo "hostapd exited before the hotspot became ready on channel $CHANNEL." >&2
    echo "A channel marked 'no IR' cannot be used for an access point; verify SURF_COUNTRY." >&2
    exit 1
fi

if ! iw dev "$AP_IF" info | grep -q 'type AP'; then
    echo "$AP_IF was created but is not operating as an AP." >&2
    exit 1
fi

if ! ip -4 address show dev "$AP_IF" |
    grep -q "$AP_ADDR/$AP_PREFIX"; then
    echo "$AP_IF does not have the expected address." >&2
    exit 1
fi

dnsmasq --conf-file="$DNSMASQ_CONF"

trap - ERR

echo
echo "$SSID is active."
echo "Interface: $AP_IF"
echo "Channel:   $CHANNEL"
echo "Laptop IP: $AP_ADDR"
echo "Internet:  not provided"
