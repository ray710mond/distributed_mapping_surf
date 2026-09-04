#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run this installer as root: sudo $0" >&2
  exit 1
fi

project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
config_file=${SURF_LOCAL_CONFIG:-$project_dir/.env}
if [[ ! -f $config_file ]]; then
  echo "Create $project_dir/.env from .env.example before installing." >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$config_file"
: "${MORSE_COUNTRY:?MORSE_COUNTRY is required}"
: "${MORSE_BCF_SOURCE:?MORSE_BCF_SOURCE is required}"
MORSE_BCF_BOARDTYPE_ALIASES=${MORSE_BCF_BOARDTYPE_ALIASES:-}
if [[ ! $MORSE_COUNTRY =~ ^[A-Z]{2}$ ]]; then
  echo "MORSE_COUNTRY must be a two-letter uppercase country code." >&2
  exit 1
fi
if [[ ! $MORSE_BCF_SOURCE =~ ^bcf/[A-Za-z0-9._/-]+\.bin$ ]]; then
  echo "MORSE_BCF_SOURCE must name a .bin file below the firmware bcf directory." >&2
  exit 1
fi
morse_bcf_file=${MORSE_BCF_SOURCE##*/}
kernel_release=$(uname -r)
dkms_name=morse
dkms_version=mm8108-2.0.0
dkms_source="/usr/src/$dkms_name-$dkms_version"
driver_url=https://github.com/MorseMicro/morse_driver.git
firmware_url=https://github.com/MorseMicro/morse-firmware.git
driver_ref=mm8108-2.0.0
firmware_ref=mm8108-2.0.0

source_root=$(mktemp -d -t surf-morse-source.XXXXXXXX)
trap 'rm -rf -- "$source_root"' EXIT
driver_dir="$source_root/morse_driver"
firmware_dir="$source_root/morse-firmware"

git clone --quiet --depth 1 --branch "$driver_ref" --recurse-submodules \
  --shallow-submodules "$driver_url" "$driver_dir"
git clone --quiet --depth 1 --branch "$firmware_ref" \
  "$firmware_url" "$firmware_dir"

for source_file in \
  "$firmware_dir/firmware/mm8108b2-rl.bin" \
  "$firmware_dir/$MORSE_BCF_SOURCE" \
  "$project_dir/morse-dkms.conf"; do
  if [[ ! -f $source_file ]]; then
    echo "Required build artifact is missing: $source_file" >&2
    exit 1
  fi
done

install -d /lib/firmware/morse
install -m 0644 "$firmware_dir/firmware/mm8108b2-rl.bin" /lib/firmware/morse/
install -m 0644 "$firmware_dir/$MORSE_BCF_SOURCE" "/lib/firmware/morse/$morse_bcf_file"
for boardtype in $MORSE_BCF_BOARDTYPE_ALIASES; do
  if [[ ! $boardtype =~ ^[0-9A-Fa-f]{4}$ ]]; then
    echo "Invalid board-type alias: $boardtype" >&2
    exit 1
  fi
  ln -sfn "$morse_bcf_file" "/lib/firmware/morse/bcf_boardtype_${boardtype,,}.bin"
done

install -d "$dkms_source"
cp -a "$driver_dir/." "$dkms_source/"
sed "s/@MORSE_COUNTRY@/$MORSE_COUNTRY/g" \
  "$project_dir/morse-dkms.conf" > "$dkms_source/dkms.conf"

if ! dkms status -m "$dkms_name" -v "$dkms_version" | grep -q .; then
  dkms add -m "$dkms_name" -v "$dkms_version"
fi
dkms build -m "$dkms_name" -v "$dkms_version" -k "$kernel_release"
dkms install --force -m "$dkms_name" -v "$dkms_version" -k "$kernel_release"

# Retain the original manually signed modules as recoverable, non-loadable backups.
manual_module_dir="/lib/modules/$kernel_release/updates/morse"
for module_file in "$manual_module_dir/dot11ah.ko" "$manual_module_dir/morse.ko"; do
  if [[ -f $module_file ]]; then
    mv "$module_file" "$module_file.manual-backup"
  fi
done

printf '%s\n' dot11ah morse > /etc/modules-load.d/morse-rd09.conf
printf 'options morse country=%s bcf=%s\n' "$MORSE_COUNTRY" "$morse_bcf_file" \
  > /etc/modprobe.d/morse-rd09.conf

depmod -a "$kernel_release"
modprobe dot11ah
modprobe morse

echo "RD09 firmware and signed kernel modules installed for $kernel_release."
