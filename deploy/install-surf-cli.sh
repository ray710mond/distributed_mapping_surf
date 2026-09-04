#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "Run as root: sudo $0" >&2
    exit 1
fi

repository=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
install -d -m 0755 /etc/surf /usr/local/bin /usr/local/sbin
printf 'SURF_REPOSITORY=%q\n' "$repository" > /etc/surf/repository.env
chmod 0644 /etc/surf/repository.env

install -m 0755 "$repository/tools/validate-config" \
    /usr/local/bin/surf-validate-config

install -m 0755 /dev/stdin /usr/local/bin/surf-drone-compose <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source /etc/surf/repository.env
exec "$SURF_REPOSITORY/deploy/jetson/drone-compose" "$@"
EOF

install -m 0755 /dev/stdin /usr/local/sbin/surf-install-laptop-tools <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source /etc/surf/repository.env
exec "$SURF_REPOSITORY/deploy/local/install-surf-hotspot-service.sh" "$@"
EOF

install -m 0755 /dev/stdin /usr/local/sbin/surf-install-jetson-tools <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source /etc/surf/repository.env
exec "$SURF_REPOSITORY/deploy/jetson/install-surf-jetson-network-tools.sh" "$@"
EOF

install -m 0755 /dev/stdin /usr/local/sbin/surf-install-rd09-driver <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source /etc/surf/repository.env
exec "$SURF_REPOSITORY/deploy/local/install-rd09-driver.sh" "$@"
EOF

echo "Installed:"
echo "  surf-drone-compose"
echo "  surf-validate-config"
echo "  sudo surf-install-laptop-tools"
echo "  sudo surf-install-jetson-tools"
echo "  sudo surf-install-rd09-driver"
