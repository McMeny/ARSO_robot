#!/usr/bin/env bash
# Self-signed TLS certificate for the WebXR app (https://<jetson>:8443) and rosbridge (wss://<jetson>:9090).
# WebXR requires a secure context. Accept the certificate once in the Quest browser:
#   open https://<jetson>:8443  -> "Advanced" -> "Proceed"; then open https://<jetson>:9090 once and accept too.
# Usage: deploy/certs/make_cert.sh [hostname] [extra IP...]
set -euo pipefail
cd "$(dirname "$0")"
HOST="${1:-$(hostname)}"
shift || true
SAN="DNS:${HOST},DNS:${HOST}.local,DNS:localhost,IP:127.0.0.1"
for ip in $(hostname -I 2>/dev/null || true) "$@"; do SAN="${SAN},IP:${ip}"; done
openssl req -x509 -newkey rsa:2048 -nodes -days 825 -keyout eyesteer.key -out eyesteer.pem \
  -subj "/CN=${HOST}/O=eyesteer" -addext "subjectAltName=${SAN}" -addext "extendedKeyUsage=serverAuth" 2>/dev/null
chmod 600 eyesteer.key
echo "[certs] wrote $(pwd)/eyesteer.pem (SAN ${SAN})"
