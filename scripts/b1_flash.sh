#!/usr/bin/env bash
# B1 flash sihirbazı: docs/bringup-checklist.md'yi adım adım yürütür.
#   ./scripts/b1_flash.sh                 # gerçek tezgah (dev container'da)
#   ./scripts/b1_flash.sh --prova         # sahte donanımla prova, soruları sen cevaplarsın
#   ./scripts/b1_flash.sh --prova --otomatik
#   ./scripts/b1_flash.sh --liste | --adim <id> | --sifirla | --fw-dir <dizin>
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHONPATH="${REPO}/tools/b1_flash${PYTHONPATH:+:${PYTHONPATH}}" exec python3 -m b1_flash "$@"
