#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

./waf --run "freqccv4_4flow --trustedBwSelectionSelfTest=1"
./waf --run "freqccv4_4flow --trustedBwPacingSelfTest=1"
