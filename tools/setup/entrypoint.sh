#!/usr/bin/env bash
set -eo pipefail

eval $(fixuid)
source /setup.sh
exec "$@"
