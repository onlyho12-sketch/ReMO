#!/usr/bin/env bash
# Pi에서 실행: ./run_robot.sh   (Ctrl+C로 정지, 로그는 logs/run_*.csv)
set -e
cd "$(dirname "$0")"
exec python3 -m remo_nav --config config.yaml "$@"
