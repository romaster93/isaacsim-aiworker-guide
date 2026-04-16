#!/usr/bin/env bash
# Launch the 4 VLM servers (YOLOv7, GroundingDINO, BLIP2-ITM, MobileSAM) in tmux.
#
# Usage:
#   scripts/start-vlm-servers.sh           # Start 4 servers in a new tmux session
#   scripts/start-vlm-servers.sh --restart # Kill existing VLM processes first, then start
#   scripts/start-vlm-servers.sh --attach  # Start if needed, then attach to tmux session
#
# Each server runs in its own tmux window. Session name: "vlm_servers".
# Conda env required: apexnav_ros2.

set -e

SESSION="vlm_servers"
APEXNAV_DIR="${APEXNAV_DIR:-$HOME/ApexNav_ROS2_wrapper}"
CONDA_ENV="${CONDA_ENV:-apexnav_ros2}"

declare -A SERVERS=(
  [0]="yolov7:vlm.detector.yolov7:12184"
  [1]="grounding_dino:vlm.detector.grounding_dino:12181"
  [2]="blip2itm:vlm.itm.blip2itm:12182"
  [3]="mobilesam:vlm.segmentor.sam:12183"
)

restart=false
attach=false
for arg in "$@"; do
  case "$arg" in
    --restart) restart=true ;;
    --attach) attach=true ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
    *) echo "Unknown arg: $arg" >&2; exit 2 ;;
  esac
done

if $restart; then
  echo "[restart] Killing existing VLM server processes..."
  pkill -9 -f "vlm\.(detector|itm|segmentor)" 2>/dev/null || true
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  sleep 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[info] tmux session '$SESSION' already exists."
  $attach && exec tmux attach -t "$SESSION"
  exit 0
fi

if ! command -v tmux >/dev/null; then
  echo "[error] tmux not installed. Run: sudo apt install tmux" >&2
  exit 1
fi

if [[ ! -d "$APEXNAV_DIR" ]]; then
  echo "[error] ApexNav_ROS2_wrapper not found at: $APEXNAV_DIR" >&2
  exit 1
fi

CONDA_INIT="source $HOME/miniconda3/etc/profile.d/conda.sh && conda activate $CONDA_ENV"

tmux new-session -d -s "$SESSION" -n "${SERVERS[0]%%:*}"
for i in 0 1 2 3; do
  entry="${SERVERS[$i]}"
  name="${entry%%:*}"
  mod="${entry#*:}"; mod="${mod%%:*}"
  port="${entry##*:}"
  if [[ $i -gt 0 ]]; then
    tmux new-window -t "$SESSION:$i" -n "$name"
  fi
  tmux send-keys -t "$SESSION:$i" \
    "$CONDA_INIT && cd '$APEXNAV_DIR' && echo '[*] starting $name on port $port' && python -m $mod" Enter
done

echo "[ok] Started tmux session '$SESSION' with 4 VLM servers."
echo "     Attach: tmux attach -t $SESSION"
echo "     Ports: 12184 (yolo), 12181 (gdino), 12182 (blip2), 12183 (sam)"
echo "     Give ~30s for all models to load, then verify with:"
echo "       for p in 12184 12181 12182 12183; do curl -s -o /dev/null -w \"port \$p: %{http_code}\\n\" http://localhost:\$p/; done"

$attach && exec tmux attach -t "$SESSION"
