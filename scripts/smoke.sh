#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UI_HOST_PORT="${UI_HOST_PORT:-18080}"
UI_STATUS_URL="${UI_STATUS_URL:-http://localhost:${UI_HOST_PORT}/api/status}"
V2V_CONTAINER="${V2V_CONTAINER:-lead-vanetza}"
V2V_IFACE="${V2V_IFACE:-eth0}"
TOPIC_SAMPLE_RETRIES="${TOPIC_SAMPLE_RETRIES:-5}"

sample_topic() {
  local title="$1"
  local container="$2"
  local topic="$3"
  local timeout="$4"
  local retries="${5:-${TOPIC_SAMPLE_RETRIES}}"
  local attempt

  echo "\n== ${title} =="
  for ((attempt = 1; attempt <= retries; attempt++)); do
    if docker exec "${container}" sh -lc "mosquitto_sub -h localhost -t '${topic}' -C 1 -W ${timeout}"; then
      return 0
    fi

    if [[ "${attempt}" -lt "${retries}" ]]; then
      echo "waiting for topic ${topic} (${attempt}/${retries})"
      sleep 1
    fi
  done

  echo "failed to sample topic ${topic} from ${container}"
  return 1
}

api_status() {
  local title="$1"
  local attempt

  echo "\n== ${title} =="
  for ((attempt = 1; attempt <= TOPIC_SAMPLE_RETRIES; attempt++)); do
    if curl -fsS "${UI_STATUS_URL}"; then
      echo
      return 0
    fi

    if [[ "${attempt}" -lt "${TOPIC_SAMPLE_RETRIES}" ]]; then
      echo "retrying status api (${attempt}/${TOPIC_SAMPLE_RETRIES})"
      sleep 1
    fi
  done

  echo "failed to fetch status api"
  return 1
}

wait_for_cam_tick() {
  local attempt

  for ((attempt = 1; attempt <= TOPIC_SAMPLE_RETRIES; attempt++)); do
    if docker exec ego-broker sh -lc "mosquitto_sub -h localhost -t vanetza/out/cam -C 1 -W 8 >/dev/null"; then
      return 0
    fi
    sleep 1
  done

  echo "failed waiting for CAM tick on ego-broker"
  return 1
}

sample_ego_cam_out_with_recovery() {
  if sample_topic "ego broker vanetza/out/cam sample" "ego-broker" "vanetza/out/cam" "10" "8"; then
    return 0
  fi

  echo "\n== cam out recovery: restart vanetza nodes =="
  docker compose restart lead-vanetza ego-vanetza >/dev/null
  sleep 3

  sample_topic "ego broker vanetza/out/cam sample (after recovery)" "ego-broker" "vanetza/out/cam" "12" "10"
}

echo "== preflight =="
"${ROOT_DIR}/scripts/preflight.sh"

echo "\n== compose down (clean state) =="
cd "${ROOT_DIR}"
docker compose down --remove-orphans >/dev/null

echo "\n== compose up =="
docker compose up --build -d

echo "\n== startup settle =="
sleep 2

echo "\n== compose ps =="
docker compose ps

sample_topic "main world/pos/lead sample" "main-broker" "world/pos/lead" "5"
sample_topic "main world/pos/ego sample" "main-broker" "world/pos/ego" "5"
sample_topic "lead broker vanetza/in/cam sample" "lead-broker" "vanetza/in/cam" "6"
sample_ego_cam_out_with_recovery

api_status "ego status api baseline"

echo "\n== netem mild =="
"${ROOT_DIR}/scripts/netem_profiles.sh" apply mild "${V2V_CONTAINER}" "${V2V_IFACE}"
wait_for_cam_tick
api_status "ego status api after mild"

echo "\n== netem severe =="
"${ROOT_DIR}/scripts/netem_profiles.sh" apply severe "${V2V_CONTAINER}" "${V2V_IFACE}"
wait_for_cam_tick
api_status "ego status api after severe"

echo "\n== netem recovery =="
"${ROOT_DIR}/scripts/netem_profiles.sh" clear "${V2V_CONTAINER}" "${V2V_IFACE}"
wait_for_cam_tick
api_status "ego status api after recovery"

echo "\n== qdisc state =="
docker exec "${V2V_CONTAINER}" tc qdisc show dev "${V2V_IFACE}"

echo "\nsmoke ok"
