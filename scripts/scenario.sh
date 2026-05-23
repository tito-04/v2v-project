#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIOS_DIR="${ROOT_DIR}/scenarios"
UI_HOST_PORT="${UI_HOST_PORT:-18080}"
TOPIC_SAMPLE_RETRIES="${TOPIC_SAMPLE_RETRIES:-6}"

usage() {
  printf 'Usage:\n'
  printf '  %s list\n' "$0"
  printf '  %s run <scenario-name>\n' "$0"
  printf '  %s smoke <scenario-name>\n' "$0"
}

validate_scenario() {
  local name="$1"
  if [[ -z "${name}" || "${name}" == *"/"* || "${name}" == *"\\"* ]]; then
    echo "invalid scenario name: ${name}" >&2
    exit 2
  fi
  if [[ ! -f "${SCENARIOS_DIR}/${name}.json" ]]; then
    echo "unknown scenario: ${name}" >&2
    echo "available scenarios:" >&2
    list_scenarios >&2
    exit 2
  fi
}

list_scenarios() {
  find "${SCENARIOS_DIR}" -maxdepth 1 -type f -name '*.json' -printf '%f\n' \
    | sed 's/\.json$//' \
    | sort
}

sample_topic() {
  local title="$1"
  local container="$2"
  local topic="$3"
  local timeout="$4"
  local retries="${5:-${TOPIC_SAMPLE_RETRIES}}"
  local attempt

  printf '\n== %s ==\n' "${title}"
  for ((attempt = 1; attempt <= retries; attempt++)); do
    if docker exec "${container}" sh -lc "mosquitto_sub -h localhost -t '${topic}' -C 1 -W ${timeout}"; then
      return 0
    fi
    if [[ "${attempt}" -lt "${retries}" ]]; then
      echo "waiting for topic ${topic} (${attempt}/${retries})"
      sleep 1
    fi
  done

  echo "failed to sample topic ${topic} from ${container}" >&2
  return 1
}

run_scenario() {
  local name="$1"
  validate_scenario "${name}"
  cd "${ROOT_DIR}"
  export SCENARIO_NAME="${name}"
  docker compose down --remove-orphans
  docker compose up --build
}

smoke_scenario() {
  local name="$1"
  validate_scenario "${name}"
  cd "${ROOT_DIR}"
  export SCENARIO_NAME="${name}"

  docker compose down --remove-orphans
  docker compose up --build -d

  sleep 2
  docker compose ps

  sample_topic "scenario metadata" "main-broker" "world/scenario" "5"
  sample_topic "main world/pos/lead sample" "main-broker" "world/pos/lead" "5"
  sample_topic "main world/pos/ego sample" "main-broker" "world/pos/ego" "5"
  sample_topic "main world/pos/obstacle sample" "main-broker" "world/pos/obstacle/+" "5"
  sample_topic "lead broker vanetza/in/cam sample" "lead-broker" "vanetza/in/cam" "8"
  sample_topic "ego broker vanetza/out/cam sample" "ego-broker" "vanetza/out/cam" "12" "8"
  sample_topic "main world/tx/cam sample" "main-broker" "world/tx/cam" "5"
  sample_topic "lead broker vanetza/in/cpm sample" "lead-broker" "vanetza/in/cpm" "20" "10"
  sample_topic "ego broker vanetza/out/cpm sample" "ego-broker" "vanetza/out/cpm" "20" "10"
  sample_topic "main world/tx/cpm sample" "main-broker" "world/tx/cpm" "20" "10"

  printf '\n== ego status api ==\n'
  curl -fsS "http://localhost:${UI_HOST_PORT}/api/status"
  printf '\n\nscenario smoke ok: %s\n' "${name}"
}

command="${1:-}"
case "${command}" in
  list)
    list_scenarios
    ;;
  run)
    if [[ $# -ne 2 ]]; then
      usage >&2
      exit 2
    fi
    run_scenario "$2"
    ;;
  smoke)
    if [[ $# -ne 2 ]]; then
      usage >&2
      exit 2
    fi
    smoke_scenario "$2"
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
