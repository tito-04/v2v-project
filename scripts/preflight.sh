#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ROOT_DIR}/.env"

fail() {
  echo "preflight failed: $1"
  exit 1
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    fail "${cmd} not found"
  fi
}

require_cmd docker
require_cmd jq

if ! docker compose version >/dev/null 2>&1; then
  fail "docker compose not available"
fi

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "missing .env file, creating from .env.example"
  cp "${ROOT_DIR}/.env.example" "${ENV_FILE}"
fi

pushd "${ROOT_DIR}" >/dev/null

compose_json_file="$(mktemp)"
trap 'rm -f "${compose_json_file}"' EXIT

docker compose config --format json >"${compose_json_file}"

lead_id="$(jq -r '.services["lead-vanetza"].environment.VANETZA_STATION_ID // empty' "${compose_json_file}")"
ego_id="$(jq -r '.services["ego-vanetza"].environment.VANETZA_STATION_ID // empty' "${compose_json_file}")"

if [[ -z "${lead_id}" || -z "${ego_id}" ]]; then
  fail "station ids missing in compose runtime config"
fi

if [[ "${lead_id}" == "${ego_id}" ]]; then
  fail "station ids must be unique"
fi

lead_mac="$(jq -r '.services["lead-vanetza"].environment.VANETZA_MAC_ADDRESS // empty' "${compose_json_file}")"
ego_mac="$(jq -r '.services["ego-vanetza"].environment.VANETZA_MAC_ADDRESS // empty' "${compose_json_file}")"
mac_regex='^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$'

if [[ -z "${lead_mac}" || -z "${ego_mac}" ]]; then
  fail "vanetza mac addresses missing in compose runtime config"
fi

if [[ ! "${lead_mac}" =~ ${mac_regex} || ! "${ego_mac}" =~ ${mac_regex} ]]; then
  fail "invalid mac format in VANETZA_MAC_ADDRESS"
fi

if [[ "${lead_mac,,}" == "${ego_mac,,}" ]]; then
  fail "vanetza mac addresses must be unique"
fi

lead_iface="$(jq -r '.services["lead-vanetza"].environment.VANETZA_INTERFACE // empty' "${compose_json_file}")"
ego_iface="$(jq -r '.services["ego-vanetza"].environment.VANETZA_INTERFACE // empty' "${compose_json_file}")"

if [[ -z "${lead_iface}" || -z "${ego_iface}" ]]; then
  fail "vanetza interface names missing in compose runtime config"
fi

main_broker_host="$(jq -r '.services["world-generator"].environment.MAIN_BROKER_HOST // empty' "${compose_json_file}")"
main_broker_port="$(jq -r '.services["world-generator"].environment.MAIN_BROKER_PORT // empty' "${compose_json_file}")"
lead_broker_host="$(jq -r '.services["lead-vanetza"].environment.VANETZA_LOCAL_MQTT_BROKER // empty' "${compose_json_file}")"
lead_broker_port="$(jq -r '.services["lead-vanetza"].environment.VANETZA_LOCAL_MQTT_PORT // empty' "${compose_json_file}")"
ego_broker_host="$(jq -r '.services["ego-vanetza"].environment.VANETZA_LOCAL_MQTT_BROKER // empty' "${compose_json_file}")"
ego_broker_port="$(jq -r '.services["ego-vanetza"].environment.VANETZA_LOCAL_MQTT_PORT // empty' "${compose_json_file}")"

for endpoint in \
  "main:${main_broker_host}:${main_broker_port}" \
  "lead:${lead_broker_host}:${lead_broker_port}" \
  "ego:${ego_broker_host}:${ego_broker_port}"; do
  IFS=':' read -r name host port <<<"${endpoint}"
  if [[ -z "${host}" || -z "${port}" ]]; then
    fail "${name} broker host/port missing in compose runtime config"
  fi
  if [[ ! "${port}" =~ ^[0-9]+$ ]]; then
    fail "${name} broker port is not numeric: ${port}"
  fi
done

mgmt_network_name="$(jq -r '.networks.mgmt_network.name // empty' "${compose_json_file}")"
if [[ -z "${mgmt_network_name}" ]]; then
  fail "mgmt_network name not resolved from compose config"
fi

docker compose up -d --wait main-broker lead-broker ego-broker lead-vanetza ego-vanetza >/dev/null

for broker_container in main-broker lead-broker ego-broker; do
  health="$(docker inspect --format '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "${broker_container}")"
  if [[ "${health}" != "healthy" ]]; then
    fail "${broker_container} is not healthy"
  fi
done

if ! docker exec lead-vanetza sh -lc "ip link show '${lead_iface}' >/dev/null 2>&1"; then
  fail "interface ${lead_iface} not found in lead-vanetza"
fi

if ! docker exec ego-vanetza sh -lc "ip link show '${ego_iface}' >/dev/null 2>&1"; then
  fail "interface ${ego_iface} not found in ego-vanetza"
fi

for endpoint in \
  "main:${main_broker_host}:${main_broker_port}" \
  "lead:${lead_broker_host}:${lead_broker_port}" \
  "ego:${ego_broker_host}:${ego_broker_port}"; do
  IFS=':' read -r name host port <<<"${endpoint}"
  if ! docker run --rm --network "${mgmt_network_name}" eclipse-mosquitto:2 sh -lc "mosquitto_pub -h '${host}' -p '${port}' -t 'health/preflight/${name}' -m ok >/dev/null 2>&1"; then
    fail "unable to reach ${name} broker at ${host}:${port} from mgmt network"
  fi
done

echo "preflight ok"

popd >/dev/null
