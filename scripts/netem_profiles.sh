#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 apply <baseline|mild|severe|blackout> <container> <interface>"
  echo "       $0 clear <container> <interface>"
  echo "       $0 <baseline|mild|severe|blackout> [container] [interface]"
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

action="$1"
default_container="${V2V_CONTAINER:-lead-vanetza}"
default_iface="${V2V_IFACE:-eth0}"

apply_profile() {
  local profile="$1"
  local container="$2"
  local iface="$3"

  case "${profile}" in
    baseline)
      docker exec "${container}" tc qdisc del dev "${iface}" root 2>/dev/null || true
      ;;
    mild)
      docker exec "${container}" tc qdisc replace dev "${iface}" root netem delay 300ms 50ms loss 5%
      ;;
    severe)
      docker exec "${container}" tc qdisc replace dev "${iface}" root netem delay 800ms 200ms loss 25%
      ;;
    blackout)
      docker exec "${container}" tc qdisc replace dev "${iface}" root netem loss 100%
      ;;
    *)
      echo "unknown profile: ${profile}"
      exit 1
      ;;
  esac

  echo "applied profile=${profile} container=${container} iface=${iface}"
}

clear_profile() {
  local container="$1"
  local iface="$2"
  docker exec "${container}" tc qdisc del dev "${iface}" root 2>/dev/null || true
  echo "cleared profile container=${container} iface=${iface}"
}

case "${action}" in
  baseline|mild|severe|blackout)
    apply_profile "${action}" "${2:-${default_container}}" "${3:-${default_iface}}"
    ;;
  apply)
    if [[ $# -ne 4 ]]; then
      usage
      exit 1
    fi
    apply_profile "$2" "$3" "$4"
    ;;
  clear)
    if [[ $# -ne 3 ]]; then
      usage
      exit 1
    fi
    clear_profile "$2" "$3"
    ;;
  *)
    usage
    exit 1
    ;;
esac
