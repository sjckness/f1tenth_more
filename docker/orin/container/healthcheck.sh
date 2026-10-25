#!/bin/bash
# Docker HEALTHCHECK of the car container (docker/orin/Dockerfile, compose.yaml).
#
# Role `car` (F1T_CONTAINER_ROLE, default car): healthy when the discovery
# server's UDP port (11811) is bound and the gateway listens on TCP 8770, BOTH
# by processes of this container. With host networking /proc/net lists the
# host's sockets too, so a port held by the native stack or the native gateway
# must not count: the socket's inode has to be an open fd of a process this
# container sees (its own PID namespace).
# Any other role (the D3 harness): healthy, nothing to check.
#
# F1T_HEALTH_DS_PORT / F1T_HEALTH_GATEWAY_PORT override the ports (a test run
# off the car). Exit 0 healthy, 1 unhealthy (Docker's convention); one line
# on stdout either way.
set -u

role=${F1T_CONTAINER_ROLE:-car}
if [ "$role" != car ]; then
  echo "healthy: role $role, nothing to check"
  exit 0
fi
ds_port=${F1T_HEALTH_DS_PORT:-11811}
gw_port=${F1T_HEALTH_GATEWAY_PORT:-8770}

# Inodes of the sockets bound to local port $2 in the /proc/net tables $1
# (space-separated), in TCP state $3 (0A = LISTEN) or any state if empty.
inodes() {
  local port_hex
  port_hex=$(printf ':%04X' "$2")
  # shellcheck disable=SC2086 # $1 is a list of tables on purpose
  awk -v p="$port_hex" -v st="$3" \
    'FNR > 1 && substr($2, length($2) - 4) == p && (st == "" || $4 == st) { print $10 }' \
    $1 2>/dev/null
}

# Pid of a process of this container that holds socket inode $1, if any.
holder() {
  local fd
  fd=$(find /proc/[0-9]*/fd -maxdepth 1 -lname "socket:\[$1\]" -print -quit 2>/dev/null)
  [ -n "$fd" ] || return 1
  fd=${fd#/proc/}
  echo "${fd%%/*}"
}

# "<pid>" of this container's process bound to the port, or nothing.
held_here() {
  local inode pid
  for inode in $(inodes "$1" "$2" "$3"); do
    if pid=$(holder "$inode"); then
      echo "$pid"
      return 0
    fi
  done
  return 1
}

if ! ds_pid=$(held_here "/proc/net/udp /proc/net/udp6" "$ds_port" ""); then
  if [ -n "$(inodes "/proc/net/udp /proc/net/udp6" "$ds_port" "")" ]; then
    echo "unhealthy: UDP $ds_port is bound, but not by this container (the native stack?)"
  else
    echo "unhealthy: no discovery server on UDP $ds_port"
  fi
  exit 1
fi
if ! gw_pid=$(held_here "/proc/net/tcp /proc/net/tcp6" "$gw_port" 0A); then
  if [ -n "$(inodes "/proc/net/tcp /proc/net/tcp6" "$gw_port" 0A)" ]; then
    echo "unhealthy: TCP $gw_port listens, but not in this container (the native gateway?)"
  else
    echo "unhealthy: the gateway does not listen on TCP $gw_port"
  fi
  exit 1
fi
echo "healthy: discovery server UDP $ds_port (pid $ds_pid), gateway TCP $gw_port (pid $gw_pid)"
exit 0
