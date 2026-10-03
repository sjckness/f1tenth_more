#!/usr/bin/env bash
# Fix batch 4 (H1): three 30-run batches of phase5_bringup.sh isolation.
#   A baseline: default transports, Discovery Server reused across runs
#     (production behaviour; the server is started fresh once, by run 1);
#   B fresh:    default transports, a fresh Discovery Server for every run;
#   C udp:      FASTDDS_BUILTIN_TRANSPORTS=UDPv4 for the stack, its server
#     (started fresh by run 1, then reused) and every tool.
#   D gap:      as baseline, but 30 s between a shutdown and the next
#     bringup (> the 20 s default participant lease).
#   E lease:    as baseline (2 s gap), but every participant of the stack and
#     the server announces a 5 s lease (profiles/short_lease.xml).
# 30 runs: at the ~1-in-10-runs rate seen so far, P(no event in 30 runs) =
# 0.9^30 = 4 %, so a batch with zero events is evidence (95 %) that the
# condition pushes the rate below 1 in 10.
set -uo pipefail
cd ~/dev_ws/f1tenth_more
OUT=output/fix_batch_4/isolation
N=${N:-30}
H=scripts/jazzy_parity/phase5_bringup.sh
stop_ds() { pkill -INT -x fast-discovery- 2>/dev/null; sleep 2; }
for b in ${BATCHES:-baseline fresh udp}; do
  stop_ds; rm -f /tmp/mission_logger.lock
  case $b in
    baseline) bash $H isolation $OUT/baseline "$N" ;;
    fresh)    FRESH_DS=1 bash $H isolation $OUT/fresh "$N" ;;
    udp)      FASTDDS_BUILTIN_TRANSPORTS=UDPv4 bash $H isolation $OUT/udp "$N" ;;
    gap)      RUN_GAP=30 bash $H isolation $OUT/gap "$N" ;;
    lease)    FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/scripts/fix_batch_4/profiles/short_lease.xml \
                bash $H isolation $OUT/lease "$N" ;;
  esac > $OUT/$b.out 2>&1
done
stop_ds
