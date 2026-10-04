#!/usr/bin/env bash
# Fix batch 5 (H1): the lease fix's live measurements, through
# scripts/jazzy_parity/phase5_bringup.sh. BATCHES picks which, in order:
#
#   proof     60 isolation bringups, shipped profile (the launch default),
#             watchdog in alert mode (it reports, never restarts, so it cannot
#             hide an isolation from isolation_check.py). Same conditions as
#             fix batch 4's baseline otherwise: Discovery Server reused across
#             runs (started fresh, with the profile, by run 1), 2 s gap.
#   stress    20 of the same with `stress --cpu 12` running (14-core Thor:
#             the CPU saturated, standing in for the Orin's 78-100 % load):
#             does the 5 s lease drop live participants under load?
#   dl_before 10 discload runs, Fast DDS's default lease written out
#             (profiles/default_lease.xml), watchdog disabled;
#   dl_after  10 discload runs, shipped profile, watchdog disabled;
#   dl_wd     5 discload runs, shipped profile, watchdog enforce (its CPU).
#
# The harness's own tools (probes, isolation_check) get the same profile as
# the stack of that batch through FASTRTPS_DEFAULT_PROFILES_FILE.
set -uo pipefail
cd ~/dev_ws/f1tenth_more
OUT=output/fix_batch_5
H=scripts/jazzy_parity/phase5_bringup.sh
SHIPPED=$PWD/src/f1tenth_bringup/config/fastdds_profile.xml
DEFAULT=$PWD/scripts/fix_batch_5/profiles/default_lease.xml
stop_ds() { pkill -INT -x fast-discovery- 2>/dev/null; sleep 2; }
for b in ${BATCHES:-proof stress dl_before dl_after dl_wd}; do
  stop_ds; rm -f /tmp/mission_logger.lock
  mkdir -p $OUT/isolation $OUT/discload
  echo "$(date -Is) start $b" >> $OUT/run_batches.log
  case $b in
    proof)
      FASTRTPS_DEFAULT_PROFILES_FILE=$SHIPPED LAUNCH_ARGS="health_watchdog:=alert" \
        bash $H isolation $OUT/isolation/proof "${N_PROOF:-60}" > $OUT/isolation/proof.out 2>&1 ;;
    stress)
      stress --cpu 12 > /dev/null 2>&1 & SP=$!
      FASTRTPS_DEFAULT_PROFILES_FILE=$SHIPPED LAUNCH_ARGS="health_watchdog:=alert" \
        bash $H isolation $OUT/isolation/stress "${N_STRESS:-20}" > $OUT/isolation/stress.out 2>&1
      kill $SP; pkill -x stress ;;
    dl_before)
      FASTRTPS_DEFAULT_PROFILES_FILE=$DEFAULT LAUNCH_ARGS="fastdds_profile:=$DEFAULT health_watchdog:=disabled" \
        bash $H discload $OUT/discload/before 10 > $OUT/discload/before.out 2>&1 ;;
    dl_after)
      FASTRTPS_DEFAULT_PROFILES_FILE=$SHIPPED LAUNCH_ARGS="health_watchdog:=disabled" \
        bash $H discload $OUT/discload/after 10 > $OUT/discload/after.out 2>&1 ;;
    dl_wd)
      FASTRTPS_DEFAULT_PROFILES_FILE=$SHIPPED LAUNCH_ARGS="health_watchdog:=enforce" \
        bash $H discload $OUT/discload/after_watchdog 5 > $OUT/discload/after_watchdog.out 2>&1 ;;
  esac
  echo "$(date -Is) end $b" >> $OUT/run_batches.log
done
stop_ds
