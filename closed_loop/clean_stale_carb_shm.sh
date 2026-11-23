#!/usr/bin/env bash
# Remove Carbonite shared memory / named semaphores left by Isaac processes that were killed. Per-process files
# (/dev/shm/carb-RStringInternals-<pid>, sem.carb-RStringInternals-<pid>) are kept only while <pid> is a live Isaac
# (kit python) process; a recycled PID owned by anything else does not count. The global semaphore
# sem.carbonite-sharedmemory is removed only when no Isaac process runs at all. A killed Isaac can die holding these,
# and every later Isaac then waits forever at start-up (hangs of 2026-11-11 / 11-17: 0.08 s CPU, futex wait).
#   bash go2_jepa_pilot/excavator_proto/tools/clean_stale_carb_shm.sh [--loop SECONDS]
is_isaac() { [ -r /proc/$1/cmdline ] && tr '\0' ' ' 2>/dev/null < /proc/$1/cmdline | grep -q "kit/python"; }
any_isaac() { for d in /proc/[0-9]*; do is_isaac "${d#/proc/}" && return 0; done; return 1; }
clean() {
  for f in /dev/shm/carb-RStringInternals-* /dev/shm/sem.carb-RStringInternals-*; do
    [ -e "$f" ] || continue
    is_isaac "${f##*-}" || rm -f "$f"
  done
  if [ -e /dev/shm/sem.carbonite-sharedmemory ] && ! any_isaac; then
    rm -f /dev/shm/sem.carbonite-sharedmemory
  fi
}
if [ "${1:-}" = "--loop" ]; then while true; do clean; sleep "${2:-20}"; done; else clean; fi
