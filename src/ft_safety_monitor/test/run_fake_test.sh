#!/usr/bin/env bash
# Fake-execution test for ft_safety_monitor.
#
# This script never starts aubo_driver, Gazebo, or anything that can move a
# real robot.  It launches MoveIt with fake_execution:=true, a fake wrench
# publisher, a mock action server, and the monitor.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${HOME}/aubo_ws"
LOG="$(mktemp /tmp/ft_safety_test.XXXXXX.log)"
EVENT="$(mktemp /tmp/ft_safety_event.XXXXXX.txt)"
GOAL_LOG="$(mktemp /tmp/ft_safety_goal.XXXXXX.log)"
MOCK_EVENTS="/tmp/ft_safety_mock_events.txt"
GOAL_PID=""
FAILS=0
: >"${MOCK_EVENTS}"

source /opt/ros/noetic/setup.bash
source "${WS}/devel/setup.bash"

cleanup() {
  if [[ -n "${GOAL_PID}" ]]; then
    kill "${GOAL_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*"; FAILS=$((FAILS + 1)); }

state() {
  timeout 8 python3 "${HERE}/get_state.py" 2>/dev/null | tail -n1
}

wait_state() {
  local expected="$1" seconds="$2"
  local deadline=$((SECONDS + seconds))
  local now=""
  while (( SECONDS < deadline )); do
    now="$(state)"
    if [[ "${now}" == "${expected}" ]]; then
      return 0
    fi
    sleep 0.2
  done
  echo "  last state: '${now}' (expected '${expected}')"
  return 1
}

command_mode() {
  rostopic pub -1 /fake_wrench/command std_msgs/String "data: '$1'" >/dev/null 2>&1
}

echo "Launching fake-execution test stack (log: ${LOG})"
roslaunch ft_safety_monitor ft_safety_test.launch >"${LOG}" 2>&1 &
LAUNCH_PID=$!

if ! timeout 60 bash -c 'until rostopic list 2>/dev/null | grep -q /fake_wrench/command; do sleep 0.5; done'; then
  echo "Fake wrench topic did not appear. Launch log:"
  tail -n 80 "${LOG}"
  exit 1
fi
if ! timeout 60 bash -c 'until rostopic list 2>/dev/null | grep -q /ft_safety/state; do sleep 0.5; done'; then
  echo "Safety state topic did not appear. Launch log:"
  tail -n 80 "${LOG}"
  exit 1
fi

if wait_state NORMAL 15; then pass "initial state NORMAL"; else fail "initial state NORMAL"; fi

# --- warning -------------------------------------------------------------
command_mode warn_force
if wait_state WARNING 5; then pass "warn_force -> WARNING"; else fail "warn_force -> WARNING"; fi

# --- latch stop ----------------------------------------------------------
# Give the mock action server an active goal so cancel_all_goals() is
# observable.  This is a mock action, not a robot controller.
python3 "${HERE}/send_mock_goal.py" >"${GOAL_LOG}" 2>&1 &
GOAL_PID=$!
sleep 1.5

rostopic echo -n1 /trajectory_execution_event >"${EVENT}" 2>/dev/null &
EVENT_PID=$!
sleep 0.5
command_mode stop_force
if wait_state FAULT 8; then pass "stop_force -> FAULT"; else fail "stop_force -> FAULT"; fi
sleep 0.5
kill "${EVENT_PID}" 2>/dev/null || true
wait "${EVENT_PID}" 2>/dev/null || true
if grep -q "stop" "${EVENT}"; then
  pass "trajectory_execution_event stop published"
else
  fail "trajectory_execution_event stop not observed"
fi

# Reset must be refused while still above the warning threshold.  Do this
# before the (potentially slow) log check, while stop_force is still active.
RESET_OUT="$(timeout 8 rosservice call /ft_safety/reset 2>&1 || true)"
if grep -q "success: False" <<<"${RESET_OUT}"; then
  pass "reset refused while wrench is high"
else
  fail "reset should have been refused (${RESET_OUT})"
fi

# The mock server writes this file with flush(); do not rely on the
# roslaunch log, which can be block-buffered.
for _ in $(seq 1 40); do
  grep -q "MOCK_CANCEL_RECEIVED" "${MOCK_EVENTS}" && break
  sleep 0.25
done
if grep -q "MOCK_CANCEL_RECEIVED" "${MOCK_EVENTS}"; then
  pass "follow_joint_trajectory cancel observed"
else
  fail "follow_joint_trajectory cancel not observed"
fi
kill "${GOAL_PID}" 2>/dev/null || true
GOAL_PID=""

# --- reset ---------------------------------------------------------------
command_mode normal
sleep 2.0
RESET_OUT="$(timeout 8 rosservice call /ft_safety/reset 2>&1 || true)"
if grep -q "success: True" <<<"${RESET_OUT}"; then
  pass "reset accepted when wrench is normal"
else
  fail "reset was not accepted (${RESET_OUT})"
fi
if wait_state NORMAL 5; then pass "state NORMAL after reset"; else fail "state NORMAL after reset"; fi

# --- watchdog ------------------------------------------------------------
command_mode dropout
if wait_state FAULT 8; then pass "watchdog -> FAULT"; else fail "watchdog -> FAULT"; fi
command_mode normal
sleep 1.5
RESET_OUT="$(timeout 8 rosservice call /ft_safety/reset 2>&1 || true)"
if grep -q "success: True" <<<"${RESET_OUT}"; then
  pass "reset accepted after watchdog and fresh data"
else
  fail "reset after watchdog failed (${RESET_OUT})"
fi

# --- debounce ------------------------------------------------------------
# 4 frames over the stop threshold + 1 frame below it, with debounce=5.
command_mode burst_force
sleep 1.5
NOW="$(state)"
if [[ "${NOW}" == "NORMAL" || "${NOW}" == "WARNING" ]]; then
  pass "burst_force did not latch FAULT (state=${NOW})"
else
  fail "burst_force latched ${NOW} (debounce should prevent this)"
fi

echo
if (( FAILS == 0 )); then
  echo "ALL CHECKS PASSED"
else
  echo "${FAILS} CHECK(S) FAILED"
  echo "Launch log: ${LOG}"
  tail -n 80 "${LOG}"
fi
exit "${FAILS}"
