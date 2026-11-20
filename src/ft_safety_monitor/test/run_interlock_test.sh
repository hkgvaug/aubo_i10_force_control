#!/usr/bin/env bash
# Fake-execution test for the fail-closed safety interlock in
# moveit_pose_executor.  No aubo_driver, no real robot, no Gazebo.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${HOME}/aubo_ws"
LOG="$(mktemp /tmp/ft_interlock_test.XXXXXX.log)"
FAILS=0

source /opt/ros/noetic/setup.bash
source "${WS}/devel/setup.bash"

cleanup() {
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
    [[ "${now}" == "${expected}" ]] && return 0
    sleep 0.2
  done
  echo "  last state: '${now}' (expected '${expected}')"
  return 1
}

command_mode() {
  rostopic pub -1 /fake_wrench/command std_msgs/String "data: '$1'" >/dev/null 2>&1
}

publish_target() {
  local seq="$1"
  timeout 12 python3 "${HERE}/send_target.py" "${seq}" >/dev/null 2>&1
}

result_after() {
  local min_stamp="$1"
  timeout 45 python3 "${HERE}/get_execution_result.py" \
    "_min_stamp:=${min_stamp}" 2>/dev/null | tail -n1
}

echo "Launching interlock test stack (log: ${LOG})"
roslaunch ft_safety_monitor ft_safety_interlock_test.launch >"${LOG}" 2>&1 &
LAUNCH_PID=$!

for topic in /ft_safety/state /bim/execution_result /fake_wrench/command; do
  if ! timeout 90 bash -c "until rostopic list 2>/dev/null | grep -q '${topic}'; do sleep 0.5; done"; then
    echo "Topic ${topic} did not appear. Launch log:"
    tail -n 80 "${LOG}"
    exit 1
  fi
done

if wait_state NORMAL 25; then pass "safety state NORMAL"; else fail "safety state NORMAL"; fi

# 1. FAULT must reject before planning.
command_mode stop_force
if wait_state FAULT 10; then pass "stop_force -> FAULT"; else fail "stop_force -> FAULT"; fi
MIN_STAMP="$(date +%s.%N)"
publish_target 1
R="$(result_after "${MIN_STAMP}")"
echo "  result 1: ${R}"
if grep -q '"success": false' <<<"${R}" && grep -q 'safety interlock' <<<"${R}"; then
  pass "FAULT rejects execution with safety interlock"
else
  fail "FAULT did not reject execution as expected"
fi

# 2. NORMAL + fresh state must pass the interlock (planning may still fail).
command_mode normal
sleep 1.5
RESET_OUT="$(timeout 8 rosservice call /ft_safety/reset 2>&1 || true)"
if grep -q "success: True" <<<"${RESET_OUT}"; then
  pass "fault reset"
else
  fail "fault reset (${RESET_OUT})"
fi
if wait_state NORMAL 5; then pass "state NORMAL after reset"; else fail "state NORMAL after reset"; fi

MIN_STAMP="$(date +%s.%N)"
publish_target 2
R="$(result_after "${MIN_STAMP}")"
echo "  result 2: ${R}"
if grep -q 'safety interlock' <<<"${R}"; then
  fail "NORMAL state was wrongly rejected"
else
  pass "NORMAL state passed the interlock"
fi

# 3. Monitor down / stale state must reject.
rosnode kill /ft_safety_monitor >/dev/null 2>&1 || true
sleep 2.5
MIN_STAMP="$(date +%s.%N)"
publish_target 3
R="$(result_after "${MIN_STAMP}")"
echo "  result 3: ${R}"
if grep -q '"success": false' <<<"${R}" && \
   grep -q -E 'safety state is stale|no safety state|safety state is None' <<<"${R}"; then
  pass "missing/stale safety state rejects execution"
else
  fail "missing/stale safety state did not reject execution"
fi

echo
if (( FAILS == 0 )); then
  echo "ALL INTERLOCK CHECKS PASSED"
else
  echo "${FAILS} CHECK(S) FAILED"
  echo "Launch log: ${LOG}"
  tail -n 100 "${LOG}"
fi
exit "${FAILS}"
