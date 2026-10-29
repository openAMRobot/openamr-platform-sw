#!/usr/bin/env bash
# Local H-CI-1 reproducible cold WORKSPACE gate for Windows/WSL2 + Docker.
# No deletion of Docker images, no changes to existing source trees.
set -Eeuo pipefail

IMAGE='ghcr.io/openamrobot/ci-ros2-jazzy@sha256:1653d4bdadce9999a48f73315e2da7aa667d3777579a97b0f1c12146667892d2'
PLATFORM_SHA='056df0a18e2409414f37d8bd7fe96cd46ecb6f6d'
INTERFACES_SHA='77950470a87f1f5815346b2f000a662ee8c56f94'
LIMIT=600
START=$(date +%s)
PROOFS="$HOME/openamrobot-hci1-proofs"
mkdir -p "$PROOFS"
RUN="$(mktemp -d "$PROOFS/run-$(date -u +%Y%m%dT%H%M%SZ)-XXXX")"
PLATFORM="$RUN/platform"
INTERFACES="$RUN/interfaces"
CACHE=unknown
exec > >(tee "$RUN/console.log") 2>&1

finish() {
  local code=$? elapsed=$(( $(date +%s) - START ))
  trap - EXIT
  {
    echo '# OpenAMRobot H-CI-1 local cold-workspace proof'
    echo "- UTC date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "- Host: $(uname -srmo)"
    echo "- PR #57 source commit: $PLATFORM_SHA"
    echo "- Interfaces commit: $INTERFACES_SHA"
    echo "- Candidate image: $IMAGE"
    echo "- Image already cached: $CACHE"
    echo "- Time including clones, image pull, build and tests: $elapsed seconds"
    echo "- Gate succeeded: $([ "$code" -eq 0 ] && echo yes || echo no)"
    echo "- Under 600 seconds: $([ "$elapsed" -lt "$LIMIT" ] && echo yes || echo no)"
    if [ -f "$PLATFORM/ros2/test-evidence-summary.txt" ]; then
      echo "- Tests: $(head -n1 "$PLATFORM/ros2/test-evidence-summary.txt")"
    fi
    echo
    echo 'Scope: local clean checkout/workspace. If image was cached, this is'
    echo 'not proof of a first-time GHCR image download. It does not test the'
    echo 'graphical VS Code Dev Containers extension.'
  } > "$RUN/summary.md"
  echo
  echo "Evidence: $RUN/summary.md"
  echo "Log: $RUN/console.log"
  echo "Elapsed: $elapsed seconds"
  if [ "$code" -ne 0 ]; then
    echo "RESULT: FAIL (exit $code)"
    exit "$code"
  fi
  if [ "$elapsed" -ge "$LIMIT" ]; then
    echo 'RESULT: FAIL — full local gate exceeded 10 minutes'
    exit 2
  fi
  echo 'RESULT: PASS — local build and tests finished under 10 minutes'
}
trap finish EXIT

echo "Evidence folder: $RUN"
echo '[1/5] Checking WSL2/Linux Docker engine'
for cmd in git docker; do
  command -v "$cmd" >/dev/null || { echo "Missing $cmd in WSL"; exit 10; }
done
docker info >/dev/null || { echo 'Docker Desktop is not available inside WSL'; exit 11; }
test "$(docker info --format '{{.OSType}}')" = linux
docker version --format 'Docker server {{.Server.Version}}'

echo '[2/5] Cloning immutable revisions into a fresh WSL workspace'
git clone --quiet --single-branch --branch h-ci-1-devcontainer \
  https://github.com/openAMRobot/openamr-platform-sw.git "$PLATFORM"
git -C "$PLATFORM" checkout --quiet --detach "$PLATFORM_SHA"
test "$(git -C "$PLATFORM" rev-parse HEAD)" = "$PLATFORM_SHA"
git clone --quiet https://github.com/openAMRobot/openamrobot-interfaces.git "$INTERFACES"
git -C "$INTERFACES" checkout --quiet --detach "$INTERFACES_SHA"
test "$(git -C "$INTERFACES" rev-parse HEAD)" = "$INTERFACES_SHA"
test -s "$INTERFACES/ros2/openamr_nav_msgs/package.xml"
test ! -e "$PLATFORM/ros2/src/openamr_nav_msgs"
cp -a "$INTERFACES/ros2/openamr_nav_msgs" "$PLATFORM/ros2/src/openamr_nav_msgs"

echo '[3/5] Checking cached image and pulling immutable digest'
if docker image inspect "$IMAGE" >/dev/null 2>&1; then CACHE=yes; else CACHE=no; fi
echo "Image cached before benchmark: $CACHE"
docker pull --platform linux/amd64 "$IMAGE" || {
  echo 'GHCR pull failed. This image may require docker login to ghcr.io.'
  echo 'Do not paste credentials or tokens into ChatGPT.'
  exit 12
}

echo '[4/5] Building and testing ROS 2 inside the pinned image'
RUNNER=$(cat <<'INSIDE'
set -Eeuo pipefail
source /opt/ros/jazzy/setup.bash
test ! -e build && test ! -e install && test ! -e log
colcon list
colcon build --event-handlers console_direct+ | tee ../colcon-build.log
source install/setup.bash
colcon test --event-handlers console_direct+ | tee ../colcon-test.log
colcon test-result --verbose | tee test-result.txt
python3 - <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

reports = sorted(Path('build').glob('*/pytest.xml'))
if not reports:
    sys.exit('FAIL: no pytest XML reports')
total = skipped = failed = 0
nav = []
for path in reports:
    cases = list(ET.parse(path).getroot().iter('testcase'))
    total += len(cases)
    skipped += sum(any(c.tag == 'skipped' for c in item) for item in cases)
    failed += sum(any(c.tag in ('failure', 'error') for c in item) for item in cases)
    if path.parent.name == 'openamrobot_nav2':
        nav = cases
executed = total - skipped
if executed < 1 or failed:
    sys.exit(f'FAIL: executed={executed}, failures={failed}')
for suite in ('test_sensor_tracker', 'test_status_trackers',
              'test_status_rules', 'test_navigation_status_node'):
    if not any(
        suite in ' '.join(item.attrib.values()) and
        not any(c.tag == 'skipped' for c in item)
        for item in nav
    ):
        sys.exit(f'FAIL: navigation suite not executed: {suite}')
summary = f'JUnit cases={total}, executed={executed}, skipped={skipped}, failures={failed}'
Path('test-evidence-summary.txt').write_text(summary + '\n', encoding='utf-8')
print(summary)
PY
INSIDE
)
docker run --rm --platform linux/amd64 --pull=never \
  --mount "type=bind,source=$PLATFORM,target=/repo" \
  --workdir /repo/ros2 --env OPENAMR_REQUIRE_INTERFACES=1 \
  "$IMAGE" bash -lc "$RUNNER"

echo '[5/5] Capturing evidence'
test -s "$PLATFORM/ros2/test-evidence-summary.txt"
cat "$PLATFORM/ros2/test-evidence-summary.txt"
