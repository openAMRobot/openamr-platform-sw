#!/usr/bin/env bash
# Build the firmware image and fetch Renode for stm32:=renode and stm32:=renode_readonly.
#
# Usage: sil_renode_firmware.sh [--default-too] [--uros] OUT_DIR
#
# 1. Fetches openamr-platform-fw at the pinned commit (openamrobot_sil/renode_fw.py FW_COMMIT)
#    into OUT_DIR/openamr-platform-fw and checks that HEAD is that commit.
# 2. Checks that the Renode URL and SHA-256 pinned here are the ones of that commit's
#    .github/workflows/firmware-nightly.yml.
# 3. Builds boards/stm32/telemetry_image with CMake preset RenodeTest (needs arm-none-eabi-gcc,
#    CMake 3.22+, Ninja and github.com for the FreeRTOS fetch) and, with --default-too, the
#    Debug preset as well (a build whose banner prints "params":"default", for the negative test),
#    and with --uros the TelemetryUrosSerialSimBench preset (micro-ROS over USART3, for
#    stm32:=renode_readonly; needs libstdc++-arm-none-eabi-newlib, colcon, catkin_pkg, empy 3 and
#    lark, and fetches the pinned micro-ROS repositories) plus the micro-ROS agent, built by
#    the firmware checkout's tools/ci/build_uros_agent.sh (needs ROS 2 Jazzy sourced).
# 4. Downloads the Renode portable tarball, checks its SHA-256, unpacks it to OUT_DIR/renode.
#
# Prints SIL_RENODE, SIL_FW_ELF, SIL_FW_DIR (and SIL_FW_ELF_DEFAULT, SIL_FW_ELF_UROS,
# SIL_UROS_AGENT) as KEY=VALUE lines on
# stdout and writes them to OUT_DIR/sil_renode.env. Nothing in openamr-platform-fw is changed.
set -euo pipefail
default_too=0
uros=0
while [ "${1:-}" = "--default-too" ] || [ "${1:-}" = "--uros" ]; do
  [ "$1" = "--default-too" ] && default_too=1
  [ "$1" = "--uros" ] && uros=1
  shift
done
out="${1:?usage: sil_renode_firmware.sh [--default-too] [--uros] OUT_DIR}"
here="$(cd "$(dirname "$0")" && pwd)"
pins="$here/../openamrobot_sil/renode_fw.py"
[ -f "$pins" ] || pins="$(python3 -c 'import openamrobot_sil.renode_fw as m; print(m.__file__)')"
eval "$(python3 -I "$pins" shell)"
mkdir -p "$out"; out="$(cd "$out" && pwd)"
fw="$out/openamr-platform-fw"

echo "firmware: $FW_REPOSITORY@$FW_COMMIT preset $FW_PRESET" >&2
if [ ! -d "$fw/.git" ]; then
  git init -q "$fw"
  git -C "$fw" remote add origin "https://github.com/$FW_REPOSITORY.git"
fi
git -C "$fw" fetch -q --depth 1 origin "$FW_COMMIT"
git -C "$fw" checkout -q --detach FETCH_HEAD
head="$(git -C "$fw" rev-parse HEAD)"
if [ "$head" != "$FW_COMMIT" ]; then
  echo "FAILED: openamr-platform-fw HEAD is $head, pinned $FW_COMMIT" >&2; exit 1
fi
if [ -n "$(git -C "$fw" status --porcelain)" ]; then
  echo "FAILED: the openamr-platform-fw checkout has changes" >&2; exit 1
fi

nightly="$fw/.github/workflows/firmware-nightly.yml"
grep -q "RENODE_URL: $RENODE_URL\$" "$nightly" \
  || { echo "FAILED: RENODE_URL differs from $nightly" >&2; exit 1; }
grep -q "RENODE_SHA256: $RENODE_SHA256\$" "$nightly" \
  || { echo "FAILED: RENODE_SHA256 differs from $nightly" >&2; exit 1; }

presets=("$FW_PRESET")
[ "$default_too" = 1 ] && presets+=(Debug)
[ "$uros" = 1 ] && presets+=("$FW_PRESET_UROS")
# The firmware builds without ROS: with ROS 2 sourced, CMake would find /opt/ros packages
# through PATH while it builds the micro-ROS library (platform-fw uros/build_libmicroros.sh
# clears the ROS variables, not PATH). So each preset is built with the ROS paths removed;
# ROS stays sourced for the agent build below.
no_ros_path="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v '^/opt/ros/' | paste -sd: -)"
for preset in "${presets[@]}"; do
  echo "building $FW_PROJECT_DIR preset $preset" >&2
  (unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH \
         ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
   export PATH="$no_ros_path"
   cd "$fw/$FW_PROJECT_DIR" && cmake --preset "$preset" >&2 && cmake --build --preset "$preset" >&2)
done
elf="$fw/$FW_ELF"
[ -f "$elf" ] || { echo "FAILED: no $elf" >&2; exit 1; }
agent=""
if [ "$uros" = 1 ]; then
  [ -f "$fw/$FW_ELF_UROS" ] || { echo "FAILED: no $fw/$FW_ELF_UROS" >&2; exit 1; }
  echo "building the micro-ROS agent ($FW_UROS_AGENT_SCRIPT at $FW_COMMIT)" >&2
  bash "$fw/$FW_UROS_AGENT_SCRIPT" "$out/agent_ws" >&2
  bin="$out/agent_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
  [ -x "$bin" ] || { echo "FAILED: no $bin" >&2; exit 1; }
  # The agent needs its workspace's libraries (micro_ros_msgs type support): a wrapper sources
  # that workspace, so a caller with only ROS 2 sourced can start it.
  agent="$out/micro_ros_agent.sh"
  cat > "$agent" <<WRAP
#!/usr/bin/env bash
set +u
. "$out/agent_ws/install/setup.bash"
exec "$bin" "\$@"
WRAP
  chmod +x "$agent"
fi

renode_dir="$out/renode"
if [ ! -x "$renode_dir/renode" ]; then
  curl -sSL --retry 3 -o "$out/renode.tar.gz" "$RENODE_URL"
  echo "$RENODE_SHA256  $out/renode.tar.gz" | sha256sum -c - >&2
  mkdir -p "$renode_dir"
  tar xzf "$out/renode.tar.gz" -C "$renode_dir" --strip-components=1
fi

{
  echo "SIL_RENODE=$renode_dir/renode"
  echo "SIL_FW_ELF=$elf"
  echo "SIL_FW_DIR=$fw"
  [ "$default_too" = 1 ] && echo "SIL_FW_ELF_DEFAULT=$fw/$FW_PROJECT_DIR/build/Debug/telemetry_image.elf"
  [ "$uros" = 1 ] && echo "SIL_FW_ELF_UROS=$fw/$FW_ELF_UROS"
  [ "$uros" = 1 ] && echo "SIL_UROS_AGENT=$agent"
  [ "$uros" = 1 ] && echo "SIL_FW_PRESET_UROS=$FW_PRESET_UROS"
  echo "SIL_FW_COMMIT=$FW_COMMIT"
  echo "SIL_FW_PRESET=$FW_PRESET"
  echo "SIL_FW_ELF_SHA256=$(sha256sum "$elf" | cut -d' ' -f1)"
} | tee "$out/sil_renode.env"
