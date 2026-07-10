#!/bin/bash
# =============================================================================
# teleimager-test deployment helper
# -----------------------------------------------------------------------------
# Creates an isolated conda env and installs this test package (separate from
# the production `teleimager`). Run this ON THE ROBOT, from inside this folder:
#
#     cd ~/teleimager-test
#     bash setup_test_env.sh
#
# It is safe to re-run. After it finishes:
#     conda activate teleimager-test
#     teleimager-test-server --cf          # list cameras / serials
#     teleimager-test-server --rs          # start server (RealSense depth)
#     teleimager-test-client --host <server-ip>
# =============================================================================
set -e

ENV_NAME="${1:-teleimager-test}"
PY_VER="${2:-3.10}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== teleimager-test setup ==="
echo "conda env : $ENV_NAME (python $PY_VER)"
echo "package   : $SCRIPT_DIR"

if ! command -v conda >/dev/null 2>&1; then
    echo "ERROR: conda not found on PATH. Install/activate conda first." >&2
    exit 1
fi
# make `conda activate` usable inside this non-interactive shell
source "$(conda info --base)/etc/profile.d/conda.sh"

# 1. create env if missing
if conda env list | grep -qE "^\s*${ENV_NAME}\s"; then
    echo "[1/4] conda env '$ENV_NAME' already exists, reusing."
else
    echo "[1/4] creating conda env '$ENV_NAME'..."
    conda create -y -n "$ENV_NAME" "python=$PY_VER"
fi

conda activate "$ENV_NAME"

# 2. install the package (core + server extras: aiohttp/aiortc/pupil-labs-uvc)
echo "[2/4] installing teleimager-test (editable, with [server])..."
pip install -e "${SCRIPT_DIR}[server]"

# 3. RealSense support (color + depth)
ARCH="$(uname -m)"
echo "[3/4] installing RealSense support (arch=$ARCH)..."
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    echo "  Jetson/aarch64 detected: pyrealsense2 has no prebuilt wheel here."
    echo "  Build librealsense + pyrealsense2 from source, then re-run, OR skip depth."
    echo "  See README for the build steps."
else
    pip install -e "${SCRIPT_DIR}[realsense]"
fi

# 4. sanity check
echo "[4/4] verifying imports..."
python - <<'PY'
import teleimager_test
try:
    import pyrealsense2 as rs
    n = len(rs.context().query_devices())
    print(f"  pyrealsense2 OK, RealSense devices detected: {n}")
except Exception as e:
    print(f"  pyrealsense2 NOT available: {e} (depth will be disabled)")
print("  teleimager_test import OK")
PY

echo ""
echo "=== done ==="
echo "Next:"
echo "  conda activate $ENV_NAME"
echo "  teleimager-test-server --cf     # confirm camera serials, then edit cam_config_server.yaml"
echo "  teleimager-test-server --rs     # start (RealSense head -> RGB + depth)"
