"""Paths, filenames, and constants for the Carrot model selector."""
from __future__ import annotations

from pathlib import Path

# Storage locations
MODELS_DIR = Path("/data/models")
MODELS_TMP_DIR = Path("/data/models_tmp")
MODELS_BACKUP_DIR = Path("/data/models_backup")
COMPILE_STATUS_FILE = Path("/data/model_compile_status")

# 编译环境戳 — 若 /data/models/.compile_env 内容与当前 tag 不一致，
# boot_compile 会用保留的 onnx 自动重新编译（让旧引擎安装的模型在
# 启动时重新构建，而不是加载失败 → 被隔离）。
COMPILE_ENV_STAMP_NAME = ".compile_env"
# 重编译失败时留下的标记，避免同一 tag 下每次开机重复尝试（拖慢启动）。
RECOMPILE_FAILED_MARKER_NAME = ".recompile_failed"

# tag 从 git 树自动推导：凡触及下面路径的提交（即使由自动 cherry-pick
# 带入）都会自动改变 tag 并触发重编译 — 无需人工上调版本号。
# 只列决定编译产物（pkl）兼容性的路径。
_COMPILE_ENV_PATHS = (
    "tinygrad_repo",
    "carrot/model_selector/config.py",
    "carrot/model_selector/installer.py",
    "carrot/model_selector/compile_legacy_warp.py",
    "carrot/model_selector/carrot_modeld.py",
    "openpilot/selfdrive/modeld/compile_modeld.py",
    "openpilot/selfdrive/modeld/compile_dm_warp.py",
    "openpilot/selfdrive/modeld/get_model_metadata.py",
    "openpilot/selfdrive/modeld/helpers.py",
    "openpilot/selfdrive/modeld/constants.py",
    "openpilot/selfdrive/modeld/modeld.py",
    "openpilot/selfdrive/modeld/dmonitoringmodeld.py",
    "openpilot/selfdrive/modeld/SConscript",
    "openpilot/common/file_chunker.py",
    "openpilot/common/transformations/camera.py",
    "openpilot/common/transformations/model.py",
    "openpilot/system/camerad/cameras/nv12_info.py",
    "openpilot/system/hardware/hw.py",
)
# 无法使用 git 的环境（无 .git 的发布包等）的回退值 — 仅在该环境下人工维护。
TINYGRAD_UPSTREAM_REVISION = "1858f1fd9aa94ca4e302b60a88f075d0d1dd88bc"
_COMPILE_ENV_TAG_FALLBACK = f"2026.08-tg-{TINYGRAD_UPSTREAM_REVISION[:8]}"
_compile_env_tag_cache: str | None = None


def compile_env_tag() -> str:
    global _compile_env_tag_cache
    if _compile_env_tag_cache is None:
        _compile_env_tag_cache = _derive_compile_env_tag()
    return _compile_env_tag_cache


def _derive_compile_env_tag() -> str:
    import hashlib
    import subprocess
    repo_root = Path(__file__).resolve().parents[2]
    try:
        out = subprocess.run(
            ["git", "-C", str(repo_root), "rev-parse", *[f"HEAD:{p}" for p in _COMPILE_ENV_PATHS]],
            capture_output=True, text=True, timeout=10, check=True,
        )
        digest = hashlib.sha256(out.stdout.encode()).hexdigest()[:16]
        return f"tg-env:{digest}"
    except Exception:
        return _COMPILE_ENV_TAG_FALLBACK


def compile_env_tag_file_matches(path: Path) -> bool:
    """Return whether *path* contains the current compiler-generation tag.

    This sidecar check must happen before unpickling a tinygrad artifact: an
    older pickle can fail while its TinyJit objects are being reconstructed,
    before any metadata stored inside the pickle is available to inspect.
    """
    try:
        return path.read_text().strip() == compile_env_tag()
    except (OSError, ValueError):
        return False


def model_compile_env_is_current(model_dir: Path) -> bool:
    return compile_env_tag_file_matches(model_dir / COMPILE_ENV_STAMP_NAME)

# Default built-in model directory (fallback when no custom model is installed)
# 注意：本树的实际源码位于 openpilot/ 命名空间之下
# (selfdrive → openpilot/selfdrive).
OPENPILOT_ROOT = Path("/data/openpilot")
DEFAULT_MODEL_DIR = OPENPILOT_ROOT / "openpilot" / "selfdrive" / "modeld" / "models"

# 远程 manifest — 双轨制:
#  - models_v4.json: 完整目录（v4+ 选择器主清单，含 supercombo 等新架构文件名）
#  - models.json:    仅含旧架构文件名（为 v3 及以下旧版本冻结）
# 旧版(v3) manifest 解析器在 minimum_selector_version 门槛"之前"就检查文件名，
# 只要出现一个未知文件名就让整个列表失败，因此含新文件名的条目
# 绝不能放进 models.json。openpilot-models 仓库的
# scripts/update_models.py 会自动分离生成并签名这两个文件。
_MODELS_JSON_BASE = "https://raw.githubusercontent.com/happymaj11r/openpilot-models/main"
MODELS_JSON_URL = f"{_MODELS_JSON_BASE}/models_v4.json"
# 拉取 models_v4.json 失败时（文件缺失/临时故障）回退到旧版 manifest。
MODELS_JSON_FALLBACK_URL = f"{_MODELS_JSON_BASE}/models.json"
# str.startswith 接受元组，因此无需改动 downloader 的校验逻辑。
# releases/download 前缀用于超过 95MB、无法走 GitHub raw 托管的文件
# （例如 Giga 的 driving_vision.onnx 122MB）以 Release 附件分发的场景。
ALLOWED_URL_PREFIX = (
    "https://raw.githubusercontent.com/happymaj11r/openpilot-models/",
    "https://github.com/happymaj11r/openpilot-models/releases/download/",
)

# Allowed onnx filenames for download (allowlist)
ALLOWED_ONNX_FILES = frozenset({
    "driving_vision.onnx",
    "driving_policy.onnx",
    "driving_on_policy.onnx",
    "driving_off_policy.onnx",
    "driving_supercombo.onnx",
})

# Base names that we compile (.onnx → _tinygrad.pkl + _metadata.pkl)
VISION_BASE = "driving_vision"
ON_POLICY_BASE = "driving_on_policy"
POLICY_BASE = "driving_policy"
OFF_POLICY_BASE = "driving_off_policy"

# New-architecture (lebowski) single-onnx model: compile_modeld.py bundles
# metadata + model JIT + per-resolution warp JITs into one pkl that the
# upstream modeld engine loads directly.
SUPERCOMBO_BASE = "driving_supercombo"
SUPERCOMBO_PKL_NAME = "driving_tinygrad.pkl"

# Env var honored by openpilot/selfdrive/modeld/helpers.py::modeld_pkl_path() to load
# the unified pkl from a custom directory instead of the built-in models dir.
MODELD_MODELS_DIR_ENV = "MODELD_MODELS_DIR"

# Params keys
PARAM_DRIVING_MODEL_NAME = "DrivingModelName"
PARAM_PENDING_MODEL_NAME = "PendingModelName"

# tinygrad compile flags (must match openpilot/selfdrive/modeld/SConscript)
TINYGRAD_COMPILE_ENV_QCOM = {
    "DEV": "QCOM",
    "FLOAT16": "1",
    "NOLOCALS": "1",
    "JIT_BATCH_SIZE": "0",
    "IMAGE": "1",
    "OPENPILOT_HACKS": "1",
}
TINYGRAD_COMPILE_ENV_FALLBACK = {
    "DEV": "CPU:LLVM",
    # THREADS=0 是 SConscript 中没有的有意追加（保证 PC 回退编译的稳定性）。
    # 内核结构已烧录进 pkl，不会与运行期冲突。
    "THREADS": "0",
}

# Model ID validation (matches model_manager.cc isValidModelId())
MODEL_ID_REGEX = r"^[A-Za-z0-9_\-\s]{1,64}$"
