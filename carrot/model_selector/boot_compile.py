"""Tiny hook called once from `system/manager/manager.py::main()` so the
upstream patch stays a single line:

    from openpilot.carrot.model_selector.boot_compile import run as _ms_boot
    _ms_boot()

All heavy lifting lives in `installer`.  This wrapper exists so the patch
site doesn't have to import anything heavy (e.g. tinygrad) unless a pending
model is present or the installed model needs a recompile.
"""
from __future__ import annotations

from openpilot.common.swaglog import cloudlog

from .config import (
    COMPILE_ENV_STAMP_NAME,
    MODELS_DIR,
    MODELS_TMP_DIR,
    RECOMPILE_FAILED_MARKER_NAME,
    compile_env_tag_file_matches,
)


def run() -> None:
    try:
        if MODELS_TMP_DIR.exists():
            from .installer import compile_pending
            compile_pending()
            # 这里不 return — 若成功，新 /data/models 的戳与当前一致，
            # 下面的检查就是 no-op；若只是失败清理，则应在同一次启动中
            # 立即尝试恢复性重编译（否则这次启动期间 stale pkl 会暴露给
            # modeld，导致崩溃循环 → 隔离）。

        # 已安装的自定义模型若是在旧编译环境（不同 tinygrad/pkl 格式）
        # 下构建的，则用保留的 onnx 在开机时自动重编译。
        # 戳一致（正常）或存在失败标记（重试无意义）时，
        # 不做重量级 import 直接跳过。
        stamp_ok = compile_env_tag_file_matches(MODELS_DIR / COMPILE_ENV_STAMP_NAME)
        already_failed = compile_env_tag_file_matches(MODELS_DIR / RECOMPILE_FAILED_MARKER_NAME)
        if MODELS_DIR.is_dir() and not stamp_ok and not already_failed:
            from .installer import recompile_stale_if_needed
            recompile_stale_if_needed()
    except Exception as e:
        # Never crash manager on install errors — installer already restores
        # the backup and wipes the tmp dir on failure.
        cloudlog.error(f"model_selector boot_compile: {e}")
