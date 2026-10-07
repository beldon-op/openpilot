#!/usr/bin/env python3
"""模型选择器 ↔ upstream 契约检查。

检查 upstream 提交（尤其是自动 cherry-pick）是否破坏了选择器所依赖的契约。
cherry-pick/merge 之后必须运行本脚本并确认全部 PASS — 出现 FAIL 意味着
需要让选择器代码随之适配（不会自动修改）。

用法（任意目录均可）：
    python3 carrot/model_selector/check_contracts.py                    # 契约检查
    python3 carrot/model_selector/check_contracts.py --sync-baselines  # 镜像 review 后刷新快照

只使用标准库 — 不依赖 openpilot 构建/重量级依赖。
"""
from __future__ import annotations

import ast
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

MODELD_DIR = REPO_ROOT / "openpilot" / "selfdrive" / "modeld"

# installer.py 以 subprocess 方式执行的脚本（仓库相对路径契约）
REQUIRED_SCRIPTS = (
    MODELD_DIR / "get_model_metadata.py",
    MODELD_DIR / "compile_modeld.py",
    REPO_ROOT / "tinygrad_repo" / "examples" / "openpilot" / "compile3.py",
    REPO_ROOT / "carrot" / "model_selector" / "compile_legacy_warp.py",
)

# installer._compile_supercombo 组装 compile_modeld CLI 参数的契约
REQUIRED_CLI_FLAGS = ("--model-size", "--camera-resolutions", "--onnx", "--output", "--frame-skip")

# carrot_modeld 以位置参数调用的 upstream 函数签名契约
EXPECTED_SIGNATURES = {
    "fill_model_msg": ["msg", "net_output_data", "action", "publish_state", "vipc_frame_id",
                       "vipc_frame_id_extra", "frame_id", "frame_drop", "timestamp_eof",
                       "model_execution_time", "valid"],
    "fill_pose_msg": ["msg", "net_output_data", "vipc_frame_id", "vipc_dropped_frames",
                      "timestamp_eof", "live_calib_seen"],
    "fill_driving_model_data": ["msg", "modelv2_send"],
}

# installer/_validate 与 carrot_modeld 引用的常量契约
REQUIRED_MODEL_CONSTANTS = ("MODEL_RUN_FREQ", "MODEL_CONTEXT_FREQ", "DESIRE_LEN",
                            "TRAFFIC_CONVENTION_LEN", "FEATURE_LEN")

REQUIRED_COMPILE_ENV_PATHS = {
    "tinygrad_repo",
    "carrot/model_selector/config.py",
    "carrot/model_selector/installer.py",
    "carrot/model_selector/compile_legacy_warp.py",
    "carrot/model_selector/carrot_modeld.py",
    "openpilot/selfdrive/modeld/SConscript",
    "openpilot/selfdrive/modeld/compile_modeld.py",
    "openpilot/selfdrive/modeld/compile_dm_warp.py",
    "openpilot/selfdrive/modeld/get_model_metadata.py",
    "openpilot/selfdrive/modeld/helpers.py",
    "openpilot/selfdrive/modeld/constants.py",
    "openpilot/selfdrive/modeld/modeld.py",
    "openpilot/selfdrive/modeld/dmonitoringmodeld.py",
    "openpilot/common/file_chunker.py",
}

# carrot_modeld / carrot_parse_model_outputs "镜像"的 upstream 文件。
# 与 fill_model_msg 那种直接 import 的文件不同，这两个文件的逻辑变更
# （例如 has_wide_camera 分支）不会自动同步，需要人工判断后移植 —
# 签名不变时其它检查抓不到。
# 与上次镜像 review 时的快照（upstream_baseline/）比对并提示变更。
BASELINE_DIR = Path(__file__).resolve().parent / "upstream_baseline"
MIRRORED_UPSTREAM_FILES = {
    MODELD_DIR / "modeld.py": BASELINE_DIR / "modeld.py.baseline",
    MODELD_DIR / "parse_model_outputs.py": BASELINE_DIR / "parse_model_outputs.py.baseline",
}


def _func_params(path: Path, name: str) -> list[str] | None:
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return [a.arg for a in node.args.args]
    return None


def check_script_paths() -> str | None:
    missing = [str(p.relative_to(REPO_ROOT)) for p in REQUIRED_SCRIPTS if not p.is_file()]
    return f"missing: {missing}" if missing else None


def check_helpers_hook() -> str | None:
    """helpers.modeld_pkl_path() 是否响应 MODELD_MODELS_DIR 覆写。

    需要设备同款 Python（3.10+）；在老版本解释器上 import helpers 会
    因 PEP 604 标注直接报错，这里把导入失败与钩子丢失区分开。
    """
    probe = "/tmp/__ms_contract_check__"
    old = os.environ.get("MODELD_MODELS_DIR")
    os.environ["MODELD_MODELS_DIR"] = probe
    try:
        try:
            from openpilot.selfdrive.modeld.helpers import modeld_pkl_path
        except TypeError as e:
            # 宿主解释器过老（TypeVar | None 等），不是契约问题 — 退回静态检查
            src = (MODELD_DIR / "helpers.py").read_text()
            if "MODELD_MODELS_DIR" in src and "modeld_pkl_path" in src:
                return None
            return f"静态兜底也未找到 MODELD_MODELS_DIR 钩子: {e!r}"
        got = Path(modeld_pkl_path(False))
        big = Path(modeld_pkl_path(True))
        if got.parent != Path(probe) or big.parent != Path(probe):
            return f"MODELD_MODELS_DIR ignored — hook lost (got {got}, big {big})"
        big_ok = big.name.startswith("big_driving_") and big.name.endswith("_tinygrad.pkl")
        if got.name != "driving_tinygrad.pkl" or not big_ok:
            return f"pkl naming changed: {got.name}, {big.name}"
        return None
    finally:
        if old is None:
            os.environ.pop("MODELD_MODELS_DIR", None)
        else:
            os.environ["MODELD_MODELS_DIR"] = old


def check_compile_modeld_cli() -> str | None:
    src = (MODELD_DIR / "compile_modeld.py").read_text()
    missing = [f for f in REQUIRED_CLI_FLAGS if f"'{f}'" not in src and f'"{f}"' not in src]
    return f"argparse flags missing: {missing}" if missing else None


def check_metadata_naming() -> str | None:
    src = (MODELD_DIR / "get_model_metadata.py").read_text()
    if "_metadata.pkl" not in src:
        return "get_model_metadata.py no longer derives {stem}_metadata.pkl output naming"
    return None


def check_pkl_format_pair() -> str | None:
    """installer 编译出的 pkl 与 modeld 读取的序列化格式必须成对维持。"""
    compile_src = (MODELD_DIR / "compile_modeld.py").read_text()
    modeld_src = (MODELD_DIR / "modeld.py").read_text()
    if "dump_oob" not in compile_src:
        return "compile_modeld.py no longer writes with dump_oob"
    # upstream 可能先传入显式的 override（pkl_path 或 modeld_pkl_path(...)），
    # 所以同一调用里直到 modeld_pkl_path 回退为止都允许。
    if not re.search(r"load_oob\(open_file_chunked\([^)]*modeld_pkl_path", modeld_src):
        return "modeld.py loader changed (expected load_oob(open_file_chunked(... modeld_pkl_path(...))))"
    return None


def check_tinygrad_pickle_compat() -> str | None:
    """编译与加载两侧的 Buffer pickle 方案必须配套。

    两种合法组合：
    1. 本分支形态 — vendored tinygrad 的 Buffer 没有原生 __reduce_ex__，
       compile_modeld.py 用构造函数式 monkeypatch 补上；reduce 结果只含
       (类, 构造参数)，加载端无需再打补丁，与出厂内置模型的流程一致。
    2. 新 tinygrad — Buffer 自带 __reduce_ex__，monkeypatch 必须保持移除。
    两者都缺（无法序列化）或同时存在（补丁覆盖原生实现、埋升级隐患）都算违约。
    """
    compile_src = (MODELD_DIR / "compile_modeld.py").read_text()
    device_src = (REPO_ROOT / "tinygrad_repo" / "tinygrad" / "device.py").read_text()
    has_patch = "_patch_tinygrad_buffer_reduce" in compile_src
    has_native = "def __reduce_ex__" in device_src
    if has_patch and has_native:
        return "tinygrad 已原生支持 Buffer pickle，compile_modeld.py 的 monkeypatch 应移除"
    if not has_patch and not has_native:
        return "tinygrad Buffer 无原生 __reduce_ex__，且 compile_modeld.py 缺少 monkeypatch"
    return None


def check_sconscript_flags() -> str | None:
    """installer 的编译环境必须与 SCons 构建内置模型时的 QCOM flags 一致。

    注意：本分支的 SConscript 仍用 probe_devices() 在构建期探测后端
    （源码分支改为按构建架构选择）；契约的实质是 QCOM 分支的 tg_flags
    与 config.TINYGRAD_COMPILE_ENV_QCOM 保持同步，故这里检查 QCOM 段。
    """
    from carrot.model_selector.config import TINYGRAD_COMPILE_ENV_QCOM
    src = (MODELD_DIR / "SConscript").read_text()
    if "tg_backend = 'QCOM'" not in src:
        return "SConscript 中找不到 QCOM 后端分支"
    m = re.search(r"DEV=\{tg_backend\}\s+([A-Z0-9_= ]+)'", src)
    if not m:
        return "cannot locate QCOM tg_flags line in SConscript"
    sconscript = dict(kv.split("=", 1) for kv in m.group(1).split())
    expected = {k: v for k, v in TINYGRAD_COMPILE_ENV_QCOM.items() if k != "DEV"}
    if sconscript != expected:
        return f"QCOM flags diverged: SConscript={sconscript} config={expected}"
    return None


def check_compile_env_guards() -> str | None:
    """旧 tinygrad 的 pkl 必须在两个加载器运行之前被拒绝。"""
    from carrot.model_selector.config import _COMPILE_ENV_PATHS

    problems = []
    missing = sorted(REQUIRED_COMPILE_ENV_PATHS - set(_COMPILE_ENV_PATHS))
    if missing:
        problems.append(f"compiler fingerprint missing paths: {missing}")

    runner = (REPO_ROOT / "carrot/model_selector/modeld_runner.py").read_text()
    if "model_compile_env_is_current(CUSTOM_MODELS_DIR)" not in runner:
        problems.append("modeld_runner does not reject stale custom PKLs before load")

    launcher = (REPO_ROOT / "launch_chffrplus.sh").read_text()
    if "from carrot.model_selector.config import compile_env_tag" not in launcher:
        problems.append("内置模型 stamp 未使用与 installer 相同的编译器指纹")
    if "HEAD:openpilot/selfdrive/modeld HEAD:tinygrad_repo" in launcher:
        problems.append("内置模型 stamp 仍在哈希会自变的 modeld 目录树")

    # release 预编译流程守卫：仅当本分支的打包脚本引入 prepare_prebuilt_models
    # 时才适用（本分支 release 不在打包期编译模型，由开机首启完成）。
    release_path = REPO_ROOT / "release" / "build_carrot.sh"
    if release_path.is_file() and "prepare_prebuilt_models" in release_path.read_text():
        release = release_path.read_text()
        build_at = release.find("prepare_prebuilt_models\n")
        delete_at = release.find('rm -f -- openpilot/selfdrive/modeld/models/*.onnx')
        if build_at < 0 or delete_at < 0 or build_at >= delete_at:
            problems.append("prebuilt model compilation must happen before ONNX removal")
        if 'validate_prebuilt_models "$FINAL_MODEL_TAG"' not in release:
            problems.append("final prebuilt compiler stamp is not verified")

    return "; ".join(problems) if problems else None


def check_fill_model_msg_signatures() -> str | None:
    path = MODELD_DIR / "fill_model_msg.py"
    bad = []
    for name, expected in EXPECTED_SIGNATURES.items():
        actual = _func_params(path, name)
        if actual != expected:
            bad.append(f"{name}: expected {expected}, got {actual}")
    return "; ".join(bad) if bad else None


def check_model_constants() -> str | None:
    src = (MODELD_DIR / "constants.py").read_text()
    missing = [c for c in REQUIRED_MODEL_CONSTANTS if not re.search(rf"^\s*{c}\s*=", src, re.M)]
    return f"ModelConstants missing: {missing}" if missing else None


def check_modeld_mirror() -> str | None:
    stale = []
    for current, baseline in MIRRORED_UPSTREAM_FILES.items():
        if not baseline.is_file():
            stale.append(f"{baseline.name} 缺失")
        elif current.read_bytes() != baseline.read_bytes():
            stale.append(current.name)
    if stale:
        return ("自上次镜像 review 后发生变更: " + ", ".join(stale)
                + " — 请与 upstream_baseline/ 快照 diff，把所需逻辑移植进 carrot_modeld.py /"
                  " carrot_parse_model_outputs.py，然后用 `check_contracts.py --sync-baselines` 刷新快照")
    return None


def sync_baselines() -> None:
    """镜像 review（移植）完成后运行 — 把当前 upstream 文件存为新快照。"""
    for current, baseline in MIRRORED_UPSTREAM_FILES.items():
        baseline.parent.mkdir(parents=True, exist_ok=True)
        baseline.write_bytes(current.read_bytes())
        print(f"synced {baseline.relative_to(REPO_ROOT)}")


def check_wiring() -> str | None:
    problems = []
    pc = (REPO_ROOT / "openpilot/system/manager/process_config.py").read_text()
    if "openpilot.carrot.model_selector.modeld_runner" not in pc:
        problems.append("process_config.py: modeld no longer points at modeld_runner")
    mgr = (REPO_ROOT / "openpilot/system/manager/manager.py").read_text()
    if "model_selector.boot_compile" not in mgr:
        problems.append("manager.py: boot_compile hook removed")
    keys = (REPO_ROOT / "openpilot/common/params_keys.h").read_text()
    for k in ("DrivingModelName", "PendingModelName"):
        if k not in keys:
            problems.append(f"params_keys.h: {k} unregistered")
    return "; ".join(problems) if problems else None


CHECKS = (
    ("script-paths", check_script_paths),
    ("helpers-hook", check_helpers_hook),
    ("compile-modeld-cli", check_compile_modeld_cli),
    ("metadata-naming", check_metadata_naming),
    ("pkl-format-pair", check_pkl_format_pair),
    ("tinygrad-pickle-compat", check_tinygrad_pickle_compat),
    ("sconscript-flags", check_sconscript_flags),
    ("compile-env-guards", check_compile_env_guards),
    ("fill-model-msg-signatures", check_fill_model_msg_signatures),
    ("model-constants", check_model_constants),
    ("wiring", check_wiring),
    ("modeld-mirror", check_modeld_mirror),
)


def main() -> int:
    if "--sync-baselines" in sys.argv:
        sync_baselines()
        print("快照刷新完成 — 请重新运行检查确认 PASS")
        return 0

    failed = 0
    for name, fn in CHECKS:
        try:
            detail = fn()
        except Exception as e:  # 检查自身的错误也按契约违约处理
            detail = f"check errored: {e!r}"
        if detail is None:
            print(f"PASS {name}")
        else:
            failed += 1
            print(f"FAIL {name} — {detail}")

    from carrot.model_selector.config import compile_env_tag
    print(f"INFO compile_env_tag = {compile_env_tag()}")

    if failed:
        print(f"\n{failed} 项契约违约 — 需要让选择器代码随之适配（见 carrot/model_selector/README.md）")
        return 1
    print("\n全部契约 PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
