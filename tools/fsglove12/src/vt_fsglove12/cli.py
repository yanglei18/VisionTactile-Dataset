"""Offline tools and an explicitly connected standalone live application."""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import json
from pathlib import Path
import sys
import time

from .config import check_proposed_mount, digest, file_digest, layout_report, load_config, validate_config, write_json


def _json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _report(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2))


def _positive_int(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return result


def _parser():
    parser = argparse.ArgumentParser(
        prog="vt-fsglove12", description="官方 FSGlove 数值核心 / 12 路 IMU 适配（upstream-live 手动连接）")
    sub = parser.add_subparsers(dest="command", required=True)
    layout = sub.add_parser("layout", help="查看传感器→骨段映射及缺测部分")
    layout.add_argument("--config")
    inspect = sub.add_parser("inspect-input", help="只读检查12路输入单位、编码边界与接收频率；不连接硬件")
    inspect.add_argument("--config")
    inspect.add_argument("--raw", help="可选已有 raw.csv；省略时只展示输入配置契约")
    inspect.add_argument("--output", help="另存 JSON 报告，不覆盖已有文件")
    inspect.add_argument("--max-samples", type=_positive_int, help="限制读取前N条样本；适合仍在写入的录制")
    readiness = sub.add_parser("input-readiness", help="只读列出12路单位/时间契约与缺失证据；不判输入无效、不连接硬件")
    readiness_source = readiness.add_mutually_exclusive_group(required=True)
    readiness_source.add_argument("--config", help="明确指定配置 JSON；报告只输出到 stdout")
    readiness_source.add_argument("--session", help="已有会话目录；仅读取小型 config/metadata/contract JSON")
    units = sub.add_parser("fit-input-units", help="用静止端点与中间陀螺积分逐路核对换算系数；只读已有录制")
    units.add_argument("--raw", required=True)
    units.add_argument("--config", required=True, help="原会话配置，不能用默认配置猜历史单位")
    unit_windows = units.add_mutually_exclusive_group(required=True)
    unit_windows.add_argument("--windows", help="JSON 数组：至少三个已标注静止窗口，含 start_s/end_s/label")
    unit_windows.add_argument("--events", help="官方界面 events.jsonl；提取已完成三姿势采集的静止段")
    units.add_argument("--bias-window", help="可选 JSON 静止偏置窗口；不能把缓慢转动当零偏")
    units.add_argument("--report", required=True, help="新的逐路证据报告文件")
    units.add_argument("--output-config", help="另存候选配置，不改原配置；新融合结果须重新拟合安装标定")
    units.add_argument("--max-samples", type=_positive_int)
    check = sub.add_parser("doctor", help="检查可选 MANO 依赖与模型，不安装或下载")
    check.add_argument("--mano-assets")
    replay = sub.add_parser("replay", help="已有 raw.csv → 官方 VQF 六轴四元数")
    replay.add_argument("--raw", required=True)
    replay.add_argument("--config")
    replay.add_argument("--output", required=True)
    replay.add_argument("--report")
    replay.add_argument("--max-frames", type=_positive_int)
    rest = sub.add_parser("export-rest", help="从用户提供的真实 MANO 模型导出骨架参考帧")
    rest.add_argument("--mano-assets", required=True)
    rest.add_argument("--output", required=True)
    calibration = sub.add_parser("calibrate", help="显式标注姿态区间 → 本次回放专用安装/坐标标定")
    calibration.add_argument("--orientations", required=True)
    calibration.add_argument("--poses", required=True)
    calibration.add_argument("--rest", required=True)
    calibration.add_argument("--config")
    calibration.add_argument("--output", required=True)
    solve = sub.add_parser("solve", help="经标定的稀疏姿态 → 完整骨架角度与实测/推断标记")
    solve.add_argument("--orientations", required=True)
    solve.add_argument("--profile", required=True)
    solve.add_argument("--rest", required=True)
    solve.add_argument("--config")
    solve.add_argument("--output", required=True)
    solve.add_argument("--report")
    solve.add_argument("--max-frames", type=_positive_int)
    view = sub.add_parser("view", help="使用真实 MANO 网格播放 solve 结果（需要可选依赖）")
    view.add_argument("--poses", required=True)
    view.add_argument("--mano-assets", required=True)
    view.add_argument("--speed", type=float, default=1.0)
    live = sub.add_parser("live", help="旧实验算法窗口（非官方核心）；官方移植请用 upstream-live")
    live.add_argument("--mano-assets", required=True)
    live.add_argument("--config")
    live.add_argument("--port", default="", help="仅预填串口路径，不自动连接")
    live.add_argument("--output-root", default="tools/fsglove12/output/live")
    live.add_argument("--replay-raw", help="用已有 raw.csv 按时间回放，不连接串口")
    live.add_argument("--replay-speed", type=float, default=1.)
    official = sub.add_parser("upstream-live", help="官方 FSGlove 核心 + 12 路适配；不使用旧求解器")
    official.add_argument("--fsglove-source", required=True, help="可信的官方固定版本源码 checkout")
    official.add_argument("--mano-assets", required=True)
    official.add_argument("--config")
    official.add_argument("--output-rate-hz", type=float,
                          help="本次会话的解算目标频率（如 120）；不能高于输入名义频率，不代表性能保证")
    official.add_argument("--port", default="")
    official.add_argument("--output-root", default="tools/fsglove12/output/upstream_live")
    official.add_argument("--replay-raw")
    official.add_argument("--replay-speed", type=float, default=1.)
    official.add_argument("--shape-profile", help="upstream-shape 生成的形状参数（可选）")
    official.add_argument("--ros-socket", help="可选 ROS 2 结果桥接的本地 socket 绝对路径；不发布原始 IMU、不自动连接")
    official.add_argument("--tactile-socket", help="可选 ROS 触觉只读显示桥接 socket；在手模型上显示 ΔADC，不改变标定或解算")
    official.add_argument("--tactile-layout", help="显式触觉皮肤映射 JSON；左手 3D 叠加必须提供左手映射")
    check_upstream = sub.add_parser("upstream-check", help="加载并报告实际官方数值核心；不启动窗口或串口")
    check_upstream.add_argument("--fsglove-source", required=True)
    check_upstream.add_argument("--mano-assets", required=True)
    shape = sub.add_parser("upstream-shape", help="调用官方可微 MANO 捏合手形优化；需真实接触标签")
    shape.add_argument("--fsglove-source", required=True)
    shape.add_argument("--mano-assets", required=True)
    shape.add_argument("--contacts", required=True, help="JSON: mano_pose_rotvec[N,16,3], finger_category[N] (2..5)")
    shape.add_argument("--iterations", type=_positive_int, default=500)
    shape.add_argument("--seed", type=int, default=0)
    shape.add_argument("--output", required=True)
    for command in (layout, inspect, units, check, replay, rest, calibration, solve, view,
                    live, official, check_upstream, shape):
        command.add_argument("--hand", choices=("right", "left"), default=None,
                             help="手别；有配置时从配置读取，否则默认 right；显式冲突报错")
    return parser


def _check_new_paths(*paths):
    targets = [Path(p).expanduser().resolve() for p in paths if p is not None]
    if len(set(targets)) != len(targets):
        raise ValueError("output/report paths must be distinct")
    for target in targets:
        if target.exists():
            raise FileExistsError(f"Refusing to replace existing file: {target}")
        if not target.parent.is_dir():
            raise ValueError(f"Output directory does not exist: {target.parent}")


def _solve(args):
    from .calibration import apply_calibration, check_profile_binding, validate_rest
    from .functional_axes import apply_functional_axes
    from .replay import iter_orientation_frames
    from .solver import SparseHandSolver
    config, rest, profile = load_config(args.config, hand=args.hand), _json(args.rest), _json(args.profile)
    check_proposed_mount(config)
    rest_rotations = validate_rest(rest)
    check_profile_binding(profile, config, rest, file_digest(args.orientations))
    _check_new_paths(args.output, args.report)
    solver = SparseHandSolver(config, rest_rotations)
    count, unavailable, unconverged, rejections = 0, 0, 0, {}
    started = time.perf_counter()
    with Path(args.output).open("x", encoding="utf-8") as output:
        for frame in iter_orientation_frames(args.orientations):
            observations, rejected = apply_calibration(frame, profile)
            observations, functional = apply_functional_axes(observations, profile, rest)
            pose = solver.solve(observations, frame["timestamp_s"])
            pose["functional_axis_retargeting"] = functional
            pose.update(schema_version=1, frame_index=count, model_rest_sha256=digest(rest),
                        config_sha256=digest(config), calibration_sha256=digest(profile),
                        calibration_status=profile["status"], calibration_rejected_nodes=rejected,
                        upstream_fsglove_solver_executed=False,
                        model_provenance=rest.get("provenance", {}))
            output.write(json.dumps(pose, allow_nan=False, separators=(",", ":")) + "\n")
            count += 1
            unavailable += int(pose["quality"] == "unavailable")
            unconverged += int(pose["quality"] == "experimental_not_converged")
            for node in rejected:
                rejections[node] = rejections.get(node, 0) + 1
            if args.max_frames and count >= args.max_frames:
                break
    report = {"status": "completed", "kind": "experimental_sparse_pose_replay", "frames": count,
              "unavailable_frames": unavailable, "not_converged_frames": unconverged,
              "calibration_rejected_frames_by_node": rejections,
              "elapsed_s": time.perf_counter() - started, "output": str(Path(args.output).resolve()),
              "physical_accuracy_verified": False, "relative_heading_drift_corrected": False,
              "upstream_fsglove_solver_executed": False,
              "note": "New sparse adapter using the FSGlove/MANO layout, not an unmodified DiffHCal run."}
    if args.report:
        write_json(args.report, report)
    return report


def main(argv=None):
    args = _parser().parse_args(argv)
    try:
        if args.command == "layout":
            _report(layout_report(load_config(args.config, hand=args.hand)))
        elif args.command == "input-readiness":
            from .input_readiness import input_readiness_from_files
            _report(input_readiness_from_files(config_path=args.config, session_directory=args.session))
        elif args.command == "inspect-input":
            from .input_contract import describe_input_contract
            from .input_diagnostics import inspect_raw_input
            _check_new_paths(args.output)
            if args.max_samples is not None and args.raw is None:
                raise ValueError("--max-samples requires --raw")
            config = load_config(args.config, hand=args.hand)
            result = (inspect_raw_input(args.raw, config, max_samples=args.max_samples) if args.raw else
                      dict(input_contract=describe_input_contract(config), hardware_accessed=False))
            if args.output:
                write_json(args.output, result)
            _report(result)
        elif args.command == "fit-input-units":
            from .input_unit_fit import fit_input_units
            from .input_unit_workflow import stationary_windows_from_events
            _check_new_paths(args.report, args.output_config)
            window_source = None
            if args.events:
                windows, window_source = stationary_windows_from_events(args.events)
            else:
                windows = _json(args.windows)
            result = fit_input_units(args.raw, load_config(args.config, hand=args.hand), windows,
                                     max_samples=args.max_samples,
                                     bias_window=_json(args.bias_window) if args.bias_window else None)
            if window_source is not None:
                result["stationary_window_source"] = window_source
            write_json(args.report, result)
            if args.output_config:
                write_json(args.output_config, result["candidate_config"])
            _report({"report": str(Path(args.report).resolve()),
                     "output_config": str(Path(args.output_config).resolve()) if args.output_config else None,
                     "recommended_nodes": result["recommended_nodes"],
                     "nominal_supported_nodes": result["nominal_supported_nodes"],
                     "rejected_nodes": result["rejected_nodes"], "hardware_accessed": False,
                     "note": "候选是逐节点有效换算系数，不是寄存器读回；不修改运行中的程序或旧标定。"})
        elif args.command == "doctor":
            from .mano import doctor
            result = doctor(args.mano_assets, hand=args.hand or "right")
            _report(result)
            return 0 if result["ok"] else 2
        elif args.command == "replay":
            from .replay import replay_raw
            _check_new_paths(args.output, args.report)
            result = replay_raw(args.raw, load_config(args.config, hand=args.hand), args.output, max_frames=args.max_frames)
            result.update(config_sha256=digest(load_config(args.config, hand=args.hand)), raw_sha256=file_digest(args.raw),
                          output_sha256=file_digest(args.output))
            if args.report:
                write_json(args.report, result)
            _report(result)
        elif args.command == "export-rest":
            from .mano import ManoBackend
            _check_new_paths(args.output)
            rest = ManoBackend(args.mano_assets, hand=args.hand or "right").export_rest()
            write_json(args.output, rest)
            _report({"status": "completed", "output": args.output, "rest_sha256": digest(rest),
                     "model_provenance": rest["provenance"]})
        elif args.command == "calibrate":
            from .calibration import calibrate, validate_rest
            from .replay import iter_orientation_frames
            _check_new_paths(args.output)
            rest = _json(args.rest)
            validate_rest(rest)
            profile = calibrate(iter_orientation_frames(args.orientations), _json(args.poses), rest,
                                load_config(args.config, hand=args.hand), stream_sha256=file_digest(args.orientations))
            write_json(args.output, profile)
            _report({"status": profile["status"], "output": args.output,
                     "calibrated_nodes": sorted(profile["nodes"], key=int),
                     "rejected_nodes": profile["rejected_nodes"], "warnings": profile["warnings"]})
        elif args.command == "solve":
            _report(_solve(args))
        elif args.command == "view":
            from .viewer import run_viewer
            run_viewer(args.poses, args.mano_assets, speed=args.speed, hand=args.hand or "right")
        elif args.command == "live":
            from .live_ui import run_live
            return run_live(load_config(args.config, hand=args.hand), args.mano_assets, output_root=args.output_root,
                            port=args.port, replay_raw=args.replay_raw, replay_speed=args.replay_speed)
        elif args.command == "upstream-check":
            from .upstream_core import load_upstream_core
            core = load_upstream_core(args.fsglove_source, args.mano_assets)
            if args.hand is not None:
                core.require_model(args.hand)
            _report({"status": "loaded", "upstream": core.provenance, "hardware_accessed": False,
                     "paper_complete_reproduction": False})
        elif args.command == "upstream-live":
            from .upstream_core import load_upstream_core
            from .live_ui import run_live
            config = load_config(args.config, hand=args.hand)
            core = load_upstream_core(args.fsglove_source, args.mano_assets)
            if args.output_rate_hz is not None:
                config["output_rate_hz"] = args.output_rate_hz
                config = validate_config(config)
            config["pose_backend"] = "official_fsglove_sparse_adapter"
            config["upstream_provenance"] = core.provenance
            if args.shape_profile:
                shape = _json(args.shape_profile)
                if (shape.get("algorithm") != "upstream_fsglove_touch_shape"
                        or shape.get("shape_calibrated") is not True):
                    raise ValueError("shape profile is not an upstream-shape result")
                provenance = shape.get("upstream", {})
                if (provenance.get("upstream_commit") != core.provenance["upstream_commit"]
                        or any(provenance.get("sources", {}).get(name, {}).get("sha256") != entry["sha256"]
                               for name, entry in core.provenance["sources"].items())):
                    raise ValueError("shape profile uses a different upstream implementation")
                if shape.get("mano_model") is not None or config["hand"] == "left":
                    from .shape_calibration import model_provenance, validate_shape_profile
                    model = model_provenance(args.mano_assets, hand=config["hand"])
                    shape = validate_shape_profile(shape, {**core.provenance, "mano_model": model})
                import numpy as np
                params = np.asarray(shape["shape_params"], dtype=float)
                if params.shape != (10,) or not np.isfinite(params).all():
                    raise ValueError("shape_params must contain 10 finite MANO beta values")
                config["upstream_shape_params"] = shape["shape_params"]
                config["upstream_shape_calibrated"] = True
                config["upstream_shape_provenance"] = {
                    "profile_sha256": file_digest(args.shape_profile),
                    "upstream": provenance, "algorithm": shape["algorithm"],
                    "source_sha256": shape.get("source_sha256"),
                    "physical_accuracy_verified": False,
                }
            return run_live(config, args.mano_assets, output_root=args.output_root, port=args.port,
                            replay_raw=args.replay_raw, replay_speed=args.replay_speed,
                            upstream_checkout=args.fsglove_source,
                            **({"ros_socket": args.ros_socket} if args.ros_socket else {}),
                            **({"tactile_socket": args.tactile_socket} if args.tactile_socket else {}),
                            **({"tactile_layout": args.tactile_layout} if args.tactile_layout else {}))
        elif args.command == "upstream-shape":
            from .upstream_core import load_upstream_core
            from .shape_calibration import model_provenance, validate_shape_profile
            import numpy as np
            import torch
            _check_new_paths(args.output)
            contacts = _json(args.contacts)
            poses = np.asarray(contacts["mano_pose_rotvec"], dtype=float)
            labels = np.asarray(contacts["finger_category"])
            if (poses.ndim != 3 or poses.shape[1:] != (16, 3) or not np.isfinite(poses).all()
                    or labels.shape != (poses.shape[0],) or not np.isin(labels, [2, 3, 4, 5]).all()
                    or set(labels.tolist()) != {2, 3, 4, 5}):
                raise ValueError("contacts must contain finite [N,16,3] MANO poses and all four contact categories 2..5")
            if contacts.get("contact_labels_confirmed") is not True:
                raise ValueError("contact_labels_confirmed=true requires explicitly observed fingertip contact labels")
            core = load_upstream_core(args.fsglove_source, args.mano_assets)
            torch.set_num_threads(1)
            contact_model = contacts.get("mano_model", {})
            if not isinstance(contact_model, dict):
                raise ValueError("contacts require valid MANO model provenance")
            hand = args.hand or contact_model.get("hand", "right")
            if contact_model.get("hand", hand) != hand:
                raise ValueError("contact MANO model hand conflicts with --hand")
            model = model_provenance(args.mano_assets, hand=hand)
            if contacts.get("mano_model") is not None and contacts["mano_model"].get("sha256") != model["sha256"]:
                raise ValueError("contacts use a different MANO model")
            # Keep original optimizer diagnostics, without corrupting stdout's
            # JSON interface. The numerical function itself is unchanged.
            with redirect_stdout(sys.stderr):
                betas = core.optimize_shape(torch.tensor(poses, dtype=torch.float32),
                                            torch.tensor(labels, dtype=torch.long), hand,
                                            iter_num=args.iterations, seed=args.seed)
            betas = np.asarray(betas, dtype=float)
            if betas.shape != (10,) or not np.isfinite(betas).all():
                raise ValueError("official shape optimizer returned invalid MANO beta values")
            result = dict(algorithm="upstream_fsglove_touch_shape", shape_params=betas.tolist(),
                          shape_calibrated=True, mano_model=model,
                          upstream=core.provenance, source_sha256=file_digest(args.contacts),
                          iterations=args.iterations, seed=args.seed, physical_accuracy_verified=False,
                          contact_labels_source="user_explicit_not_inferred_from_imu")
            result = validate_shape_profile(result, {**core.provenance, "mano_model": model})
            write_json(args.output, result)
            _report(result)
        return 0
    except (ValueError, KeyError, TypeError, OSError, ImportError, RuntimeError) as exc:
        print(f"vt-fsglove12: {exc}", file=sys.stderr)
        return 2
