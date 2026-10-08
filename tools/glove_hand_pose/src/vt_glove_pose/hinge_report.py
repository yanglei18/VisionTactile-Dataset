"""Self-contained offline comparison plots, without browsers or network access."""
from __future__ import annotations

from html import escape
import json

import numpy as np


def _thin_peaks(values, limit=1400):
    """Preserve endpoints and each bucket's min/max, not just every kth point."""
    values = np.asarray(values, dtype=float)
    if len(values) <= limit:
        return np.arange(len(values))
    indices = {0, len(values)-1}
    for block in np.array_split(np.arange(len(values)), max(1, (limit-2)//2)):
        indices.add(int(block[np.argmin(values[block])]))
        indices.add(int(block[np.argmax(values[block])]))
    return np.array(sorted(indices))


def _plot(times, lines, *, title, unit="角度 / °"):
    times = np.asarray(times, dtype=float)
    if times.ndim != 1 or not 1 <= len(times) <= 60000 or not np.isfinite(times).all():
        raise ValueError("comparison plot requires finite 1D time samples")
    if np.any(np.diff(times) < 0):
        raise ValueError("comparison plot timestamps must not go backward")
    vectors = [np.asarray(values, dtype=float) for _, _, values in lines]
    if any(v.shape != times.shape or not np.isfinite(v).all() for v in vectors):
        raise ValueError("plot angle arrays must be finite and match the time samples")
    low, high = min(v.min() for v in vectors), max(v.max() for v in vectors)
    padding = max(2., .08*(high-low))
    low, high = low-padding, high+padding
    span = max(1e-9, times[-1]-times[0])
    width, height = 900, 270
    left, right, top, bottom = 58, 16, 26, 36
    plotw, ploth = width-left-right, height-top-bottom
    svg = [f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{escape(title, quote=True)}">',
           f'<title>{escape(title)}</title>']
    for fraction in np.linspace(0, 1, 5):
        x = left+fraction*plotw
        y = top+fraction*ploth
        svg.extend([f'<line x1="{left}" y1="{y:.2f}" x2="{left+plotw}" y2="{y:.2f}" stroke="#e1e5e9"/>',
                    f'<text x="{left-6}" y="{y+4:.2f}" text-anchor="end">{high-fraction*(high-low):.1f}</text>',
                    f'<text x="{x:.2f}" y="{height-14}" text-anchor="middle">{times[0]+fraction*span:.1f}</text>'])
    svg.append(f'<text x="{left}" y="16">{escape(unit)}</text>')
    for (label, color, _), values in zip(lines, vectors):
        selected = _thin_peaks(values)
        points = ' '.join(f'{left+(times[i]-times[0])/span*plotw:.2f},{top+(high-values[i])/(high-low)*ploth:.2f}' for i in selected)
        svg.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.6" points="{points}"><title>{escape(label)}</title></polyline>')
    svg.append('</svg>')
    legend = ' '.join(f'<span style="color:{color}">● {escape(label)}</span>' for label, color, _ in lines)
    return f'<h3>{escape(title)}</h3><div class="legend">{legend}</div>{"".join(svg)}<p class="small">横轴：名义样本时间 / 秒。绘图保留分桶峰值；统计使用全部样本。</p>'


def _section(title, description, times, baseline, constrained, metrics, truth=None):
    lines = [("旧观察器", "#147aab", baseline), ("约束候选", "#df681e", constrained)]
    if truth is not None:
        lines.append(("合成解析参考（非真机）", "#228247", truth))
    output = [f'<section><h2>{escape(title)}</h2><p>{escape(description)}</p>']
    output.append(_plot(times, lines, title="相对角度对照"))
    if truth is not None:
        truth = np.asarray(truth)
        output.append(_plot(times, [("旧观察器误差", "#147aab", np.asarray(baseline)-truth),
                                   ("约束候选误差", "#df681e", np.asarray(constrained)-truth)], title="对解析参考的角度误差"))
    else:
        output.append(_plot(times, [("候选减旧算法", "#78509b", np.asarray(constrained)-np.asarray(baseline))],
                            title="算法差值（不是真实角度误差）"))
    output.append('<details><summary>数值与状态统计</summary><pre>'+escape(json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False))+'</pre></details></section>')
    return ''.join(output)


def render_comparison_html(report):
    """Render observer comparisons or benchmark reports with embedded series."""
    kind = report.get("kind")
    sections = []
    if kind == "vt_glove_pose.synthetic_observer_benchmark":
        title = "IMU 关节观察器：合成压力测试"
        notice = "这是理想铰链模型上的合成测试，不是真机精度，也不是产品验收。图中低误差不能证明手套已经不漂移。"
        for name, item in report["cases"].items():
            traces = item["observers"]
            if "series" not in traces["baseline"] or "series" not in traces["constrained"]:
                raise ValueError("benchmark HTML requires --include-series")
            metrics = {key: {k: v for k, v in value.items() if k != "series"} for key, value in traces.items()}
            sections.append(_section(name, item["description"], item["time_s"],
                                     [s["angle_deg"] for s in traces["baseline"]["series"]],
                                     [s["angle_deg"] for s in traces["constrained"]["series"]], metrics,
                                     truth=item["truth_angle_deg"]))
    elif kind == "vt_glove_pose.offline_hinge_comparison":
        title = "IMU 关节观察器：离线数据对照"
        notice = "没有独立真实角度参考。这里展示算法输出差异，不能根据更平滑、更接近零或更一致判定更准确。"
        a, b = report["baseline"], report["constrained"]
        if a["replay_status"] == b["replay_status"] == "completed":
            sections.append(_section("同一标定、同一输入", str(report["source"].get("source", "")),
                                     [s["nominal_time_s"] for s in a["series"]],
                                     [s["angle_deg"] for s in a["series"]],
                                     [s["angle_deg"] for s in b["series"]],
                                     {"baseline": a["summary"], "constrained": b["summary"],
                                      "disagreement_not_accuracy": report["disagreement"]}))
        else:
            sections.append('<p>未生成两条可对照轨迹：</p><pre>'+escape(json.dumps(
                {"baseline": a["replay_status"], "constrained": b["replay_status"]}, ensure_ascii=False, indent=2))+'</pre>')
    else:
        raise ValueError("unsupported hinge comparison report")
    return f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title>
<style>body{{font:15px system-ui,sans-serif;color:#263540;background:#f6f8fa;max-width:1050px;margin:24px auto;padding:0 18px}}section{{background:white;padding:18px;margin:22px 0;border:1px solid #dce3e8;border-radius:8px}}svg{{width:100%;height:auto}}svg text{{font-size:12px;fill:#50616e}}.notice{{background:#fff0d8;padding:14px;border-left:4px solid #cc8425}}.small{{font-size:12px;color:#61727e}}.legend span{{margin-right:18px}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}}</style></head>
<body><h1>{title}</h1><p class="notice">{notice}</p>
<p>离线候选；不修改实时配置。无外部脚本、字体或网络请求。模型不确定度不是实测精度保证。</p>
{''.join(sections)}</body></html>'''
