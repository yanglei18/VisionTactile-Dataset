"""Live storage-plugin measurements, independent of preview and sensor refresh."""
from ..i18n import tr


def recorder_readout(snapshot):
    metrics = snapshot.get("recorder_metrics")
    if metrics is None:
        return ((tr("正在等待本次录制的写入统计"), "") if snapshot["recording"]["state"] == "RECORDING"
                else ("", ""))
    rate = metrics["storage_mb_s"]
    count = sum(row["written"] for row in metrics["topics"])
    summary = (tr("已写入 {n} 条 · 速率待测", n=count) if rate is None else
               tr("已写入 {n} 条 · {rate:.2f} MB/s", n=count, rate=rate))
    if metrics["writer_errors"] or metrics["storage_errors"]:
        summary = tr("{summary} · 录制器错误 {writer} · 存储错误 {storage}", summary=summary,
                     writer=metrics["writer_errors"], storage=metrics["storage_errors"])
    lines = [tr("每路写入 Hz 为相邻两次存储计数的差值。结束后按文件复核。")]
    for row in metrics["topics"]:
        hz = row["written_hz"]
        rate_text = "—" if hz is None else f"{hz:.1f} Hz"
        lines.append(tr("{topic} · 收到 {received} · 写入 {written} · {hz}",
                        topic=row["topic"], received=row["received"], written=row["written"], hz=rate_text))
    return tr("{summary} · 悬停查看各路 Hz", summary=summary), "\n".join(map(str, lines))
