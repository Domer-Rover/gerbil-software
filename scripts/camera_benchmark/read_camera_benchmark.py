#!/usr/bin/env python3
"""Turn a camera benchmark CSV into a readable Markdown report."""

import argparse
import csv
from pathlib import Path


def value(row, key, suffix="", digits=1):
    raw = row.get(key, "")
    if raw is None or raw.strip() == "":
        return "—"
    try:
        number = float(raw)
        shown = f"{number:.{digits}f}" if digits is not None else str(int(number))
        return f"{shown}{suffix}"
    except ValueError:
        return raw


def make_report(source, rows):
    title = f"# Camera benchmark results: {source.name}"
    lines = [
        title,
        "",
        f"Profiles measured: **{len(rows)}**",
        "",
        "CPU columns show the mean and approximate 95th percentile across CPU core samples. "
        "GPU and RAM columns show mean and approximate 95th percentile for the whole Jetson.",
        "",
        "| Profile | Capture | Grab FPS | Published | Target FPS | Measured Hz | CPU mean / p95 | GPU mean / p95 | RAM mean / p95 | Samples |",
        "|---|---:|---:|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        published = row.get("pub_resolution", "") or "—"
        if published == "CUSTOM":
            published += f" ÷{value(row, 'downscale', digits=1)}"
        cpu = f"{value(row, 'cpu_core_mean_pct', '%')} / {value(row, 'cpu_core_p95_pct', '%')}"
        gpu = f"{value(row, 'gpu_mean_pct', '%')} / {value(row, 'gpu_p95_pct', '%')}"
        ram = f"{value(row, 'ram_mean_pct', '%')} / {value(row, 'ram_p95_pct', '%')}"
        lines.append(
            f"| {row.get('name', '—')} | {row.get('grab_resolution', '—')} | "
            f"{value(row, 'grab_fps', digits=0)} | {published} | "
            f"{value(row, 'pub_fps', digits=0)} | {value(row, 'measured_image_hz', ' Hz', digits=2)} | "
            f"{cpu} | {gpu} | {ram} | {value(row, 'samples', digits=0)} |"
        )

    no_rate = [r.get("name", "unnamed") for r in rows if not (r.get("measured_image_hz") or "").strip()]
    failed_launches = [r.get("name", "unnamed") for r in rows if r.get("launch_log") and Path(r["launch_log"]).exists() and "process has died" in Path(r["launch_log"]).read_text(errors="replace")]
    lines.extend(["", "## Review notes", ""])
    if no_rate:
        lines.append("No image rate was recorded for: " + ", ".join(f"`{n}`" for n in no_rate) + ". Check the configured image topic and each trial's launch log.")
    else:
        lines.append("An image rate was recorded for every profile. Compare it with the target FPS when evaluating whether the stream kept up.")
    if failed_launches:
        lines.append("Possible camera launch failures were found for: " + ", ".join(f"`{n}`" for n in failed_launches) + ". Inspect their launch logs.")
    lines.extend([
        "",
        "These are whole-device readings under the workload that was running during the test. "
        "Compare profiles only when the background workload and Nano power mode were consistent.",
        "",
        "## Trial logs",
        "",
    ])
    for row in rows:
        name = row.get("name", "unnamed")
        logs = []
        for label, key in (("launch", "launch_log"), ("tegrastats", "tegrastats_log")):
            path = row.get(key, "")
            if path:
                logs.append(f"{label}: `{path}`")
        lines.append(f"- **{name}:** " + (" · ".join(logs) if logs else "no log paths in CSV"))
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path, help="camera-benchmark-*.csv produced by the benchmark")
    parser.add_argument("-o", "--output", type=Path, help="report path (default: same name with .md)")
    args = parser.parse_args()
    if not args.csv_file.is_file():
        parser.error(f"CSV file not found: {args.csv_file}")
    output = args.output or args.csv_file.with_suffix(".md")
    with args.csv_file.open(newline="") as f:
        rows = list(csv.DictReader(f))
    output.write_text(make_report(args.csv_file, rows))
    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
