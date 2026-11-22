"""Replay recorded sensor streams and quantify errors without importing MuJoCo."""

import argparse
import json
import time
from pathlib import Path

import numpy as np


def read_recording(folder):
    config = json.loads((folder / "config.json").read_text())
    if config["schema_version"] != 1:
        raise ValueError(f"Unsupported recording schema {config['schema_version']}.")
    channels = [key for sensor in config["channels"] for key in sensor["columns"]]
    if not channels or len(channels) != len(set(channels)):
        raise ValueError("Empty or duplicate channel names.")
    clocks = []
    values = []
    for name in ("truth", "raw", "measurements"):
        file = folder / f"{name}.csv"
        columns = np.genfromtxt(file, delimiter=",", names=True, ndmin=1)
        expected = ("time_s", *channels)
        if columns.dtype.names[:len(expected)] != expected:
            raise ValueError(f"Unexpected columns in {file.name}.")
        clocks.append(columns["time_s"])
        values.append(np.column_stack([columns[key] for key in channels]))
    stamps = clocks[0]
    if any(not np.array_equal(stamps, clock) for clock in clocks[1:]):
        raise ValueError("Timestamp mismatch between truth, raw and measurement streams.")
    if len(stamps) != config["sample_count"] or not np.all(np.isfinite(stamps)):
        raise ValueError("Invalid timestamps or manifest sample count.")
    if not len(stamps) or np.any(np.diff(stamps) <= 0) or not np.allclose(
        stamps, np.arange(len(stamps)) * config["sample_period_s"], rtol=0, atol=1e-10
    ) or not np.isclose(stamps[-1], config["duration_s"], rtol=0, atol=1e-10):
        raise ValueError("Timestamps are not in the configured simulation timeline.")
    if any(not np.all(np.isfinite(stream)) for stream in values):
        raise ValueError("Nonfinite sensor samples.")
    return config, stamps, values


def stats(error):
    return {"bias": float(np.mean(error)), "std": float(np.std(error)),
            "rmse": float(np.sqrt(np.mean(error**2))), "max_abs": float(np.max(abs(error)))}


def analyze(config, stamps, truth, raw, measured):
    errors = {}
    for sensor in config["channels"]:
        for key in sensor["columns"]:
            i = len(errors)
            errors[key] = {
                "unit": sensor["unit"], "raw_minus_truth": stats(raw[:, i] - truth[:, i]),
                "measurement_minus_truth": stats(measured[:, i] - truth[:, i]),
                "measurement_minus_raw": stats(measured[:, i] - raw[:, i]),
            }
    return {"schema_version": 1, "case": config["case"], "samples": len(stamps),
            "window_s": [float(stamps[0]), float(stamps[-1])], "channels": errors}


def plot_errors(path, config, stamps, truth, measured):
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    sensors = config["channels"]
    fig, axes = plt.subplots(len(sensors), 2, figsize=(10, 8), layout="constrained")
    offset = 0
    for sensor, (signal, residual) in zip(sensors, axes):
        for j, channel in enumerate(sensor["columns"]):
            i = offset + j
            axis = channel.rsplit("_", 1)[-1].upper()
            trace, = signal.plot(stamps, measured[:, i], lw=0.8, alpha=0.7, label=axis)
            signal.plot(stamps, truth[:, i], "--", lw=1.1, color=trace.get_color())
            residual.plot(stamps, measured[:, i] - truth[:, i], lw=0.75,
                          color=trace.get_color())
        name = sensor["type"].capitalize()
        signal.set_ylabel(f"{name} ({sensor['unit']})")
        residual.set_ylabel(f"Error ({sensor['unit']})")
        signal.legend(loc="best", ncols=3, fontsize=8)
        for ax in (signal, residual):
            ax.grid(alpha=0.25)
        offset += len(sensor["columns"])
    axes[0, 0].set_title("Measurement and reference (dashed)")
    axes[0, 1].set_title("Measurement minus reference")
    for ax in axes[-1]:
        ax.set_xlabel("Simulation time (s)")
    fig.suptitle(f"Aether Vehicle-V1 · {config['case'].capitalize()} sensor replay")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def stream_samples(stamps, channels, truth, raw, measured, rate):
    clock_start = time.perf_counter()
    for stamp, expected, native, observed in zip(stamps, truth, raw, measured):
        if rate:
            delay = clock_start + (stamp - stamps[0]) / rate - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
        print(json.dumps({"time_s": float(stamp), "truth": dict(zip(channels, expected.tolist())),
                          "raw": dict(zip(channels, native.tolist())),
                          "measurement": dict(zip(channels, observed.tolist()))}), flush=True)


def replay(folder, output, start=None, end=None, plots=True, stream_rate=None):
    config, stamps, values = read_recording(folder)
    if ((start is not None and not np.isfinite(start))
            or (end is not None and not np.isfinite(end))):
        raise ValueError("Replay time window must be finite.")
    selected = np.ones(len(stamps), dtype=bool)
    if start is not None:
        selected &= stamps >= start - 1e-10
    if end is not None:
        selected &= stamps <= end + 1e-10
    if not np.any(selected):
        raise ValueError("The requested replay window has no samples.")
    stamps = stamps[selected]
    truth, raw, measured = [array[selected] for array in values]
    summary = analyze(config, stamps, truth, raw, measured)
    channels = list(summary["channels"])
    output.mkdir(parents=True, exist_ok=False)
    header = ["time_s", *[f"{kind}_{key}" for kind in ("raw_error", "measurement_error", "noise")
                          for key in channels]]
    np.savetxt(output / "errors.csv", np.column_stack([
        stamps, raw - truth, measured - truth, measured - raw,
    ]), delimiter=",", header=",".join(header), comments="", fmt="%.17g")
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    if plots:
        plot_errors(output / "errors.png", config, stamps, truth, measured)
    if stream_rate is not None:
        stream_samples(stamps, channels, truth, raw, measured, stream_rate)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="Directory containing one recorded case")
    parser.add_argument("--output", type=Path, required=True, help="New analysis directory")
    parser.add_argument("--start", type=float, help="First simulation time, inclusive")
    parser.add_argument("--end", type=float, help="Last simulation time, inclusive")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--stream-rate", type=float,
                        help="Also replay JSON lines to stdout at a multiple of real time; 0 bursts")
    args = parser.parse_args()
    if args.stream_rate is not None and (
        not np.isfinite(args.stream_rate) or args.stream_rate < 0
    ):
        parser.error("--stream-rate must be finite and nonnegative.")
    report = replay(args.recording, args.output, args.start, args.end,
                    not args.no_plots, args.stream_rate)
    if args.stream_rate is None:
        print(f"Replayed {report['samples']} samples for {report['case']}.")


if __name__ == "__main__":
    main()
