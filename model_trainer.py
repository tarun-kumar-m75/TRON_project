"""
Combines all session CSVs in a data folder into one master dataset,
trains a distance REGRESSOR (continuous meters, not discrete classes),
evaluates with mean absolute error (MAE), and only overwrites the
deployed STM32 C header if this run's model beats the previous best -
so the STM32 never gets a worse model just because it's newer.

Install once:
    pip install pandas scikit-learn matplotlib micromlgen

Usage:
    python train_model.py --datadir data --outdir models
"""

import argparse
import glob
import json
import os

import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import mean_absolute_error, r2_score
import matplotlib.pyplot as plt
from micromlgen import port

FEATURE_COLS = ["rssi_mean", "rssi_std", "rssi_min", "rssi_max", "rssi_median"]
HISTORY_FILE = "model_history.json"
DEPLOYED_HEADER = "distance_regressor.h"


def load_master_dataset(datadir):
    """Combine every session_*.csv in datadir into one dataframe.
    Skips the '#'-prefixed metadata comment line each session file
    starts with (session id + notes), which pandas would otherwise
    try to parse as a data row."""
    paths = sorted(glob.glob(os.path.join(datadir, "*.csv")))
    if not paths:
        raise FileNotFoundError(f"No CSV files found in {datadir}")

    frames = []
    for p in paths:
        df = pd.read_csv(p, comment="#")
        df["source_file"] = os.path.basename(p)

        # Diagnostic: flag any non-numeric label values in THIS file
        # specifically, so you know which session/file had the corrupted
        # or malformed row, rather than just seeing it disappear later.
        bad_labels = df[pd.to_numeric(df["label_distance_m"], errors="coerce").isna()]
        if len(bad_labels) > 0:
            print(f"  WARNING: {os.path.basename(p)} has {len(bad_labels)} row(s) "
                  f"with a non-numeric label_distance_m value, e.g.: "
                  f"{bad_labels['label_distance_m'].unique()[:5].tolist()}")

        frames.append(df)
        print(f"  loaded {len(df):5d} rows from {os.path.basename(p)}")

    combined = pd.concat(frames, ignore_index=True)
    print(f"\nCombined master dataset: {len(combined)} rows from {len(paths)} session file(s)")
    return combined


def clean(df):
    # Force every relevant column to numeric. Any value that fails to
    # parse (corrupted serial bytes, a stray header/text line that slipped
    # through, a shifted column from a malformed row, etc.) becomes NaN
    # instead of silently turning the whole column into mixed-type
    # "object" dtype - which is what caused the sort_index() crash, since
    # you can't compare a float to a leftover string.
    numeric_cols = ["label_distance_m"] + FEATURE_COLS
    before_bad = len(df)
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    before = len(df)
    df = df.dropna(subset=FEATURE_COLS + ["label_distance_m"])
    n_dropped = before - len(df)
    if n_dropped > 0:
        print(f"Dropped {n_dropped} rows with missing/unparseable values "
              f"(empty windows or corrupted/malformed rows)")
    print(f"{len(df)} rows remain after cleaning")

    df = df[df["label_distance_m"] != -1.0]
    print(f"After removing unlabeled rows: {len(df)} rows")

    print("\nSamples per true distance:")
    print(df["label_distance_m"].value_counts().sort_index())
    return df


def load_history(outdir):
    path = os.path.join(outdir, HISTORY_FILE)
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return {"best_mae": None, "runs": []}


def save_history(outdir, history):
    path = os.path.join(outdir, HISTORY_FILE)
    with open(path, "w") as f:
        json.dump(history, f, indent=2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--datadir", required=True, help="Folder containing session_*.csv files")
    parser.add_argument("--outdir", default="models", help="Where to save model artifacts")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    print("Loading master dataset...")
    df = load_master_dataset(args.datadir)
    df = clean(df)

    X = df[FEATURE_COLS]
    y = df["label_distance_m"].astype(float)  # continuous target now, not a string class

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )
    print(f"\nTrain: {len(X_train)} rows | Test: {len(X_test)} rows")

    model = RandomForestRegressor(n_estimators=15, max_depth=5, random_state=42)
    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)
    print(f"\nTest MAE: {mae:.3f} m")
    print(f"Test R^2: {r2:.3f}")

    print("\nFeature importance:")
    for name, imp in zip(FEATURE_COLS, model.feature_importances_):
        print(f"  {name}: {imp:.3f}")

    # ---------------- Predicted vs actual scatter (replaces confusion matrix) ----
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(y_test, y_pred, alpha=0.4, s=20)
    lims = [min(y_test.min(), y_pred.min()) - 0.5, max(y_test.max(), y_pred.max()) + 0.5]
    ax.plot(lims, lims, 'r--', linewidth=1, label="perfect prediction")
    ax.set_xlabel("Actual distance (m)")
    ax.set_ylabel("Predicted distance (m)")
    ax.set_title(f"Predicted vs actual (MAE={mae:.2f}m, R2={r2:.2f})")
    ax.legend()
    plt.tight_layout()
    plot_path = os.path.join(args.outdir, "predicted_vs_actual.png")
    plt.savefig(plot_path)
    print(f"\nSaved {plot_path}")

    # ---------------- Compare against previous best, decide deploy ----------------
    history = load_history(args.outdir)
    history["runs"].append({
        "mae": mae, "r2": r2,
        "n_train": len(X_train), "n_test": len(X_test),
        "n_sessions": len(glob.glob(os.path.join(args.datadir, "*.csv"))),
    })

    is_best = history["best_mae"] is None or mae < history["best_mae"]

    if is_best:
        print(f"\nMAE {mae:.3f}m beats previous best ({history['best_mae']}).")
        history["best_mae"] = mae
    else:
        print(f"\nMAE {mae:.3f}m did not beat previous best"
        f"({history['best_mae']:.3f}m), but exporting anyway "
        f"(size/deployment constrints override pure accuracy here).")    
    c_code = port(model)
    header_path = os.path.join(args.outdir, DEPLOYED_HEADER)
    with open(header_path, "w") as f:
        f.write(c_code)
    print(f"Saved {header_path} -> copy this to your STM32CubeIDE project")

    save_history(args.outdir, history)

    print("\nMAE across all runs so far:")
    for i, run in enumerate(history["runs"]):
        marker = " <- current" if i == len(history["runs"]) - 1 else ""
        print(f"  run {i+1}: MAE={run['mae']:.3f}m  (n_sessions={run['n_sessions']}, n_train={run['n_train']}){marker}")

    print("\nUsage in STM32 (C++), once distance_regressor.h is copied over:")
    print('  #include "distance_regressor.h"')
    print("  Eloquent::ML::Port::RandomForestRegressor reg;")
    print("  float features[] = {rssi_mean, rssi_std, rssi_min, rssi_max, rssi_median};")
    print("  float predicted_distance_m = reg.predict(features);")


if __name__ == "__main__":
    main()
