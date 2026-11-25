#!/usr/bin/env python3
"""
plot_profile.py

Lee el CSV generado por el profiling de gradient_planner.cpp
(/tmp/gradient_planner_profile.csv, columnas: timestamp_us,function,duration_ms)
y genera:

  1. summary.csv           -> estadísticas (count, mean, median, min, max, std, total) por función
  2. timeseries_<func>.png -> evolución temporal de la duración de cada función
  3. boxplot_all.png       -> boxplot comparativo de todas las funciones
  4. total_time_bar.png    -> tiempo total acumulado por función (dónde se va el tiempo)

Uso:
  python3 plot_profile.py [ruta_al_csv] [directorio_salida]

  Por defecto:
    ruta_al_csv     = /tmp/gradient_planner_profile.csv
    directorio_salida = ./profile_plots
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt


def load_data(csv_path: str) -> pd.DataFrame:
    if not os.path.isfile(csv_path):
        sys.exit(
            f"ERROR: no encuentro el fichero '{csv_path}'. "
            f"¿Se ha ejecutado el planner con el profiling activo?"
        )

    df = pd.read_csv(csv_path)
    expected_cols = {"timestamp_us", "function", "duration_ms"}
    if not expected_cols.issubset(df.columns):
        sys.exit(
            f"ERROR: el CSV no tiene las columnas esperadas {expected_cols}, "
            f"tiene: {list(df.columns)}"
        )

    # Tiempo relativo al primer registro, en segundos, para que el eje X sea legible
    df = df.sort_values("timestamp_us").reset_index(drop=True)
    df["t_rel_s"] = (df["timestamp_us"] - df["timestamp_us"].iloc[0]) / 1e6
    return df


def write_summary(df: pd.DataFrame, out_dir: str) -> pd.DataFrame:
    summary = (
        df.groupby("function")["duration_ms"]
        .agg(
            count="count",
            mean_ms="mean",
            median_ms="median",
            min_ms="min",
            max_ms="max",
            std_ms="std",
            total_ms="sum",
        )
        .sort_values("total_ms", ascending=False)
    )
    summary_path = os.path.join(out_dir, "summary.csv")
    summary.to_csv(summary_path)

    print("\n=== Resumen por función (ordenado por tiempo total acumulado) ===\n")
    print(summary.to_string(float_format=lambda x: f"{x:.4f}"))
    print(f"\nResumen guardado en: {summary_path}")
    return summary


def plot_timeseries(df: pd.DataFrame, out_dir: str) -> None:
    for func, group in df.groupby("function"):
        plt.figure(figsize=(9, 4))
        plt.plot(
            group["t_rel_s"],
            group["duration_ms"],
            marker="o",
            markersize=3,
            linewidth=0.8,
        )
        plt.title(f"Tiempo de ejecución: {func}")
        plt.xlabel("Tiempo transcurrido (s)")
        plt.ylabel("Duración (ms)")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        out_path = os.path.join(out_dir, f"timeseries_{func}.png")
        plt.savefig(out_path, dpi=130)
        plt.close()
        print(f"Guardado: {out_path}")


def plot_boxplot(df: pd.DataFrame, out_dir: str) -> None:
    functions = sorted(df["function"].unique())
    data = [df.loc[df["function"] == f, "duration_ms"].values for f in functions]

    plt.figure(figsize=(max(6, len(functions) * 1.2), 5))
    plt.boxplot(data, labels=functions, showfliers=True)
    plt.ylabel("Duración (ms)")
    plt.title("Distribución de tiempos por función")
    plt.xticks(rotation=45, ha="right")
    plt.yscale("log")  # las funciones suelen tener escalas muy distintas
    plt.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(out_dir, "boxplot_all.png")
    plt.savefig(out_path, dpi=130)
    plt.close()
    print(f"Guardado: {out_path}")


def plot_total_time(summary: pd.DataFrame, out_dir: str) -> None:
    plt.figure(figsize=(8, 5))
    summary_sorted = summary.sort_values("total_ms")
    plt.barh(summary_sorted.index, summary_sorted["total_ms"])
    plt.xlabel("Tiempo total acumulado (ms)")
    plt.title("Dónde se va el tiempo (suma de todas las llamadas)")
    plt.grid(True, axis="x", alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(out_dir, "total_time_bar.png")
    plt.savefig(out_path, dpi=130)
    plt.close()
    print(f"Guardado: {out_path}")


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/gradient_planner_profile.csv"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "./e7_gradients"
    os.makedirs(out_dir, exist_ok=True)

    df = load_data(csv_path)
    summary = write_summary(df, out_dir)
    plot_timeseries(df, out_dir)
    plot_boxplot(df, out_dir)
    plot_total_time(summary, out_dir)

    print(f"\nListo. Todas las gráficas están en: {os.path.abspath(out_dir)}")


if __name__ == "__main__":
    main()
