from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from airdrop_py import (
    run_full_simulation,
)
from airdrop_py.defaults import prompt_all_defaults


OUTPUT_DIR = Path(__file__).resolve().parent


def main() -> None:
    print("Enter simulation parameters, or press Enter to keep each default value.")
    kte_p, vel_p, qys_p, jss_p, zhs_p, sim_p = prompt_all_defaults()

    result = run_full_simulation(
        kte_p,
        vel_p,
        qys_p,
        jss_p,
        zhs_p,
        sim_p,
    )

    time_all = np.asarray(result["time_all"], dtype=float)
    data_all = np.asarray(result["data_all"], dtype=float)

    print("Python airdrop simulation finished.")
    print(f"data_all shape: {data_all.shape}")
    print(f"time_all range: {time_all[0]:.6f} to {time_all[-1]:.6f} s")
    print()
    print("Stage stop times:")
    for name in (
        "time_pd",
        "time_fz",
        "time_jslz",
        "time_jscq",
        "time_mslz",
        "time_mscq",
    ):
        print(f"  {name}: {float(result[name]):.9f} s")

    print()
    print("Final data_all row:")
    print(data_all[-1])

    csv_path = OUTPUT_DIR / "python_data_all.csv"
    png_path = OUTPUT_DIR / "python_results.png"
    npz_path = OUTPUT_DIR / "python_results.npz"

    header = "X,Y,Z,Vx,Vy,Vz,theta,beta,force"
    np.savetxt(csv_path, data_all, delimiter=",", header=header, comments="")
    np.savez(npz_path, time_all=time_all, data_all=data_all)

    titles = (
        "X position",
        "Z position",
        "Y position",
        "X velocity",
        "Z velocity",
        "Y velocity",
        "Pitch angle",
        "Roll angle",
        "Rope force",
    )
    y_labels = (
        "m",
        "m",
        "m",
        "m/s",
        "m/s",
        "m/s",
        "deg",
        "deg",
        "g",
    )
    columns = (0, 2, 1, 3, 5, 4, 6, 7, 8)

    fig, axes = plt.subplots(3, 3, figsize=(15, 10), constrained_layout=True)
    for axis, title, unit, column in zip(
        axes.ravel(), titles, y_labels, columns
    ):
        axis.plot(time_all, data_all[:, column], linewidth=0.8)
        axis.set_title(title)
        axis.set_xlabel("Time (s)")
        axis.set_ylabel(unit)
        axis.grid(True)

    fig.suptitle("Python Airdrop Simulation Results")
    fig.savefig(png_path, dpi=150)

    print()
    print(f"Plot saved to: {png_path}")
    print(f"CSV saved to:  {csv_path}")
    print(f"NPZ saved to:  {npz_path}")

    plt.show()


if __name__ == "__main__":
    main()
