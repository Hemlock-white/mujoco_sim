import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np


LEG_NAMES = ("FR", "FL", "RR", "RL")
MPC_LEG_NAMES = ("FL", "FR", "RL", "RR")


def read_csv_if_exists(path):
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return {}
    data = {}
    for key in reader.fieldnames:
        values = []
        for row in rows:
            value = row.get(key, "")
            try:
                values.append(float(value))
            except (TypeError, ValueError):
                values.append(np.nan)
        data[key] = np.asarray(values, dtype=np.float64)
    return data


def time_axis(df):
    if "sim_time" in df:
        return df["sim_time"]
    if "running_time" in df:
        return df["running_time"]
    if "wall_time_ns" in df:
        wall = df["wall_time_ns"].astype(np.float64)
        return (wall - wall[0]) / 1e9
    return np.arange(len(df))


def existing_columns(df, prefix, count):
    return [f"{prefix}_{i}" for i in range(count) if f"{prefix}_{i}" in df]


def savefig(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"[plot] wrote {path}")


def plot_latency(log_dir, out_dir, controller, bridge, pub):
    plt.figure(figsize=(12, 8))
    subplot = 1

    if bridge is not None and "lowcmd_age_ms" in bridge:
        plt.subplot(3, 1, subplot)
        subplot += 1
        t = time_axis(bridge)
        plt.plot(t, bridge["lowcmd_age_ms"], label="bridge lowcmd age ms")
        plt.ylabel("ms")
        plt.grid(True)
        plt.legend()

    if pub is not None and "wall_time_ns" in pub:
        plt.subplot(3, 1, subplot)
        subplot += 1
        t = time_axis(pub)
        wall = pub["wall_time_ns"].astype(np.float64)
        dt_ms = np.diff(wall, prepend=wall[0]) / 1e6
        plt.plot(t, dt_ms, label="controller lowcmd publish dt ms")
        plt.ylabel("ms")
        plt.grid(True)
        plt.legend()

    if controller is not None and "wall_time_ns" in controller:
        plt.subplot(3, 1, subplot)
        t = time_axis(controller)
        wall = controller["wall_time_ns"].astype(np.float64)
        dt_ms = np.diff(wall, prepend=wall[0]) / 1e6
        plt.plot(t, dt_ms, label="controller MPC loop dt ms")
        plt.ylabel("ms")
        plt.xlabel("time s")
        plt.grid(True)
        plt.legend()

    savefig(os.path.join(out_dir, "latency_timing.png"))


def plot_imu(controller, bridge, out_dir):
    source = controller if controller is not None else bridge
    if source is None:
        return
    t = time_axis(source)
    plt.figure(figsize=(12, 12))

    plt.subplot(4, 1, 1)
    cols = existing_columns(source, "se_rpy", 3)
    if cols:
        for col, name in zip(cols, ("roll", "pitch", "yaw")):
            plt.plot(t, np.rad2deg(source[col]), label=f"se {name} deg")
    plt.ylabel("deg")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 2)
    for col, name in zip(existing_columns(source, "imu_gyro", 3), ("gx", "gy", "gz")):
        plt.plot(t, source[col], label=name)
    plt.ylabel("rad/s")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 3)
    vel_prefix = "se_vbody" if "se_vbody_0" in source else "body_vel"
    for col, name in zip(existing_columns(source, vel_prefix, 3), ("vx", "vy", "vz")):
        plt.plot(t, source[col], label=name)
    plt.ylabel("m/s")
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 4)
    pos_prefix = "se_pos" if "se_pos_0" in source else ("body_pos" if "body_pos_0" in source else "high_pos")
    for col, name in zip(existing_columns(source, pos_prefix, 3), ("x", "y", "z")):
        plt.plot(t, source[col], label=name)
    plt.ylabel("m")
    plt.xlabel("time s")
    plt.grid(True)
    plt.legend()

    savefig(os.path.join(out_dir, "imu_state_estimator.png"))


def plot_foot_forces(controller, bridge, out_dir):
    source = bridge if bridge is not None else controller
    if source is None:
        return
    FR_RL = ("FR", "RL")
    if not any(f"foot_normal_{leg}" in source for leg in FR_RL):
        return
    t = time_axis(source)
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    for leg in FR_RL:
        col = f"foot_normal_{leg}"
        if col in source:
            plt.plot(t, source[col], label=leg)
    plt.ylabel("normal force")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    for leg in FR_RL:
        col = f"foot_force_norm_{leg}"
        if col in source:
            plt.plot(t, source[col], label=leg)
    plt.ylabel("force norm")
    plt.xlabel("time s")
    plt.grid(True)
    plt.legend()

    savefig(os.path.join(out_dir, "foot_contact_forces.png"))


def plot_mpc(controller, out_dir):
    if controller is None:
        return
    t = time_axis(controller)

    plt.figure(figsize=(12, 9))
    plt.subplot(3, 1, 1)
    for col, label in (("cmd_vx", "cmd vx"), ("cmd_vy", "cmd vy"), ("cmd_wz", "cmd wz")):
        if col in controller:
            plt.plot(t, controller[col], label=label)
    plt.ylabel("command")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    for idx, name in zip((9, 10, 11), ("vx", "vy", "vz")):
        col = f"mpc_x_{idx}"
        des_col = f"mpc_x_des_{idx}"
        if col in controller:
            plt.plot(t, controller[col], label=f"x {name}")
        if des_col in controller:
            plt.plot(t, controller[des_col], "--", label=f"x_des {name}")
    plt.ylabel("m/s")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    for leg_i, leg in enumerate(MPC_LEG_NAMES):
        z_col = f"mpc_u_grf_{leg_i * 3 + 2}"
        if z_col in controller:
            plt.plot(t, controller[z_col], label=f"{leg} grf z")
    plt.ylabel("MPC GRF z")
    plt.xlabel("time s")
    plt.grid(True)
    plt.legend()

    savefig(os.path.join(out_dir, "mpc_state_input.png"))


def plot_feet(controller, out_dir):
    if controller is None:
        return
    t = time_axis(controller)
    if "foot_p_FR_0" not in controller and "foot_p_RL_0" not in controller:
        return

    plt.figure(figsize=(12, 10))
    for axis_i, axis in enumerate(("x", "y", "z")):
        plt.subplot(3, 1, axis_i + 1)
        for leg in ("FR", "RL"):
            col     = f"foot_p_{leg}_{axis_i}"
            des_col = f"foot_p_des_{leg}_{axis_i}"
            if col in controller:
                plt.plot(t, controller[col], label=f"{leg} p {axis}")
            if des_col in controller:
                plt.plot(t, controller[des_col], "--", label=f"{leg} p_des {axis}")
        plt.ylabel(axis)
        plt.grid(True)
        plt.legend(ncol=2, fontsize=8)
    plt.xlabel("time s")

    savefig(os.path.join(out_dir, "foot_actual_desired.png"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", default="logs/sdk2_debug")
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args()

    out_dir = args.out_dir or os.path.join(args.log_dir, "plots")
    controller = read_csv_if_exists(os.path.join(args.log_dir, "controller_mpc.csv"))
    bridge = read_csv_if_exists(os.path.join(args.log_dir, "bridge_sim.csv"))
    pub = read_csv_if_exists(os.path.join(args.log_dir, "controller_lowcmd_pub.csv"))

    if controller is None and bridge is None and pub is None:
        raise FileNotFoundError(f"No debug CSVs found in {args.log_dir}")

    plot_latency(args.log_dir, out_dir, controller, bridge, pub)
    plot_imu(controller, bridge, out_dir)
    plot_foot_forces(controller, bridge, out_dir)
    plot_mpc(controller, out_dir)
    plot_feet(controller, out_dir)


if __name__ == "__main__":
    main()
