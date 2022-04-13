import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import logm, expm
from pathlib import Path
from pose_utils import *
import settings


class Pose:
    def __init__(self, relative_transform, timestamp):
        self.relative_transform = relative_transform
        self.timestamp = timestamp


class SplineSegment:
    def __init__(self, g_tm1: Pose, g_t0: Pose, g_tp1: Pose, g_tp2: Pose, omega_0, omega_1, omega_2):
        self.g_tm1 = g_tm1
        self.g_t0 = g_t0
        self.g_tp1 = g_tp1
        self.g_tp2 = g_tp2
        self.omega_0 = omega_0
        self.omega_1 = omega_1
        self.omega_2 = omega_2


def make_local_spline(g_tm1: Pose, g_t0: Pose, g_tp1: Pose, g_tp2: Pose):
    omega_0 = logm(np.linalg.inv(g_tm1.relative_transform) @ g_t0.relative_transform)
    omega_1 = logm(np.linalg.inv(g_t0.relative_transform) @ g_tp1.relative_transform)
    omega_2 = logm(np.linalg.inv(g_tp1.relative_transform) @ g_tp2.relative_transform)

    return SplineSegment(g_tm1, g_t0, g_tp1, g_tp2, omega_0, omega_1, omega_2)


def evaluate_spline(spline_segment: SplineSegment, query_timestamp):
    u = query_timestamp - spline_segment.g_t0.timestamp
    # Account for sensor frequency (so that the difference between start and end of spline in u terms is 1
    u /= (spline_segment.g_tp1.timestamp - spline_segment.g_t0.timestamp)

    u_vector = np.array([[1], [u], [u * u], [u * u * u]])
    C = 1 / 6 * np.array([[6, 0, 0, 0], [5, 3, -3, 1], [1, 3, 3, -2], [0, 0, 0, 1]])
    B = np.matmul(C, u_vector)

    T_spline = spline_segment.g_tm1.relative_transform @ expm(B[1] * spline_segment.omega_0) @ expm(
        B[2] * spline_segment.omega_1) @ expm(B[3] * spline_segment.omega_2)
    return Pose(T_spline, query_timestamp)


if __name__ == "__main__":
    print("Running main...")
    colour_ro = "tab:green"
    colour_ins = "black"
    colour_spline = "tab:blue"

    RO_se3s, RO_timestamps = get_poses_from_file(settings.RO_PATH)

    ro_poses = []
    spline_poses = []
    for i in range(0, 400):
        G_tm1 = Pose(RO_se3s[i], RO_timestamps[i])
        G_t0 = Pose(RO_se3s[i + 1], RO_timestamps[i + 1])
        G_tp1 = Pose(RO_se3s[i + 2], RO_timestamps[i + 2])
        G_tp2 = Pose(RO_se3s[i + 3], RO_timestamps[i + 3])

        local_spline = make_local_spline(G_tm1, G_t0, G_tp1, G_tp2)
        spline_pose = evaluate_spline(local_spline, G_tp1.timestamp)

        ro_poses.append(G_tp1)
        spline_poses.append(spline_pose)
        if i % 100 == 0:
            print("Total RO and spline poses processed:", i)

    ro_se3s = [pose.relative_transform for pose in ro_poses]
    ro_timestamps = [pose.timestamp for pose in ro_poses]
    ro_speeds = get_speeds(ro_se3s, ro_timestamps)

    spline_se3s = [pose.relative_transform for pose in spline_poses]
    spline_timestamps = [pose.timestamp for pose in spline_poses]
    spline_speeds = get_speeds(spline_se3s, spline_timestamps)

    Path(settings.OUTPUT_ROOT_PATH).mkdir(parents=True, exist_ok=True)
    print("Saving outputs to:", settings.OUTPUT_ROOT_PATH)

    plt.figure(figsize=(15, 5))
    plt.plot(ro_speeds, ".-", color=colour_ro, label="RO")
    plt.plot(spline_speeds, ".-", color=colour_spline, label="Spline")
    # plt.xlim(1100,1500)
    # plt.ylim(-5, 20)
    plt.title("Comparing smoothness of speed profile")
    plt.legend()
    plt.grid()
    plt.savefig(settings.FIGURE_PATH + "speeds.png")

    save_poses_to_csv(ro_se3s, ro_timestamps, "ro", settings.FIGURE_PATH)
    save_poses_to_csv(spline_se3s, spline_timestamps, "spline", settings.FIGURE_PATH)
