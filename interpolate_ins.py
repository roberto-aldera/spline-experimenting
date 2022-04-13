import numpy as np
import sys
import os
import matplotlib.pyplot as plt
import math
import settings
from pose_utils import *


def do_things(dataset_path, ro_timestamps, ro_x_y_th, spline_timestamps, spline_x_y_th):
    """
    Load poses from monolithic file.
    """
    print('Loading poses from monolithic file...')

    # Magic
    sys.path.append(os.path.expanduser("~/code/corelibs/src/tools-python"))
    sys.path.append(os.path.expanduser("~/code/corelibs/build/datatypes"))
    sys.path.append(os.path.expanduser("~/code/corelibs/build/datatypes/datatypes_python"))

    # from mrg.logging import MonolithicDecoder
    # from mrg.adaptors.transform import PbSerialisedTransformToPython
    from mrg.transform.relative_pose_client import RelativePoseClient
    rpc = RelativePoseClient(dataset_path)

    print("Start time for buffer:", rpc.t_start)
    print("RO start time:", ro_timestamps[0])

    time_difference = 0
    if ro_timestamps[0] < rpc.t_start:
        time_difference = rpc.t_start - ro_timestamps[0]
    else:
        print("THIS IS BAD, operating outside the current ODD...")
        # TODO - handle case where INS might be logging before RO...
        # time_difference = ro_timestamps[0] - rpc.t_start

    print("Diff =", time_difference)
    print("So diff in frames = ", (rpc.t_start - ro_timestamps[0]) / 250000)
    offset = math.ceil(time_difference / 250000) + 1  # +1 because we query between the previous and current timestamp
    print("Offset is then:", offset)

    interpolated_ins_poses = []
    interpolated_ins_timestamps = []
    for i in range(offset, len(ro_timestamps)):
        # for i in range(offset, 1000):
        origin_timestamp = ro_timestamps[i - 1]
        query_timestamp = ro_timestamps[i]
        pose = rpc.get_relative_pose(query_timestamp, origin_timestamp)
        interpolated_ins_poses.append(pose)
        interpolated_ins_timestamps.append(query_timestamp)
        if i % 100 == 0:
            print("Total poses processed:", i)
    save_poses_to_csv(interpolated_ins_poses, interpolated_ins_timestamps, "interpolated_ins", settings.FIGURE_PATH)

    # Crop other poses to align with INS:
    ro_timestamps = ro_timestamps[offset:]
    ro_x_y_th = ro_x_y_th[offset:]
    spline_timestamps = spline_timestamps[offset:]
    spline_x_y_th = spline_x_y_th[offset:]
    save_timestamps_and_x_y_th_to_csv(ro_timestamps, ro_x_y_th, "ro_aligned", settings.FIGURE_PATH)
    save_timestamps_and_x_y_th_to_csv(spline_timestamps, spline_x_y_th, "spline_aligned", settings.FIGURE_PATH)


def plot_interpolated_ins():
    ins_timestamps, ins_x_y_th = get_timestamps_and_x_y_th_from_csv(settings.INTERPOLATED_INS_CSV)
    ins_speeds = get_speeds_from_x_y_th(ins_x_y_th, ins_timestamps)
    colour_ins = "black"
    plt.figure(figsize=(15, 5))
    plt.plot(ins_speeds, ".-", color=colour_ins, label="Ground truth")
    # plt.xlim(1100,1500)
    # plt.ylim(-5, 20)
    plt.title("INS speed from querying relative pose buffer")
    plt.legend()
    plt.grid()
    plt.savefig(settings.FIGURE_PATH + "interpolated_ins_speeds.png")
    plt.close()


def filter_with_sliding_window(input_signal, input_timestamps, window_size=5):
    std_devs = []
    means = []
    for i in range(window_size // 2, len(input_signal) - window_size // 2):
        window_contents = input_signal[i - window_size // 2:i + window_size // 2]
        std_devs.append(np.std(window_contents))
        means.append(np.mean(window_contents))

    means = np.array(means)
    # means[-window_size // 2:] = 0
    means = np.roll(means, window_size // 2)
    import pdb
    # pdb.set_trace()
    # means[-window_size // 2:] = means[0:window_size // 2]
    means = np.append(means, means[0:window_size // 2])
    means[0:window_size // 2] = 0
    std_devs = np.array(std_devs)
    std_devs[-window_size // 2:] = 0
    std_devs = np.roll(std_devs, window_size // 2)
    timestamps = input_timestamps[:len(input_timestamps) - window_size // 2]
    return list(means), std_devs, timestamps


def filter_interpolated_ins():
    ins_timestamps, ins_x_y_th = get_timestamps_and_x_y_th_from_csv(settings.INTERPOLATED_INS_CSV)
    ins_speeds = get_speeds_from_x_y_th(ins_x_y_th, ins_timestamps)
    x_values = [float(item[0]) for item in ins_x_y_th]
    filtered_x_values, _, filtered_timestamps = filter_with_sliding_window([float(item[0]) for item in ins_x_y_th],
                                                                           ins_timestamps)
    filtered_y_values, _, _ = filter_with_sliding_window([float(item[1]) for item in ins_x_y_th], ins_timestamps)
    filtered_th_values, _, _ = filter_with_sliding_window([float(item[2]) for item in ins_x_y_th], ins_timestamps)
    filtered_x_y_th = []
    for i in range(len(filtered_timestamps)):
        filtered_x_y_th.append([filtered_x_values[i], filtered_y_values[i], filtered_th_values[i]])
    save_timestamps_and_x_y_th_to_csv(filtered_timestamps, filtered_x_y_th, "filtered_interpolated_ins",
                                      settings.FIGURE_PATH)

    colour_ins = "black"
    plt.figure(figsize=(15, 5))
    plt.plot(x_values, ".-", color="tab:red", label="INS")
    plt.plot(filtered_x_values, ".--", color="red", label="Filtered INS")
    # times = [(t - ins_timestamps[0]) / 1e6 for t in ins_timestamps]
    # plt.plot(times, '.')
    # plt.xlim(1100,1500)
    # plt.ylim(-5, 20)
    plt.title("Filtering INS to reduce noise")
    plt.legend()
    plt.grid()
    plt.savefig(settings.FIGURE_PATH + "filtering.png")
    plt.close()


def plot_all_sources():
    colour_ro = "tab:green"
    colour_ins = "black"
    colour_spline = "tab:blue"

    ins_timestamps, ins_x_y_th = get_timestamps_and_x_y_th_from_csv(settings.INS_FILTERED_INTERPOLATED_CSV)
    ro_timestamps, ro_x_y_th = get_timestamps_and_x_y_th_from_csv(settings.RO_ALIGNED_CSV)
    spline_timestamps, spline_x_y_th = get_timestamps_and_x_y_th_from_csv(settings.SPLINE_ALIGNED_CSV)

    # Data cropping
    # offset = math.ceil((ins_timestamps[0] - ro_timestamps[0]) / 250000)
    # ro_timestamps = ro_timestamps[offset:]
    # ro_x_y_th = ro_x_y_th[offset:]
    # spline_timestamps = spline_timestamps[offset:]
    # spline_x_y_th = spline_x_y_th[offset:]

    ins_speeds = get_speeds_from_x_y_th(ins_x_y_th, ins_timestamps)
    ro_speeds = get_speeds_from_x_y_th(ro_x_y_th, ro_timestamps)
    spline_speeds = get_speeds_from_x_y_th(spline_x_y_th, spline_timestamps)

    ins_seconds = [(t - ins_timestamps[0]) / 1e6 for t in ins_timestamps[1:]]
    ro_seconds = [(t - ro_timestamps[0]) / 1e6 for t in ro_timestamps[1:]]
    spline_seconds = [(t - spline_timestamps[0]) / 1e6 for t in spline_timestamps[1:]]

    plt.figure(figsize=(15, 5))
    plt.plot(ins_seconds, ins_speeds, ".-", color=colour_ins, label="Ground truth")
    plt.plot(ro_seconds, ro_speeds, ".-", color=colour_ro, label="RO")
    plt.plot(spline_seconds, spline_speeds, ".-", color=colour_spline, label="Splined RO")

    # plt.xlim(1100,1500)
    # plt.ylim(-5, 20)
    plt.title("Speed profiles")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.legend()
    plt.grid()
    plt.savefig(settings.FIGURE_PATH + "all_speeds.pdf")
    plt.close()


if __name__ == "__main__":
    timestamps_ro, x_y_th_ro = get_timestamps_and_x_y_th_from_csv(settings.RO_CSV)
    timestamps_spline, x_y_th_spline = get_timestamps_and_x_y_th_from_csv(settings.SPLINE_CSV)

    do_things(settings.INS_PATH, timestamps_ro, x_y_th_ro, timestamps_spline, x_y_th_spline)
    filter_interpolated_ins()
    plot_all_sources()
    print("Finished.")
