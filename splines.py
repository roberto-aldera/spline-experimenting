import numpy as np
from scipy.linalg import logm, expm
import matplotlib.pyplot as plt


def build_spline(global_transforms, rel_transforms):
    omegas = []
    for i in range(len(rel_transforms) - 1):
        omegas.append(logm(np.linalg.inv(global_transforms[i]) @ global_transforms[i + 1]))

    num_spline_points = 10
    num_intervals = len(transforms) - 3
    C = 1 / 6 * np.array([[6, 0, 0, 0], [5, 3, -3, 1], [1, 3, 3, -2], [0, 0, 0, 1]])
    T_w_s = [[None for _ in range(num_intervals)] for _ in range(num_spline_points)]
    s_i = np.linspace(0, len(rel_transforms) - 1, len(rel_transforms))
    for i in range(num_intervals):
        s_t = np.linspace(i + 1, i + 2, num_spline_points)
        for t in range(num_spline_points):
            u = s_t[t] - s_i[i + 1]
            u_vector = np.array([[1], [u], [u * u], [u * u * u]])
            B = np.matmul(C, u_vector)
            T_w_s[t][i] = global_transforms[i] @ expm(B[1] * omegas[i]) @ expm(B[2] * omegas[i + 1]) @ expm(
                B[3] * omegas[i + 2])
    return T_w_s


def plot_spline(spline_points_to_plot):
    plt.figure(figsize=(10, 10))
    for i in range(len(transforms)):
        plt.plot(T_world[i][0, 3], T_world[i][1, 3], 'o', color='tab:red')

    for i in range(len(spline_points_to_plot)):
        for j in range(len(spline_points_to_plot[0])):
            plt.plot(spline_points_to_plot[i][j][0, 3], spline_points_to_plot[i][j][1, 3], '.', color='tab:blue')

    plt.title("Cubic B-spline")
    plt.grid()
    plt.savefig("/Users/roberto/Desktop/splines.png")
    plt.close()


if __name__ == "__main__":
    print("Building spline with some toy data...")
    theta = -np.pi / 32
    R = np.identity(3)
    R[0, :] = np.cos(theta), np.sin(theta), 0
    R[1, :] = -np.sin(theta), np.cos(theta), 0
    R[2, :] = 0, 0, 1
    a = np.array([[1], [0], [0]])  # position of origin frame a in reference frame b
    T0 = np.c_[R, a]
    T0 = np.r_[T0, np.array([[0, 0, 0, 1]])]

    transforms = [np.array(T0) for _ in range(10)]
    transforms[2][0, 3] += 0.3
    transforms[2][1, 3] -= 0.3
    transforms[5][0, 3] += 0.2
    transforms[5][1, 3] -= 0.4
    transforms[7][0, 3] += 0.1
    transforms[7][1, 3] -= 0.1
    T_world = [np.identity(4)]

    for idx in range(1, len(transforms)):
        T_world.append(np.matmul(T_world[idx - 1], transforms[idx]))

    spline_points = build_spline(T_world, transforms)
    plot_spline(spline_points)

    print("Complete!")
