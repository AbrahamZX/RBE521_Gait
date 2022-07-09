from helpers import generate_leg_positions
import matplotlib as mpl
import matplotlib.pyplot as plt
from IPython.terminal.embed import InteractiveShellEmbed


def plot_gait_analysis_stuff():
    fig, ax = plt.subplots()

    for t in range(10):
        # t = ms / 1000
        xr, zr, xl, zl = generate_leg_positions(t)
        print("xr", xr)
        print("zr", zr)
        print("xl", xl)
        print("zl", zl)
        ax.plot(t, xr, "green")
        ax.plot(t, zr, "yellow")
        ax.plot(t, xl, "blue")
        ax.plot(t, zl, "black")
    fig.show()

    try:
        InteractiveShellEmbed()
    except KeyboardInterrupt:
        print("die!")
        

if __name__ == '__main__':
    plot_gait_analysis_stuff()
