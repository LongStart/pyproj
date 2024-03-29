import PlotCollection
import pylab as pl

def check_signal_dict(signal_dict):
    length = len(list(signal_dict.values())[0][0])
    for key in signal_dict:
        for i in range(1,len(signal_dict[key])):
            if len(signal_dict[key][0]) != len(signal_dict[key][i]):
                raise ValueError('signal {0} axis {1} length mismatch, len(t): {2}, len axis: {3}'.format(key, i, len(signal_dict[key][0]), len(signal_dict[key][i])))

def add_3axis_figure(plotter, plot_name, vec3d_dict, linewidth=1, fmt=''):
    fig = pl.figure()
    check_signal_dict(vec3d_dict)
    ax1 = fig.add_subplot(311)
    for i in range(0, 3):
        ax = None
        if i == 0:
            ax = ax1
        else:
            ax = fig.add_subplot(311 + i, sharex = ax1)
        for key in vec3d_dict:
            ax.plot(vec3d_dict[key][0], vec3d_dict[key][1+i], fmt, label= key , linewidth=linewidth)
            ax.set_ylabel(chr(120 + i))
            ax.legend()
            ax.grid(1)
    plotter.add_figure(plot_name, fig)

def add_naxis_figure(plotter, plot_name, vecxd_dict, linewidth=1, fmt='', markersize=1.):
    fig = pl.figure()
    check_signal_dict(vecxd_dict)
    signal_dim = len(list(vecxd_dict.values())[0]) - 1
    subplot_code = signal_dim * 100 + 11
    ax1 = fig.add_subplot(subplot_code)
    for i in range(0, signal_dim):
        ax = None
        if i == 0:
            ax = ax1
        else:
            ax = fig.add_subplot(subplot_code + i, sharex = ax1)
        for key in vecxd_dict:
            ax.plot(vecxd_dict[key][0], vecxd_dict[key][1+i], fmt, label= key , linewidth=linewidth, markersize=markersize)
            ax.set_ylabel(chr(120 + i))
            ax.legend()
            ax.grid(1)
    plotter.add_figure(plot_name, fig)


        

if __name__ == '__main__':
    
    sig = {'v': [[1,2,3],[0,1,1],[0,0,1],[1,1,2]]}

    fig2=pl.figure()
    pl.plot([3,1],[4,5])

    plotter = PlotCollection.PlotCollection("My window name")
    add_3axis_figure(plotter, "ddd", sig)
    plotter.add_figure("My plot2 name", fig2)

    #show collection
    plotter.show()