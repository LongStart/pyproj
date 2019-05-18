import PlotCollection
import pylab as pl

def add_3axis_figure(plotter, plot_name, vec3d_dict, linewidth=1):
    fig = pl.figure()
    for i in range(0, 3):
        ax = fig.add_subplot(311 + i)
        for key in vec3d_dict:
            ax.plot(vec3d_dict[key][0], vec3d_dict[key][1+i], label= key + str(i+1), linewidth=linewidth)
            ax.set_ylabel(str(i+1))
            pl.setp(ax.get_xticklabels(), visible=False)
            ax.legend()
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