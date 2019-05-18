import PlotCollection
import pylab as pl

#create the plot as usual
fig1=pl.figure()
pl.plot([1,2],[2,3])
fig2=pl.figure()
pl.plot([3,1],[4,5])

#add to collection
plotter = PlotCollection.PlotCollection("My window name")
plotter.add_figure("My plot1 name", fig1)
plotter.add_figure("My plot2 name", fig2)

#show collection
plotter.show()