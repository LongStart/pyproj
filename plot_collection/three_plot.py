import PlotCollection
import pylab as pl

#create the plot as usual
fig1=pl.figure()

ax1 = fig1.add_subplot(311)
ax1.plot([1,2,3], [4,5,6], 'r', label='ax', linewidth=1)
ax1.set_ylabel('aaaa')
pl.setp(ax1.get_xticklabels(), visible=False)
ax1.legend()

ax2 = fig1.add_subplot(312, sharex=ax1)
ax2.plot([1,2,3], [4,5,6], 'r', label='ax', linewidth=1)
ax2.set_ylabel('bbbb')
pl.setp(ax2.get_xticklabels(), visible=False)
ax2.legend()

ax3 = fig1.add_subplot(313, sharex=ax1)
ax3.plot([3,4,5], [4,5,6], 'r', label='ax', linewidth=1)
ax3.set_ylabel('ccc')
pl.setp(ax3.get_xticklabels(), visible=False)
ax3.legend()

fig2=pl.figure()
pl.plot([3,1],[4,5])

#add to collection
plotter = PlotCollection.PlotCollection("My window name")
plotter.add_figure("My plot1 name", fig1)
plotter.add_figure("My plot2 name", fig2)

#show collection
plotter.show()