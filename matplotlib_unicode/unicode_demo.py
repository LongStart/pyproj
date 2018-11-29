import matplotlib as mpl
import matplotlib.pyplot as plt
font_name = "Microsoft YaHei"
mpl.rcParams['font.family']=font_name
mpl.rcParams['axes.unicode_minus']=False # in case minus sign is shown as box
plt.text(0.5, 0.5, s= u'测试')
plt.show()