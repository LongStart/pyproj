import matplotlib 
import matplotlib.pyplot as plt
font_name = "Microsoft YaHei"
matplotlib.rcParams['font.family']=font_name
matplotlib.rcParams['axes.unicode_minus']=False # in case minus sign is shown as box
plt.text(0.5, 0.5, s= u'测试')
plt.show()