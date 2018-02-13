from datetime import datetime
import matplotlib.dates as mdates
import matplotlib.pyplot as plt

stamptime = [1517025497244088029, 1517025497247888080]
strtime = ['2018-01-27 11:58:17.239676', '2018-01-27 11:58:17.247676']
y1 = [1, 3]
y2 = [4, 2]


def stringtimetodatetime(strtimelist):
    ret = [datetime.strptime(d, '%Y-%m-%d %H:%M:%S.%f') for d in strtimelist]
    return ret


def stamptimetodatetime(stamptimelist):
    ret = [datetime.fromtimestamp(d / 1e9) for d in stamptimelist]
    return ret


x1 = stringtimetodatetime(strtime)
x2 = stamptimetodatetime(stamptime)

plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S.%f'))

plt.plot(x1, y1, x2, y2)
plt.gcf().autofmt_xdate()  # 自动旋转日期标记
plt.show()
