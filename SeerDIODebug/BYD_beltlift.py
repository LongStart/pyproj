from socketinterface import *
from time import sleep
from os import system

ADD_TASK_COMMAND_HEAD = 0x00001048
BELTLIFT_COMMAND_EHAD = 0x00001049
BELTLIFT_LIFT_COMMAND = 1
BELTLIFT_BELT_COMMAND = 2


def lanchtask():
    f4kernelcommand(ADD_TASK_COMMAND_HEAD, [118])
    f4kernelcommand(ADD_TASK_COMMAND_HEAD, [117])

# liftbelt(0.02)
# system('pause')
# liftbelt(0.01)
# system('pause')
# liftbelt(0)


def increamentcountmove():
    loop = True
    while(loop):
        try:
            str_count = input('input count: ')
        except KeyboardInterrupt:
            loop = False
            str_count = '0'

        count = int(str_count)
        setpulse(count)


def inputexecute():
    loop = True
    while(loop):
        try:
            str_height = input('input height: ')
        except KeyboardInterrupt:
            loop = False
            str_height = '0'

        height = float(str_height)
        liftbelt(height)


def looptest():
    loop = True
    height = 0
    cnt = 0
    targetheight = 0.001
    while(loop):
        if height == targetheight:
            height = targetheight + 0.001
        else:
            height = targetheight
        try:
            sleep(0.5)
        except KeyboardInterrupt:
            loop = False

        liftbelt(height)
        cnt = cnt + 1
        print('cnt = %d' % cnt)

    liftbelt(0.0)


def multistarttest():
    delta = 0.005
    height = delta
    for i in range(0, 10):
        liftbelt(height)
        height = height + delta
        sleep(2)

    for i in range(0, 10):
        liftbelt(height)
        height = height - delta
        sleep(2)

    liftbelt(0)


def continuecali():
    loop = True
    cnt = 0
    while(loop):
        try:
            sleep(0.3)
        except KeyboardInterrupt:
            loop = False

        liftbelt(0)
        cnt = cnt + 1
        print('cnt num = %d' % cnt)


# lanchtask()
# multistarttest()
# looptest()
# inputexecute()
# increamentcountmove()
continuecali()
