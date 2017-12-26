import os
import sys
import time
import atexit
import signal
import traceback


fout = open('.\\log.txt','w')
def term_sig_handler(signum, frame):
    print('catched signal %d' % signum)
    fout.write('catch signal' + str(signum))
    fout.close()
    sys.exit()

@atexit.register
def atexit_fun():
    print('I am exit, stack track:')

    exc_type, exc_value, exc_tb = sys.exc_info()
    traceback.print_exception(exc_type, exc_value, exc_tb)
    
if __name__ == '__main__':
    signal.signal(signal.SIGTERM, term_sig_handler)
    signal.signal(signal.SIGINT, term_sig_handler)

    while True:
        print('hello')
        time.sleep(3)
