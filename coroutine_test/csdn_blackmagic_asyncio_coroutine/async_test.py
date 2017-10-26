#http://blog.csdn.net/ChenVast/article/details/77943059?locationNum=10&fps=1
import time
import asyncio

now = lambda : time.time()

async def do_some_work(x):
    print('Waiting: ', x)

start = now()

coroutine = do_some_work(2)

loop = asyncio.get_event_loop()
loop.run_until_complete(coroutine)

print('TIME: ', now() - start)
