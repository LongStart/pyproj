import asyncio
import time
from prompt_toolkit import prompt
from prompt_toolkit.history import FileHistory
from prompt_toolkit.contrib.completers import WordCompleter

from prompt_toolkit import prompt_async
import sys

async def my_coroutine():
    while True:
        result = await prompt_async('Say something: ', patch_stdout=True)
        print('You said: %s' % result)

async def print_ro():
	while True:
		print('################')
		sys.stdout.flush()
		await asyncio.sleep(0.1)


coroutine = my_coroutine()
co2 = print_ro()
loop = asyncio.get_event_loop()
#task = [asyncio.ensure_future(coroutine), asyncio.ensure_future(co2)]
task = [asyncio.ensure_future(co2),asyncio.ensure_future(coroutine) ]
loop.run_until_complete(asyncio.wait(task))

