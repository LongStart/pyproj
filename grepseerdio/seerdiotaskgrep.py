import re
import sys
import json
import os

if(len(sys.argv) < 2):
    print("no input file")
jsondata = {}
jsonfile = sys.argv[1]
print(jsonfile)
with open(jsonfile, 'r', encoding='utf-8') as f:
    jsondata = json.loads(f.read())
    f.close()

replacedict = jsondata['targettext']

for file in jsondata["targetfile"]:
    targetfile = jsondata['targetpath'] + '\\' + file

    f = open(targetfile, 'r')
    targettext = f.read()
    f.close()

    os.rename(targetfile, targetfile + '.bak')

    f = open(targetfile, 'w')

    for key in replacedict:
        targettext = re.sub(key, replacedict[key], targettext)
    f.write(targettext)
    f.close()
