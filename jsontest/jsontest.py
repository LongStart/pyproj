import json
filename = 'test.json'
file = open(filename, 'r')
filedata = file.read()
file.close()
print(filedata)

dictdata = json.loads(filedata)
print(dictdata)

outstring = json.dumps(dictdata)
fileoutname = 'test1.json'
fileout = open(fileoutname, 'w')
fileout.write(outstring)
fileout.close()

