import rsa
uid = '250024000251363332363934'
keyfile = open('privkey.pem')
keydata = keyfile.read()
privk = rsa.PrivateKey.load_pkcs1(keydata, format='PEM')
signature = rsa.sign(uid, privk, 'SHA-1')
print(signature.hex().upper())
