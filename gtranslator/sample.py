from googletrans import Translator
translator = Translator(service_urls=['translate.google.cn'])
ret = translator.translate("confessional", dest='zh-cn')
# print(ret)
print(ret.extra_data['definitions'][0])
# print(ret.extra_data['synonyms'])
# for key in ret.extra_data:
#     print(key)