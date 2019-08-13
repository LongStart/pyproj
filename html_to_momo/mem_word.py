import sys

def LexicoWordOrigin(w):
    from lxml import html
    import requests

    page = requests.get('https://www.lexico.com/en/definition/' + w)
    tree = html.fromstring(page.content)
    origin = tree.xpath('//div[@class="senseInnerWrapper"]/p')
    if len(origin) == 0:
        return ''
    return origin[0].text_content()

def GoogleTransMeaning(translator, w):
    result = translator.translate(w, dest='zh-cn')


class MemWord():
    def __init__(self, word, g_trans, mem_level, origin):
        self.word = word
        self.definition = g_trans.extra_data['definitions']
        self.synonyms = g_trans.extra_data['synonyms']
        self.mem_level = mem_level
        self.definition_cn = g_trans.text
        self.origin = origin

    @classmethod
    def OnlineConstruct(cls, word, translator):
        origin = LexicoWordOrigin(w)
        g_trans = GoogleTransMeaning(translator, w)
        return cls(w, g_trans, 3., origin)

    @staticmethod
    def Vocabulary(words):
        translator = Translator(service_urls=['translate.google.cn'])
        voc = []
        for w in words:
            voc.append(MemWord.OnlineConstruct(w, translator))
        return voc

