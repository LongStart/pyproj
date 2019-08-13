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

class WordDefBlock():
    def __init__(self, definition, example, synonyms):
        self.definition = definition
        self.example = example
        self.synonyms = synonyms

class POSBlock():
    def __init__(self, pos, word_defs):
        self.pos = pos
        self.word_defs = word_defs

def LexicoWordOrigin(w):
    from lxml import html
    import requests

    page = requests.get('https://www.lexico.com/en/definition/' + w)
    tree = html.fromstring(page.content)
    origin = tree.xpath('//div[@class="senseInnerWrapper"]/p')
    pos_block_root = tree.xpath('//section[@class="gramb"]')
    pos_blocks = []
    for pos_block in pos_block_root:
        pos = tree.xpath('h3[@class="ps pos"]/span[@class="pos"]/text()')
        meaning_block = tree.xpath('ul[@class="semb"]/li/div[@class="trg"]')
        word_def_blocks = []
        for m in meaning_block:
            meaning = m.xpath('p/span[@class="ind"]/text()')
            example = m.xpath('div[@class="exg"]/div[@class="ex"]/em/text()')
            synonyms = m.xpath('div[@class="synonyms"]/div[@class="exg"]/div')
            word_def_blocks.append(WordDefBlock(meaning, example, synonyms))
        pos_blocks.append(POSBlock(pos, word_def_blocks))
    pos_blocks

    if len(origin) == 0:
        return ''
    return origin[0].text_content()

class MemWord():
    def __init__(self, word, mem_level, pos_blocks, origin):
        self.word = word
        self.mem_level = mem_level
        self.pos_blocks = pos_blocks
        self.definition_cn = ''
        self.origin = origin

    @classmethod
    def OnlineConstruct(cls, word):
        origin = LexicoWordOrigin(w)
        return cls(w, g_trans, 3., origin)

    @staticmethod
    def Vocabulary(words):
        translator = Translator(service_urls=['translate.google.cn'])
        voc = []
        for w in words:
            voc.append(MemWord.OnlineConstruct(w, translator))
        return voc

