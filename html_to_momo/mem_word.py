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

    def __str__(self):
        output = "="*50 + '\n'
        output += self.pos.upper() + '\n'

        for wd in self.word_defs:
            wd_buff = "-"*10 + '\n'
            wd_buff += wd.definition + '\n'
            wd_buff += wd.example + '\n'
            wd_buff += wd.synonyms + '\n'
            output += wd_buff

        return output

def GetFirst(w):
    return w[0] if len(w) > 0 else ''

def LexicoWordOrigin(w):
    from lxml import html
    import requests

    page = requests.get('https://www.lexico.com/en/definition/' + w)
    tree = html.fromstring(page.content)
    origin = GetFirst(tree.xpath('//div[@class="senseInnerWrapper"]/p'))
    origin = origin.text_content() if origin != '' else ''
    pos_block_root = tree.xpath('//section[@class="gramb"]')
    pos_blocks = []
    for pos_block in pos_block_root:
        pos = pos_block.xpath('h3[@class="ps pos"]/span[@class="pos"]/text()')[0]
        meaning_block = pos_block.xpath('ul[@class="semb"]/li/div[@class="trg"]')
        word_def_blocks = []
        for m in meaning_block:
            meaning = GetFirst(m.xpath('p/span[@class="ind"]/text()'))
            #print("meaning: {}".format(meaning))
            example = GetFirst(m.xpath('div[@class="exg"]/div[@class="ex"]/em/text()'))
            synonyms = GetFirst(m.xpath('div[@class="synonyms"]/div[@class="exg"]/div'))
            synonyms = synonyms.text_content() if synonyms != '' else ''
            word_def_blocks.append(WordDefBlock(meaning, example, synonyms))
        pos_blocks.append(POSBlock(pos, word_def_blocks))

    derivative_word = GetFirst(tree.xpath('//div[@class="empty_sense"]/p[@class="derivative_of"]/a/text()'))
    derivative_of = MemWord.OnlineConstruct(derivative_word) if derivative_word != '' else None
    return pos_blocks, origin, derivative_of

    # if len(origin) == 0:
    #     return ''
    # return origin[0].text_content()

class MemWord():
    def __init__(self, word, mem_level, pos_blocks, origin, dervative_of=None):
        self.word = word
        self.mem_level = mem_level
        self.pos_blocks = pos_blocks
        self.definition_cn = ''
        self.origin = origin
        self.dervative_of = dervative_of

    def __str__(self):
        if self.dervative_of is not None:
            result = 'DEFINITION EMPTY: "{}" is the derivative of "{}"\n'.format(self.word, self.dervative_of.word)
            return result + self.dervative_of.__str__()

        result = ""
        for b in self.pos_blocks:
            result += b.__str__()
        result += "="*50 + '\n'
        result += self.origin + '\n'
        return result

    @classmethod
    def OnlineConstruct(cls, word):
        pos_blocks, origin, derivative_of = LexicoWordOrigin(word)
        return cls(word, 3.,pos_blocks, origin, derivative_of)

    @staticmethod
    def Vocabulary(words):
        voc = []
        for w in words:
            voc.append(MemWord.OnlineConstruct(w))
        return voc

