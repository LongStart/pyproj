import sys
from html_to_momo import ExtractWordsFromHtmlVocabulary
from googletrans import Translator

class MemWord():
    def __init__(self, word, g_trans, mem_level, origin):
        self.word = word
        self.definition = g_trans.extra_data['definitions']
        self.synonyms = g_trans.extra_data['synonyms']
        self.mem_level = mem_level
        self.definition_cn = g_trans.text
        self.origin = origin

    # @classmethod

def GetWordOrigin(w):
    from lxml import html
    import requests

    page = requests.get('https://www.lexico.com/en/definition/' + w)
    tree = html.fromstring(page.content)
    origin = tree.xpath('//div[@class="senseInnerWrapper"]/p')
    if len(origin) == 0:
        return ''
    return origin[0].text_content()

def PartOfSpeech(pos):
    abbrevs = {'形容词': 'adj.', '动词': 'v.', '名词': 'n.', '介词':'prep.'}
    if pos in abbrevs:
        return abbrevs[pos]
    else:
        return pos

def MemLevel(cls):
        return cls.mem_level

def LoadMemWords(words):
    translator = Translator(service_urls=['translate.google.cn'])
    mem_words = []
    print('loading...')
    for w in words:
        print('.', end='', flush=True)
        g_trans = translator.translate(w, dest='zh-cn')
        mem_word = MemWord(w, g_trans, 4, GetWordOrigin(w))
        mem_words.append(mem_word)
    return mem_words

def ParseDefinitionTuple(d):
    part_of_speech = PartOfSpeech(d[0])
    meaning = d[1][0][0]
    example = d[1][0][2] if (len(d[1][0]) > 2) else ''
    return part_of_speech, meaning, example

def ParseSynonymsTuple(s):
    part_of_speech = PartOfSpeech(s[0])
    words = s[1][0][0]
    return part_of_speech, words

def FormatPrintDefinition(definition):
    if definition is None:
        return
    for d in definition:
        print("|||meaning|||:")
        print("{}: \n\t{}\n\t\"{}\"".format(*ParseDefinitionTuple(d)))

def FormatPrintSynonyms(synonyms):
    if synonyms is None:
        return
    for s in synonyms:
        print("|||synonyms|||:")
        print("{}: \n\t{}".format(*ParseSynonymsTuple(s)))

def FormatPrintTranslation(trans):
    if trans is None:
        return
    print("|||ch|||:")
    print(trans)

def UpdateMemWords(mem_words):
    for w in mem_words:
        w.mem_level += 0.01
    mem_words.sort(key=MemLevel , reverse=True)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("eg.: python {} vocabulary.html".format(sys.argv[0]))
        quit()
    html_filename = sys.argv[1]
    words = ExtractWordsFromHtmlVocabulary(html_filename)
    mem_words = LoadMemWords(words)
    while True:
        UpdateMemWords(mem_words)
        input(mem_words[0].word)
        if mem_words[0].definition is None and mem_words[0].synonyms is None:
            print(mem_words[0].definition_cn)
        else:
            FormatPrintDefinition(mem_words[0].definition)
            FormatPrintSynonyms(mem_words[0].synonyms)
            FormatPrintTranslation(mem_words[0].definition_cn)

        while True:
            try:
                level = input("mem_level: ")
                print('\033c')
                mem_words[0].mem_level = float(level)
            except ValueError:
                pass
            else:
                break