import sys
from html_to_momo import ExtractWordsFromHtmlVocabulary
from googletrans import Translator

class MemWord():
    def __init__(self, word, g_trans, mem_level):
        self.word = word
        self.definition = g_trans.extra_data['definitions']
        self.synonyms = g_trans.extra_data['synonyms']
        self.mem_level = mem_level
        self.definition_cn = g_trans.text

    # @classmethod
    
def MemLevel(cls):
        return cls.mem_level

def LoadMemWords(words):
    translator = Translator(service_urls=['translate.google.cn'])
    mem_words = []
    for w in words:
        g_trans = translator.translate(w)
        mem_word = MemWord(w, g_trans, 4)
        mem_words.append(mem_word)
    return mem_words

def FormatPrintDefinition(definition):
    if definition is None:
        return
    for d in definition:
        print("|||meaning|||:")
        print("{}: \n\t{}\neg: {}".format(d[0], d[1][0][0], d[1][0][2]))

def FormatPrintSynonyms(synonyms):
    if synonyms is None:
        return
    for s in synonyms:
        print("|||synonyms|||:")
        print("{}: \n\t{}".format(s[0], s[1][0][0]))

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("eg.: python {} vocabulary.html".format(sys.argv[0]))
        quit()
    html_filename = sys.argv[1]
    words = ExtractWordsFromHtmlVocabulary(html_filename)
    mem_words = LoadMemWords(words)
    while True:
        mem_words.sort(key=MemLevel , reverse=True)
        input(mem_words[0].word)
        if mem_words[0].definition is None and mem_words[0].synonyms is None:
            print(mem_words[0].definition_cn)
        else:
            FormatPrintDefinition(mem_words[0].definition)
            FormatPrintSynonyms(mem_words[0].synonyms)

        while True:
            try:    
                level = input("mem_level: ")
                print('\033c')
                mem_words[0].mem_level = int(level)
            except ValueError:
                pass
            else:
                break