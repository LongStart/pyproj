import sys
from html_to_momo import ExtractWordsFromHtmlVocabulary
#from googletrans import Translator


def MemLevel(cls):
        return cls.mem_level

def UpdateMemWords(mem_words):
    for w in mem_words:
        w.mem_level += 0.1
    mem_words.sort(key=MemLevel , reverse=True)

if __name__ == "__main__":
    from mem_vocabulary import ConcurrentInitVocabulary
    if len(sys.argv) < 2:
        print("eg.: python {} vocabulary.html".format(sys.argv[0]))
        quit()
    html_filename = sys.argv[1]
    words = ExtractWordsFromHtmlVocabulary(html_filename)
    mem_words = ConcurrentInitVocabulary(words)
    #mem_words = LoadMemWords(words)
    while True:
        UpdateMemWords(mem_words)
        input((mem_words[0].word).center(100))
        print(mem_words[0])
        while True:
            try:
                level = input("mem_level: ")
                print('\033[H\033[J')
                print('\n' * 10)
                mem_words[0].mem_level = float(level)
            except ValueError:
                pass
            else:
                break
