from googletrans import Translator
import multiprocessing as mp

# ret = translator.translate("wane", dest='zh-cn')

# print(ret.text)
# print(ret.extra_data['definitions'][0])

def ProcessFunc(w, q):
    translator = Translator(service_urls=['translate.google.cn'])
    meaning = translator.translate(w, dest='zh-cn')
    # origin = LexicoWordOrigin(w)
    q.put(meaning)

if __name__ == "__main__":
    vocabulary = ['detection'] * 30

    q = mp.Queue()
    ps = []
    for w in vocabulary:
        p = mp.Process(target=ProcessFunc, args=(w,q))
        ps.append(p)
        p.start()

    for p in ps:
        p.join()

    print(q.qsize())