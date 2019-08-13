from lxml import html
import requests
import multiprocessing as mp

# page = requests.get('https://www.lexico.com/en/definition/eleemosynary')
# tree = html.fromstring(page.content)
# buyers = tree.xpath('//div[@class="senseInnerWrapper"]/p')
# print(buyers[0].text_content())

def LexicoWordOrigin(w):
    from lxml import html
    import requests

    page = requests.get('https://www.lexico.com/en/definition/' + w)
    tree = html.fromstring(page.content)
    # origin = tree.xpath('//div[@class="senseInnerWrapper"]/p')
    # origin = tree.xpath('//section[@class="gramb"]')
    origin = tree.xpath('//div[@class="trg"]')
    if len(origin) == 0:
        return ''
    return origin[0][1][0][0].text_content()

def ExtractMeaningExample(trg_node):
    meaning = trg_node.xpath('/p/span[@class="ind"]/text()')
    examples = trg_node.xpath('/div[@class="exg"]/div[@class="ex"]/em')
    return meaning, examples[0].text_content()


def ProcessFunc(w, q):
    origin = LexicoWordOrigin(w)
    q.put(origin)

if __name__ == "__main__":
    vocabulary = ['detection'] * 1
    q = mp.Queue()
    ps = []
    for w in vocabulary:
        p = mp.Process(target=ProcessFunc, args=(w,q))
        ps.append(p)
        p.start()

    for p in ps:
        p.join()

    print(q.get())


