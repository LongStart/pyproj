from lxml import html
import requests

if __name__ == "__main__":
    from lxml import html
    import requests

    page = requests.get('https://www.lexico.com/en/definition/happy' + w)
    tree = html.fromstring(page.content)
    tree.xpath()


