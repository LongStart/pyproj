from bs4 import BeautifulSoup
from making_web_request import simple_get
# import sys

if __name__ == '__main__':
    raw_html = simple_get('http://www.66rpg.com/game/995198')
    soup = BeautifulSoup(raw_html, 'html.parser')
    # test = soup.find('div', attrs={'class': 'heart-count'})
    print(soup)
    # print(len(test))
    # for div in html.find('div', attrs={'class': 'heart-count'}):
    #     print(div)
        