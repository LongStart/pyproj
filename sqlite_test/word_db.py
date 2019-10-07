import sqlite3
import os

class SQLGlossary(object):
    def __init__(self, db_name='.glossary'):
        self.table_name = 'glossary'
        self.db_name = db_name
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        
        cursor.execute(''' SELECT count(name) FROM sqlite_master WHERE type='table' AND name='{}' '''.format(self.table_name))
        if cursor.fetchone()[0] == 1 : 
            print('Table exists.')
        else:
            cursor.execute('''CREATE TABLE glossary
                (WORD TEXT PRIMARY KEY NOT NULL,
                DEFINITION  BLOB     NOT NULL,
                MEM_LEVEL   REAL     NOT NULL,
                PRONOUNCE   BLOB);''')
        conn.commit()
        conn.close()

    def InsertWord(self,word, definition, audio, mem):
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute("INSERT INTO glossary (WORD,DEFINITION,MEM_LEVEL) \
            VALUES (?, ?, 3.00)",  [word, sqlite3.Binary(definition)])    
        conn.commit()
        conn.close()

    def UpdateMemLevel(self, word, mem_level):
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute("UPDATE glossary set MEM_LEVEL=? where WORD=? ",  [mem_level, word])    
        conn.commit()
        conn.close()
        

if __name__ == "__main__":
    db_name = 'test.db'

    bin_data = None
    with open('test.bin', 'rb') as f:
        bin_data = f.read()

    db = SQLGlossary() 
    db.InsertWord('happy', bin_data, 'asdf', 0.)
    db.InsertWord('unhappy', bin_data, 'asdf', 0.)
    db.UpdateMemLevel('happy', 3.4)

