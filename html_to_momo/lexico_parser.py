from mem_word import MemWord

if __name__ == "__main__":
    results = MemWord.Vocabulary(['hello', 'ripple'])
    for w in results:
        print(w.origin)
        # print(len(w.pos_blocks))
        for b in w.pos_blocks:
            print(b)
    # print(result.)

