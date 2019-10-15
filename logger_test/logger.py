import logging

if __name__ == "__main__":
    logging.basicConfig(filename='ddd.log', format='%(asctime)s - %(message)s', level=logging.INFO)
    logging.info('This is an info message')
    logging.critical('This is a critical message')
