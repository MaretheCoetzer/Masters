import logging
import sys
from datetime import datetime

default_logger_name="ToddlerStumblings"

#
# Global static function to setup a logger of given name
# which is then accessible across classes. In order to
# use said logger one simply adds the following to their
# module:
# import log
# my_modules_logger = logging.getLogger(logger_name)
# 
def setup_custom_logger(logger_name=default_logger_name, log_level = 'DEBUG', log_path = './'):
    formatter = logging.Formatter(fmt='[%(asctime)s.%(msecs)04d][%(levelname)s ][%(module)s] - %(message)s')
    formatter.datefmt = "%Y-%m-%dT%H:%M:%S%z"

    # Configure handlers to stdout and a file
    stdhandler = logging.StreamHandler()
    stdhandler.setFormatter(formatter)
    today = datetime.today().strftime('%Y-%m-%d')
    fileHandler = logging.FileHandler(f'{log_path}{logger_name}-{today}.log')
    fileHandler.setFormatter(formatter)

    logger = logging.getLogger(logger_name)
    logger.setLevel(logging._nameToLevel[log_level])
    logger.addHandler(stdhandler)
    logger.addHandler(fileHandler)

    # allow these to be redirected out to our log file
    #sys.stdout = StreamToLogger(logger.info)
    #sys.stderr = StreamToLogger(logger.error)

    return logger

# Return the current logger name so that we have a single definition for this
# that callers can use
# return: string of the logger name as such:
# katebush-2023-03-20.log
def get_log_name():
    today = datetime.today().strftime('%Y-%m-%d')
    full_logger_name = f'{default_logger_name}-{today}.log'
    return full_logger_name

class StreamToLogger(object):
    """
    Fake file-like stream object that redirects writes to a logger instance.
    Courtesy of https://stackoverflow.com/a/66209331
    """

    def __init__(self, logfct):
        self.logfct = logfct
        self.buf = []

    def write(self, msg):
        if msg.endswith('\n'):
            self.buf.append(msg.removesuffix('\n'))
            self.logfct(''.join(self.buf))
            self.buf = []
        else:
            self.buf.append(msg)

    def flush(self):
        pass