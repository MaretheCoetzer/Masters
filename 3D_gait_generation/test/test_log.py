# Have to refer to the parent path to access the module
# NB need test_setup first as it pulls the src path into our root path
# allowing the unit tests to find all our modules
import test_setup
from src.log import setup_custom_logger

import unittest
import os

class TestLog(unittest.TestCase):

    def setUp(self):
        self.__logger = setup_custom_logger("unit-test-logger", "INFO")

    # We should have created the test file after a log line
    def write_to_file(self):
        self.__logger.info("THIS IS A TEST")
        self.assertEqual(os.path.exists("./unit-test-logger.log"), True, "log file should exist")

        
    def test_skip_debug_logging(self):
        self.__logger.debug("THIS IS A DEBUG TEST")