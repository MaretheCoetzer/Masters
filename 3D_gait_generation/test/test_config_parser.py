# Have to refer to the parent path to access the module
# NB need test_setup first as it pulls the src path into our root path
# allowing the unit tests to find all our modules
import test_setup
from src.config_parser import RunConfig
from src.config_parser import IpoptConfig
from src.config_parser import StairClimbParams
import src.config_parser as cfg
from src.constants import Dimension

import unittest

class TestConfigAndParser(unittest.TestCase):

    def setUp(self):
        self.run_config = RunConfig("test-result", "uint test run", 'DEBUG', 99,
                           0.5, 6, "walk", 0.2, "yes",Dimension["TWO_D"], "unit-test-uuid",
                           IpoptConfig(5, 100, 25, 0.05),
                           StairClimbParams(0.0, 0.0, 0.0, 0.0))

    #
    # We should generate a config
    # and the correct paths
    def test_config_construction(self):
        self.assertEqual(self.run_config.get_result_parent_dir(), "../results/test-result.unit-test-uuid/",  "Should generate correct result path")
        self.assertEqual(self.run_config.get_result_path("new-result.csv"), "../results/test-result.unit-test-uuid/new-result.csv", "Should generate correct result path")

    #
    # We should correct parse a given config
    #
    def test_correct_parsing(self):
        parser = cfg.parse_config("./test-run-config.json")
        self.assertEqual(parser.result_name, "2_feet_step_up")
        self.assertEqual(parser.log_level, "INFO")
        self.assertEqual(parser.stair_climb_params.stair_height, 0.3)
        self.assertEqual(parser.stair_climb_params.step_length, 0.2)
        self.assertEqual(parser.stair_climb_params.distance_from_step, 0.1)
        self.assertEqual(parser.stair_climb_params.clearance_x, 0.0)
        self.assertEqual(parser.num_nodes, 40)
        self.assertEqual(parser.ipopt_config.print_level, 5)
        self.assertEqual(parser.ipopt_config.max_iter, 100000)
        self.assertEqual(parser.ipopt_config.max_cpu_time, 120000)
        self.assertEqual(parser.ipopt_config.tolerance, 0.000001)
        self.assertEqual(parser.dimension, Dimension.TWO_D)