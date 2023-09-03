# Os level imperatives
import shutil
import os

import json
import uuid
from dataclasses import dataclass
from enum import Enum
import constants

@dataclass
class IpoptConfig:
    print_level: int
    max_iter: int
    max_cpu_time: int
    tolerance: float

@dataclass
class StairClimbParams:
    stair_height: float
    step_length: float
    distance_from_step: float
    clearance_x: float    # Allowed +-error in step position location

# Small wrapper for our run configuration
@dataclass
class RunConfig:
    result_name: str
    run_description: str
    log_level: str
    num_nodes: int
    clearance_z: float  # minimum clearance of foot lift above ground in step
    iterations: int # Amount of times IPOPT eps value is divided by 10 for increased precision
    movement_action: str # Choose from available gaits: "walk, step-up", code is set up to walk, by default.
    travel_distance: float # min=travel_distance, max=2*travel_distance of body coordinate x
    refine: str # Full UUID that needs to be refined, otherwise its starts the solver from scratch 
    dimension: constants.Dimension # either TWO_D or THREE_D
    unique_run_id: str # uniquely identifies a specific run apart from the result name, allowing multiple runs of the same name
    ipopt_config: IpoptConfig
    stair_climb_params: StairClimbParams

    def get_result_parent_dir(self):
        return "../results/" + self.result_name + "." + self.unique_run_id + "/"

    def get_result_path(self, result_file):
        """
        Given a result_name (for this entire run) and the specific result file name,
        returns the full path for where this file should go
        """
        return self.get_result_parent_dir() + result_file 
    
    def get_full_result_name(self):
        return self.result_name+ "." + self.unique_run_id
    
    def should_walk(self):
        return self.movement_action=="walk"
    
    def should_step_up_front(self):
        return self.movement_action=="step-up-front"
    
    def should_step_up_hind(self):
        return self.movement_action=="step-up-hind"

    def get_model_path(self):
        return "../results/" + self.refine +"/model.pkl"
    # ---------------------------------------------------
    # HELPER FUNCTIONS
    # Some of these are poorly name - we will fix this!
    # ---------------------------------------------------
    def pre_run_setup(self):
        self.generate_result_directory()
        self.copy_run_config()
        self.generate_readme()

    def generate_result_directory(self):
        parent_dir = self.get_result_parent_dir()
        os.makedirs(parent_dir)
        
    def copy_run_config(self):
        """
        copies the current run config into our results directory
        """
        shutil.copy2('./run-config.json', self.get_result_parent_dir())

    def generate_readme(self):
        readme_path = self.get_result_parent_dir() + "README"
        with open(readme_path, "w") as new_file:
            new_file.write(self.run_description)

#
# Given the path to the config file sets our
# internal run config
#
def parse_config(config_file_path):
    file = open(config_file_path, mode="r")
    rawConfig = json.load(file)
    file.close()

    ipopt = IpoptConfig(rawConfig["ipopt_params"]["print_level"],
                        rawConfig["ipopt_params"]["max_iter"],
                        rawConfig["ipopt_params"]["max_cpu_time"],
                        rawConfig["ipopt_params"]["tolerance"])
    

    climb_params = get_stair_climb_params(rawConfig)
    run_config = RunConfig(rawConfig["result_name"] if 'result_name' in rawConfig else None,
                     rawConfig["run_description"] if 'run_description' in rawConfig else 'No description',
                     rawConfig["log_level"] if 'log_level' in rawConfig else 'DEBUG',
                     int(rawConfig["num_nodes"]) if 'num_nodes' in rawConfig else None,
                     float(rawConfig["clearance_z"]) if 'clearance_z' in rawConfig else None,
                     int(rawConfig["iterations"]) if 'iterations' in rawConfig else None,
                     str(rawConfig["movement_action"]) if 'movement_action' in rawConfig else "walk",
                     float(rawConfig["travel_distance"]) if 'travel_distance' in rawConfig else 0.05,
                     str(rawConfig["refine"]) if 'refine' in rawConfig else "no",
                     dimension=constants.Dimension[rawConfig["dimensionality"]] if 'dimensionality' in rawConfig else "THREE_D",
                     unique_run_id = str(uuid.uuid4()),
                     ipopt_config = ipopt,
                     stair_climb_params=climb_params)
    return run_config

def get_stair_climb_params(raw_config):
        if 'stair_climbing_params' in raw_config:
           return StairClimbParams(float(raw_config["stair_climbing_params"]["stair_height"]),
                                   float(raw_config["stair_climbing_params"]["step_length"]),
                                   float(raw_config["stair_climbing_params"]["distance_from_step"]),
                                   float(raw_config["stair_climbing_params"]["clearance_x"]))
        else:
            return StairClimbParams(0.0, 0.0, 0.0, 0.0)