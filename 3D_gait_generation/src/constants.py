# A file for global constants used in the StepUp program

# MODULES
from enum import Enum

# 2D/3D class selector
class Dimension(Enum):
    TWO_D = 0
    THREE_D = 1

#========================================================================================
### Variable specific constants
# +
#Node Points
cN = 3

#Mass (kg)
FemurMass = 0.2
TibiaMass = 0.3   
BodyMass = 1.5

#Lengths (meters)
FemurLength = 0.115
TibiaLength = 0.12
BodyLength = 0.335

#Time Bounds
tmax = 0.1 #s 
tmin = 0.01 #s
hm = 1 #factor

co_location_a = [[0.19681547722366, 0.39442431473909, 0.37640306270047],
     [-0.06553542585020, 0.29207341166523, 0.51248582618842],
     [0.02377097434822, -0.04154875212600, 0.11111111111111]]

#========================================================================================
### Method specific constants
# Return: A list of degrees of freedoms
DOFs = ['x','y','z','theta_bx','theta_by','theta_bz','theta_h1','theta_k1','theta_h2','theta_k2','theta_h3','theta_k3','theta_h4','theta_k4']

BodyWidth = [0.0, 0.172]  #[2D,3D]
LegRadius = [0.0, 0.06] #[2D,3D]
def get_body_width(cord_type):
    return BodyWidth[cord_type]

def get_leg_radius(cord_type):
    return LegRadius[cord_type]