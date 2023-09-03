# All things to do with the concrete model

# MODULES
from pyomo.environ import*
import constants
import log
from constants import body_component as bd

#
# Constructs an initial state for the pyomo concrete model
# @param logger: A logger to use in generating this concrete model
# return: pyomo ConcreteModel
def get_concrete_model(logger):
    concrete_model = ConcreteModel()

    #Servo Motor Specs
    concrete_model.Tmax = 6.864655 #Nm, corresponding to a 8V input - CALLEN changed this back, maybe high values is wrong
    concrete_model.Wmax = 8.055366 #rad/s, corresponding to a 8V input

    #Travel distance
    concrete_model.distance = 0.005 # 0.07 #m
    concrete_model.vel = 0.1 #m/s

    concrete_model.N = RangeSet(constants.N)
    concrete_model.cN = RangeSet(constants.cN)
    concrete_model.L = Set(dimen=2, initialize = get_model_links())
    concrete_model.DOF = Set(initialize = constants.get_degrees_of_freedom())

    # system coordinates
    concrete_model.q = Var(constants.N, constants.cN, concrete_model.DOF) # position
    concrete_model.dq = concrete_model.q # velocity
    concrete_model.ddq = concrete_model.q # acceleration
    concrete_model.q0 = Var(constants.N, concrete_model.DOF) # position
    concrete_model.dq0 = Var(constants.N, concrete_model.DOF) # velocity
    concrete_model.tt = Var(constants.N, constants.cN)
    concrete_model.tt0 = Var(constants.N, bounds = constants.time_between_nodes)
    concrete_model.h = Var(constants.N, bounds =(constants.tmin,constants.tmax))
    concrete_model.g = Param(initialize = 9.81)

    concrete_model.m = Param(concrete_model.L, initialize = get_model_links())
    mbody = sum(concrete_model.m[l] for l in get_model_links())
    mBW = mbody*concrete_model.g.value
    logger.info(f"Total body mass={mbody}, body weight={mBW}")

    return concrete_model

# Return: A multidimensional Set of body component and order
def get_model_links():
    return Set(dimen=2, initialize = [(bd.BODY,1),
                                      (bd.BODY_WIDTH,1),
                                      (bd.LEG_RADIUS,1),
                                      (bd.FEMUR,1),
                                      (bd.TIBIA,1),
                                      (bd.FEMUR,2),
                                      (bd.TIBIA,2),
                                      (bd.FEMUR,3),
                                      (bd.TIBIA,3),
                                      (bd.FEMUR,4),
                                      (bd.TIBIA,4)])

# Returns the mass of said component 
def get_component_mass(n, lb, ln):
    match lb:
        case bd.FEMUR:
            return constants.FemurMass
        case bd.BODY:
            return constants.BodyMass
        case _:
            return constants.TibiaMass