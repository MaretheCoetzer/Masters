#!/usr/bin/env python
# coding: utf-8

# # 3D Array format
# 
# ## Physical properties
# ---------------------------------
# Properties:
# <br>[N],
#               <br>[Body Mass],[Femur 1 Mass],[Tibia 1 Mass],[Femur 2 Mass],[Tibia 2 Mass],[Femur 3 Mass],[Tibia 3 Mass],[Femur 4 Mass],[Tibia 4 Mass]
#               <br>[Body Length],[Femur 1 Length],[Tibia 1 Length],[Femur 2 Length],[Tibia 2 Length],[Femur 3 Length],[Tibia 3 Length],[Femur 4 Length],[Tibia 4 Length]
# 
# ## Torques 
# ----------------
# Torques:
# <br>[Torque Hip 1],[Torque Knee 1],[Torque Hip 2],[Torque Knee 2],
#            <br>[Torque Hip 3],[Torque Knee 3],[Torque Hip 4],[Torque Knee 4]
# 
# ## Angles, Velocities and GRFs 
# -----------------------------------------------
# Movement:
# <br>[X Angle],[Z Angle],[Hip 1 Angle],[Relative Knee 1 Angle],[Hip 2 Angle],[Relative Knee 2 Angle],[Hip 3 Angle],[Relative Knee 3 Angle],[Hip 4 Angle],[Relative Knee 4 Angle],
#            <br> [X Velocity],[Z Velocity],[Hip 1 Velocity],[Knee 1 Velocity],[Hip 2 Velocity],[Knee 2 Velocity],[Hip 3 Velocity],[Knee 3 Velocity],[Hip 4 Velocity],[Knee 4 Velocity],
#            <br> [GRF 1x],[GRF 2x],[GRF 3x],[GRF 4x],[GRF 1z],[GRF 2z],[GRF 3z],[GRF 4z]

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as ani
from IPython.display import HTML
from dataclasses import dataclass
import argparse

# Custom modules
import log
__logger = log.setup_custom_logger("trajectory_reader")

# The name of the result set for the trajectory reader to write
__default_result_name="2_feet_step_up"

# Small wrapper around our script options
@dataclass
class ScriptOptions:
    silent: bool = False
    result_name: str = None

# ---------------------------------------------------
# HELPER FUNCTIONS
# ---------------------------------------------------

def _get_result_dir(result_name):
    """
    Check if the result directory exists, throwing an exception if not. We expect
    this directory to have been created by step-up.py and from there we just read
    in the results and write out our animations.
    """
    script_path = __file__
    parent_dir = os.path.split(os.path.dirname(script_path))[0]
    __logger.info(f"Found parent dir={parent_dir}")

    # If we do not have the results directory bail out
    results_path = "{}/results/{}/".format(parent_dir, result_name)
    if not os.path.exists(results_path):
        __logger.error(f"Could not find result path {results_path} - exiting")
        raise Exception("Could not find results directory")

    __logger.info(f"Set results path to {results_path}")
    return results_path

def _load_results(result_dir):
    """
    Loads the results from a given result directory, returns an array of the
    CSV object
    """
    __logger.info(f"Loading results from {result_dir}")
    Properties =  _load_single_result(result_dir + "3D_Properties.csv")
    Movement =  _load_single_result(result_dir + "3D_col_ros.csv")
    Torque =  _load_single_result(result_dir + "3D_Nodal.csv")
    Ros =  _load_single_result(result_dir + "3D_col_traj.csv")
    return [Properties, Movement, Torque, Ros]


def _load_single_result(result_file_path):
    """
    Given a full path to a result .csv file loads it using pandas, first
    checking for its existence
    """
    if not os.path.exists(result_file_path):
        __logger.error(f"Could not find result file {result_file_path}")
        raise Exception("Result file not found")
    
    return pd.read_csv(result_file_path)

def _animate_results(Properties, Movement, Torque, Ros, result_dir, script_opts):
    """
    Animates the given csv results as a 2D & 3D gif. 
    @param result_data: A pandas csv array containing the results 
    @param result_dir: The directory to save the animated result(s) to
    @param script_opts: The ScriptOptions set for this run
    """

    # Animation stuff -------------------------------------------------------
    #-----------Print 3D---------------------
    fig2, ax2= plt.subplots(1,1) #create axes

    __logger.info(f"Animating 3D results - script_opts={script_opts}")

    # Assigning values
    N = Properties.iloc[0,0]
    l_b = Properties.iloc[0,10]
    l_w = Properties.iloc[0,11]
    l_f1 = Properties.iloc[0,12]
    l_t1 = Properties.iloc[0,13]
    step_height = Properties.iloc[0,20]
    distance_from_step = Properties.iloc[0,21]

    tau_h1 = Torque.iloc[:,1]
    tau_k1 = Torque.iloc[:,2]

    w_h1 = Ros.iloc[:,20]
    w_k1 = Ros.iloc[:,21]

    plt.plot(tau_h1)
    if script_opts.silent == False:
        plt.show()

    plt.plot(w_h1)
    if script_opts.silent == False:
        plt.show()

    x = Movement.iloc[:,0]
    end=len(x)-3
    x = Movement.iloc[0:end,0]
    y = Movement.iloc[0:end,1]
    z = Movement.iloc[0:end,2]
    th_bx = Movement.iloc[0:end,3]
    th_by = Movement.iloc[0:end,4]
    th_bz = Movement.iloc[0:end,5]
    th_h1 = Movement.iloc[0:end,6]
    th_k1 = Movement.iloc[0:end,7]
    th_h2 = Movement.iloc[0:end,8]
    th_k2 = Movement.iloc[0:end,9]
    th_h3 = Movement.iloc[0:end,10]
    th_k3 = Movement.iloc[0:end,11]
    th_h4 = Movement.iloc[0:end,12]
    th_k4 = Movement.iloc[0:end,13]
    t = Movement.iloc[0:end,14]

    for n in range (1,len(t)):
        t[n]=t[n]+t[n-1]

    x = np.concatenate((x,x),axis=None)
    y = np.concatenate((y,y),axis=None)
    z = np.concatenate((z,z),axis=None)
    th_bx = np.concatenate((th_bx,th_bx),axis=None)
    th_by = np.concatenate((th_by,th_by),axis=None)
    th_bz = np.concatenate((th_bz,th_bz),axis=None)
    th_h1 = np.concatenate((th_h1,th_h1),axis=None)
    th_k1 = np.concatenate((th_k1,th_k1),axis=None)
    th_h2 = np.concatenate((th_h2,th_h2),axis=None)
    th_k2 = np.concatenate((th_k2,th_k2),axis=None)
    th_h3 = np.concatenate((th_h3,th_h3),axis=None)
    th_k3 = np.concatenate((th_k3,th_k3),axis=None)
    th_h4 = np.concatenate((th_h4,th_h4),axis=None)
    th_k4 = np.concatenate((th_k4,th_k4),axis=None)

    # Animation stuff -------------------------------------------------------
    #-----------Print 3D---------------------
    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d') #create axes
    __logger.info("Plotting robot over 3D...")    
    #lambdify update function
    update = lambda i: _plot_robot_3d(i, ax1, th_bx,th_by, th_bz,
                                     th_h1, th_h2, th_h3, th_h4,
                                     th_k1, th_k2, th_k3, th_k4,
                                     l_b, l_w, l_f1, l_t1,
                                     x, y, z)

    __logger.info("Saving 3D animation result")
    animate = ani.FuncAnimation(fig1,update,range(1,(N+1)*3),interval = 50,repeat=False)
    # m.h[i].value*hm*100000
    animate.save(f'{result_dir}/Quad_3D.gif', writer='PillowWriter', fps=10)
    HTML(animate.to_jshtml())

    #lambdify update function
    __logger.info("Plotting robot over 2D...")
    update = lambda i: _plot_robot_2d(i,ax2, th_bx,th_by, th_bz,
                                     th_h1, th_h2, th_h3, th_h4,
                                     th_k1, th_k2, th_k3, th_k4,
                                     l_b, l_w, l_f1, l_t1,
                                     x, y, z, step_height, distance_from_step)

    animate = ani.FuncAnimation(fig2,update,range(1,(N+1)*3),interval = 50,repeat=False)
    # m.h[i].value*hm*100000
    animate.save(f'{result_dir}/Quad_XY.gif', writer='PillowWriter', fps=10)
    HTML(animate.to_jshtml())

def _plot_robot_3d(i, ax, th_bx, th_by, th_bz, 
               th_h1, th_h2, th_h3, th_h4, 
               th_k1, th_k2, th_k3, th_k4,
               l_b, l_w, l_f1, l_t1,
               x, y, z): #update function for animation
    """
    Does some plotting
    """
    ax.clear()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim(-0.3, 1.1) 
    ax.set_ylim(-0.4, 0.4)
    ax.set_zlim(-0.0, 0.7)

    rx=[[1, 0, 0], [0, np.cos(th_bx[i]), -np.sin(th_bx[i])], [0, np.sin(th_bx[i]), np.cos(th_bx[i])]]
    ry=[[np.cos(th_by[i]), 0, np.sin(th_by[i])], [0, 1, 0], [-np.sin(th_by[i]), 0, np.cos(th_by[i])]]
    rz=[[np.cos(th_bz[i]), -np.sin(th_bz[i]), 0], [np.sin(th_bz[i]), np.cos(th_bz[i]), 0], [0, 0, 1]]
    rxy=np.matmul(rx,ry)
    rxyz=np.matmul(rxy,rz)

    Hip11=[[0.5*l_b],[0.5*l_w],[0]]
    Hip22=[[0.5*l_b],[-0.5*l_w],[0]]
    Hip33=[[-0.5*l_b],[0.5*l_w],[0]]
    Hip44=[[-0.5*l_b],[-0.5*l_w],[0]]

    Knee11=[[0.5*l_b+l_f1*np.sin(th_h1[i])],[0.5*l_w],[-l_f1*np.cos(th_h1[i])]]
    Knee22=[[0.5*l_b+l_f1*np.sin(th_h2[i])],[-0.5*l_w],[-l_f1*np.cos(th_h2[i])]]
    Knee33=[[-0.5*l_b+l_f1*np.sin(th_h3[i])],[0.5*l_w],[-l_f1*np.cos(th_h3[i])]]
    Knee44=[[-0.5*l_b+l_f1*np.sin(th_h4[i])],[-0.5*l_w],[-l_f1*np.cos(th_h4[i])]]

    Foot11=[[0.5*l_b+l_f1*np.sin(th_h1[i])+l_t1*np.sin(th_k1[i])],[0.5*l_w],[-l_f1*np.cos(th_h1[i])-l_t1*np.cos(th_k1[i])]]
    Foot22=[[0.5*l_b+l_f1*np.sin(th_h2[i])+l_t1*np.sin(th_k2[i])],[-0.5*l_w],[-l_f1*np.cos(th_h2[i])-l_t1*np.cos(th_k2[i])]]
    Foot33=[[-0.5*l_b+l_f1*np.sin(th_h3[i])+l_t1*np.sin(th_k3[i])],[0.5*l_w],[-l_f1*np.cos(th_h3[i])-l_t1*np.cos(th_k3[i])]]
    Foot44=[[-0.5*l_b+l_f1*np.sin(th_h4[i])+l_t1*np.sin(th_k4[i])],[-0.5*l_w],[-l_f1*np.cos(th_h4[i])-l_t1*np.cos(th_k4[i])]]

    b = [[x[i]],[y[i]],[z[i]]]

    h1 = b + np.matmul(rxyz,Hip11)
    h2 = b + np.matmul(rxyz,Hip22)
    h3 = b + np.matmul(rxyz,Hip33)
    h4 = b + np.matmul(rxyz,Hip44)

    k1 = b + np.matmul(rxyz,Knee11)
    k2 = b + np.matmul(rxyz,Knee22)
    k3 = b + np.matmul(rxyz,Knee33)
    k4 = b + np.matmul(rxyz,Knee44)

    foot1= b + np.matmul(rxyz,Foot11)
    foot2= b + np.matmul(rxyz,Foot22)
    foot3= b + np.matmul(rxyz,Foot33)
    foot4= b + np.matmul(rxyz,Foot44)

    #plot body
    ax.plot([h1[0,0],h2[0,0]],[h1[1,0],h2[1,0]],[h1[2,0],h2[2,0]],color='xkcd:black')
    ax.plot([h1[0,0],h3[0,0]],[h1[1,0],h3[1,0]],[h1[2,0],h3[2,0]],color='xkcd:black')
    ax.plot([h4[0,0],h2[0,0]],[h4[1,0],h2[1,0]],[h4[2,0],h2[2,0]],color='xkcd:black')
    ax.plot([h3[0,0],h4[0,0]],[h3[1,0],h4[1,0]],[h3[2,0],h4[2,0]],color='xkcd:black')
    
    #Plot Femur 1
    ax.plot([h1[0,0],k1[0,0]],[h1[1,0],k1[1,0]],[h1[2,0],k1[2,0]],color='xkcd:blue')

    #Plot Femur 2
    ax.plot([h2[0,0],k2[0,0]],[h2[1,0],k2[1,0]],[h2[2,0],k2[2,0]],color='xkcd:green')

    #Plot Femur 3
    ax.plot([h3[0,0],k3[0,0]],[h3[1,0],k3[1,0]],[h3[2,0],k3[2,0]],color='xkcd:red')

    #Plot Femur 4
    ax.plot([h4[0,0],k4[0,0]],[h4[1,0],k4[1,0]],[h4[2,0],k4[2,0]],color='xkcd:purple')

    #Plot Tibia 1
    ax.plot([k1[0,0],foot1[0,0]],[k1[1,0],foot1[1,0]],[k1[2,0],foot1[2,0]],color='xkcd:black',linewidth=2)

    #Plot Tibia 2
    ax.plot([k2[0,0],foot2[0,0]],[k2[1,0],foot2[1,0]],[k2[2,0],foot2[2,0]],color='xkcd:black',linewidth=2)

    #Plot Tibia 3
    ax.plot([k3[0,0],foot3[0,0]],[k3[1,0],foot3[1,0]],[k3[2,0],foot3[2,0]],color='xkcd:black',linewidth=2)

    #Plot Tibia 4
    ax.plot([k4[0,0],foot4[0,0]],[k4[1,0],foot4[1,0]],[k4[2,0],foot4[2,0]],color='xkcd:black',linewidth=2)

def _plot_robot_2d(i, ax, th_bx, th_by, th_bz, 
               th_h1, th_h2, th_h3, th_h4, 
               th_k1, th_k2, th_k3, th_k4,
               l_b, l_w, l_f1, l_t1,
               x, y, z, step_height, distance_from_step): #update function for animation
    ax.clear()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_xlim(-0.3, 1.1) 
    ax.set_ylim(-0.4, 0.4)

    rx=[[1, 0, 0], [0, np.cos(th_bx[i]), -np.sin(th_bx[i])], [0, np.sin(th_bx[i]), np.cos(th_bx[i])]]
    ry=[[np.cos(th_by[i]), 0, np.sin(th_by[i])], [0, 1, 0], [-np.sin(th_by[i]), 0, np.cos(th_by[i])]]
    rz=[[np.cos(th_bz[i]), -np.sin(th_bz[i]), 0], [np.sin(th_bz[i]), np.cos(th_bz[i]), 0], [0, 0, 1]]
    rxy=np.matmul(rx,ry)
    rxyz=np.matmul(rxy,rz)

    Hip11=[[0.5*l_b],[0.5*l_w],[0]]
    Hip22=[[0.5*l_b],[-0.5*l_w],[0]]
    Hip33=[[-0.5*l_b],[0.5*l_w],[0]]
    Hip44=[[-0.5*l_b],[-0.5*l_w],[0]]

    Knee11=[[0.5*l_b+l_f1*np.sin(th_h1[i])],[0.5*l_w],[-l_f1*np.cos(th_h1[i])]]
    Knee22=[[0.5*l_b+l_f1*np.sin(th_h2[i])],[-0.5*l_w],[-l_f1*np.cos(th_h2[i])]]
    Knee33=[[-0.5*l_b+l_f1*np.sin(th_h3[i])],[0.5*l_w],[-l_f1*np.cos(th_h3[i])]]
    Knee44=[[-0.5*l_b+l_f1*np.sin(th_h4[i])],[-0.5*l_w],[-l_f1*np.cos(th_h4[i])]]

    Foot11=[[0.5*l_b+l_f1*np.sin(th_h1[i])+l_t1*np.sin(th_k1[i])],[0.5*l_w],[-l_f1*np.cos(th_h1[i])-l_t1*np.cos(th_k1[i])]]
    Foot22=[[0.5*l_b+l_f1*np.sin(th_h2[i])+l_t1*np.sin(th_k2[i])],[-0.5*l_w],[-l_f1*np.cos(th_h2[i])-l_t1*np.cos(th_k2[i])]]
    Foot33=[[-0.5*l_b+l_f1*np.sin(th_h3[i])+l_t1*np.sin(th_k3[i])],[0.5*l_w],[-l_f1*np.cos(th_h3[i])-l_t1*np.cos(th_k3[i])]]
    Foot44=[[-0.5*l_b+l_f1*np.sin(th_h4[i])+l_t1*np.sin(th_k4[i])],[-0.5*l_w],[-l_f1*np.cos(th_h4[i])-l_t1*np.cos(th_k4[i])]]

    b = [[x[i]],[y[i]],[z[i]]]

    h1 = b + np.matmul(rxyz,Hip11)
    h2 = b + np.matmul(rxyz,Hip22)
    h3 = b + np.matmul(rxyz,Hip33)
    h4 = b + np.matmul(rxyz,Hip44)

    k1 = b + np.matmul(rxyz,Knee11)
    k2 = b + np.matmul(rxyz,Knee22)
    k3 = b + np.matmul(rxyz,Knee33)
    k4 = b + np.matmul(rxyz,Knee44)

    foot1= b + np.matmul(rxyz,Foot11)
    foot2= b + np.matmul(rxyz,Foot22)
    foot3= b + np.matmul(rxyz,Foot33)
    foot4= b + np.matmul(rxyz,Foot44)

    #plot body
    ax.plot([h1[0,0],h2[0,0]],[h1[2,0],h2[2,0]],color='xkcd:black')
    ax.plot([h1[0,0],h3[0,0]],[h1[2,0],h3[2,0]],color='xkcd:black')
    ax.plot([h4[0,0],h2[0,0]],[h4[2,0],h2[2,0]],color='xkcd:black')
    ax.plot([h3[0,0],h4[0,0]],[h3[2,0],h4[2,0]],color='xkcd:black')
    
    #Plot Femur 1
    ax.plot([h1[0,0],k1[0,0]],[h1[2,0],k1[2,0]],color='xkcd:blue',linewidth=3)

    #Plot Femur 2
    ax.plot([h2[0,0],k2[0,0]],[h2[2,0],k2[2,0]],color='xkcd:green',linewidth=3)

    #Plot Femur 3
    ax.plot([h3[0,0],k3[0,0]],[h3[2,0],k3[2,0]],color='xkcd:red',linewidth=3)

    #Plot Femur 4
    ax.plot([h4[0,0],k4[0,0]],[h4[2,0],k4[2,0]],color='xkcd:purple',linewidth=3)

    #Plot Tibia 1
    ax.plot([k1[0,0],foot1[0,0]],[k1[2,0],foot1[2,0]],color='xkcd:black',linewidth=3)

    #Plot Tibia 2
    ax.plot([k2[0,0],foot2[0,0]],[k2[2,0],foot2[2,0]],color='xkcd:black',linewidth=3)

    #Plot Tibia 3
    ax.plot([k3[0,0],foot3[0,0]],[k3[2,0],foot3[2,0]],color='xkcd:black',linewidth=3)

    #Plot Tibia 4
    ax.plot([k4[0,0],foot4[0,0]],[k4[2,0],foot4[2,0]],color='xkcd:black',linewidth=3)

    #Plot surface
    ax.plot([-0.3,-l_b/2+distance_from_step,-l_b/2+distance_from_step,l_b/2+distance_from_step+0.4],[0,0,step_height,step_height])

def _parse_options():
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-s", "--silent", action="store_true", help="Whether to supress popups")
    argParser.add_argument("-r", "--result", type=str, help="The name of the result to process", default=__default_result_name)
    args = argParser.parse_args()
    __logger.info(f"Parsed arguments: {args}")
    return ScriptOptions(args.silent, args.result)

# ---------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------

def gen_trajectories(script_opts):
    _result_dir = _get_result_dir(script_opts.result_name)
    Properties, Movement, Torque, Ros = _load_results(_result_dir)
    _animate_results(Properties, Movement, Torque, Ros, _result_dir, script_opts)

if __name__ == "__main__":
    script_opts = _parse_options()
    gen_trajectories(script_opts)