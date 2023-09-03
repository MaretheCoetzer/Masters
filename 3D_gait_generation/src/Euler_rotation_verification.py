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


def _animate_results():
    """
    Animates the given csv results as a 2D & 3D gif. 
    @param result_data: A pandas csv array containing the results 
    @param result_dir: The directory to save the animated result(s) to
    @param script_opts: The ScriptOptions set for this run
    """

    # Animation stuff -------------------------------------------------------
    #-----------Print 3D---------------------
    fig2, ax2= plt.subplots(1,1) #create axes

    # Assigning values
    N = 1
    l_b = 0.3
    l_w = 0.15
    l_f1 = 0.125
    l_t1 = 0.125
    step_height = 0.0
    distance_from_step = 0.0

    x = 0.0
    y = 0.0
    z = 0.0
    th_bx = 0.0
    th_by = 0.0
    th_bz = 0.0
    th_h1 = 0.2
    th_k1 = 0.2
    th_h2 = 0.2
    th_k2 = 0.2
    th_h3 = 0.2
    th_k3 = 0.2
    th_h4 = 0.2
    th_k4 = 0.2

    # Animation stuff -------------------------------------------------------
    #-----------Print 3D---------------------
    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d') 
    #lambdify update function
    _plot_robot_3d( ax1, th_bx,th_by, th_bz,
                                     th_h1, th_h2, th_h3, th_h4,
                                     th_k1, th_k2, th_k3, th_k4,
                                     l_b, l_w, l_f1, l_t1,
                                     x, y, z)


    #lambdify update function
    _plot_robot_2d(ax2, th_bx,th_by, th_bz,
                                     th_h1, th_h2, th_h3, th_h4,
                                     th_k1, th_k2, th_k3, th_k4,
                                     l_b, l_w, l_f1, l_t1,
                                     x, y, z, step_height, distance_from_step)


def _plot_robot_3d( ax, th_bx, th_by, th_bz, 
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
    # ax.set_xlim(-0.3, 1.1) 
    # ax.set_ylim(-0.4, 0.4)
    # ax.set_zlim(-0.0, 0.7)

    rx=[[1, 0, 0], [0, np.cos(th_bx), -np.sin(th_bx)], [0, np.sin(th_bx), np.cos(th_bx)]]
    ry=[[np.cos(th_by), 0, np.sin(th_by)], [0, 1, 0], [-np.sin(th_by), 0, np.cos(th_by)]]
    rz=[[np.cos(th_bz), -np.sin(th_bz), 0], [np.sin(th_bz), np.cos(th_bz), 0], [0, 0, 1]]
    rxy=np.matmul(rx,ry)
    rxyz=np.matmul(rxy,rz)

    Hip11=[[0.5*l_b],[0.5*l_w],[0]]
    Hip22=[[0.5*l_b],[-0.5*l_w],[0]]
    Hip33=[[-0.5*l_b],[0.5*l_w],[0]]
    Hip44=[[-0.5*l_b],[-0.5*l_w],[0]]

    Knee11=[[0.5*l_b+l_f1*np.sin(th_h1)],[0.5*l_w],[-l_f1*np.cos(th_h1)]]
    Knee22=[[0.5*l_b+l_f1*np.sin(th_h2)],[-0.5*l_w],[-l_f1*np.cos(th_h2)]]
    Knee33=[[-0.5*l_b+l_f1*np.sin(th_h3)],[0.5*l_w],[-l_f1*np.cos(th_h3)]]
    Knee44=[[-0.5*l_b+l_f1*np.sin(th_h4)],[-0.5*l_w],[-l_f1*np.cos(th_h4)]]

    Foot11=[[0.5*l_b+l_f1*np.sin(th_h1)+l_t1*np.sin(th_k1)],[0.5*l_w],[-l_f1*np.cos(th_h1)-l_t1*np.cos(th_k1)]]
    Foot22=[[0.5*l_b+l_f1*np.sin(th_h2)+l_t1*np.sin(th_k2)],[-0.5*l_w],[-l_f1*np.cos(th_h2)-l_t1*np.cos(th_k2)]]
    Foot33=[[-0.5*l_b+l_f1*np.sin(th_h3)+l_t1*np.sin(th_k3)],[0.5*l_w],[-l_f1*np.cos(th_h3)-l_t1*np.cos(th_k3)]]
    Foot44=[[-0.5*l_b+l_f1*np.sin(th_h4)+l_t1*np.sin(th_k4)],[-0.5*l_w],[-l_f1*np.cos(th_h4)-l_t1*np.cos(th_k4)]]

    b = [[x],[y],[z]]

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

    # plt.show()

def _plot_robot_2d(ax, th_bx, th_by, th_bz, 
               th_h1, th_h2, th_h3, th_h4, 
               th_k1, th_k2, th_k3, th_k4,
               l_b, l_w, l_f1, l_t1,
               x, y, z, step_height, distance_from_step): #update function for animation
    ax.clear()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    # ax.set_xlim(-0.3, 1.1) 
    # ax.set_ylim(-0.4, 0.4)

    rx=[[1, 0, 0], [0, np.cos(th_bx), -np.sin(th_bx)], [0, np.sin(th_bx), np.cos(th_bx)]]
    ry=[[np.cos(th_by), 0, np.sin(th_by)], [0, 1, 0], [-np.sin(th_by), 0, np.cos(th_by)]]
    rz=[[np.cos(th_bz), -np.sin(th_bz), 0], [np.sin(th_bz), np.cos(th_bz), 0], [0, 0, 1]]
    rxy=np.matmul(rx,ry)
    rxyz=np.matmul(rxy,rz)

    Hip11=[[0.5*l_b],[0.5*l_w],[0]]
    Hip22=[[0.5*l_b],[-0.5*l_w],[0]]
    Hip33=[[-0.5*l_b],[0.5*l_w],[0]]
    Hip44=[[-0.5*l_b],[-0.5*l_w],[0]]

    Knee11=[[0.5*l_b+l_f1*np.sin(th_h1)],[0.5*l_w],[-l_f1*np.cos(th_h1)]]
    Knee22=[[0.5*l_b+l_f1*np.sin(th_h2)],[-0.5*l_w],[-l_f1*np.cos(th_h2)]]
    Knee33=[[-0.5*l_b+l_f1*np.sin(th_h3)],[0.5*l_w],[-l_f1*np.cos(th_h3)]]
    Knee44=[[-0.5*l_b+l_f1*np.sin(th_h4)],[-0.5*l_w],[-l_f1*np.cos(th_h4)]]

    Foot11=[[0.5*l_b+l_f1*np.sin(th_h1)+l_t1*np.sin(th_k1)],[0.5*l_w],[-l_f1*np.cos(th_h1)-l_t1*np.cos(th_k1)]]
    Foot22=[[0.5*l_b+l_f1*np.sin(th_h2)+l_t1*np.sin(th_k2)],[-0.5*l_w],[-l_f1*np.cos(th_h2)-l_t1*np.cos(th_k2)]]
    Foot33=[[-0.5*l_b+l_f1*np.sin(th_h3)+l_t1*np.sin(th_k3)],[0.5*l_w],[-l_f1*np.cos(th_h3)-l_t1*np.cos(th_k3)]]
    Foot44=[[-0.5*l_b+l_f1*np.sin(th_h4)+l_t1*np.sin(th_k4)],[-0.5*l_w],[-l_f1*np.cos(th_h4)-l_t1*np.cos(th_k4)]]

    b = [[x],[y],[z]]

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
    plt.show()

# ---------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------

def gen_trajectories():
    _animate_results()

if __name__ == "__main__":
    gen_trajectories()