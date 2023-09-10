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

import numpy as np
import pandas as pd
from numpy import asarray
import matplotlib.pyplot as plt
import matplotlib.animation as ani
from IPython.display import HTML
import pprint
import os

# Custom modules
import log
__logger = log.setup_custom_logger("3D_data_processing")

trajectory_name = 'SS_walk_0cm_z_clearance'
# Assigning values
step=0.01 #s

# _________________________________Helper functions____________________________
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

# __________________________________Loading Values_____________________________
path=_get_result_dir(trajectory_name)
[Properties,Movement,Torque,Ros]=_load_results(path)

N = Properties.iloc[0,0]

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

N = Properties.iloc[0,0]
l_b = Properties.iloc[0,10]
l_w = Properties.iloc[0,11]
l_f1 = Properties.iloc[0,12]
l_t1 = Properties.iloc[0,13]
step_height = Properties.iloc[0,20]
distance_from_step = Properties.iloc[0,21]

for n in range (1,len(t)):
    t[n]=t[n]+t[n-1]

#  ___________________________________Lineaerization of Data_________________
def Interpolater(joint,t,step):
    n = 1       #original joint and time step counter
    i = 1       #new linear array step counter
    nodes = len(t)-1
    linear_angle=np.array([])
    linear_angle=np.append(linear_angle,joint[0])
    linear_time=np.array([])
    linear_time=np.append(linear_time,t[0]) 
    linear_time=np.append(linear_time,step)                                                                                                                                                     
    while(linear_time[i]<t[nodes]):
        if (linear_time[i]<t[n]):
            new_angle=joint[n-1]+(linear_time[i]-t[n-1])*(joint[n]-joint[n-1])/(t[n]-t[n-1])
            linear_angle=np.append(linear_angle,new_angle)
            linear_time=np.append(linear_time,linear_time[i]+step)
            i+=1
        else:
            n+=1
    linear_time=linear_time[:-1]
    return linear_angle,linear_time

[X,ss_t] = Interpolater(x,t,step)
[Y,ss_t] = Interpolater(y,t,step)
[Z,ss_t] = Interpolater(z,t,step)
[SS_BY,ss_t] = Interpolater(th_by,t,step)
[SS_BX,ss_t] = Interpolater(th_bx,t,step)
[SS_BZ,ss_t] = Interpolater(th_bz,t,step)
[SS_H1,ss_t] = Interpolater(th_h1,t,step)
[SS_K1,ss_t] = Interpolater(th_k1,t,step)
[SS_H2,ss_t] = Interpolater(th_h2,t,step)
[SS_K2,ss_t] = Interpolater(th_k2,t,step)
[SS_H3,ss_t] = Interpolater(th_h3,t,step)
[SS_K3,ss_t] = Interpolater(th_k3,t,step)
[SS_H4,ss_t] = Interpolater(th_h4,t,step)
[SS_K4,ss_t] = Interpolater(th_k4,t,step)

# plt.plot(ss_t,SS_H1,t,th_h1)
# plt.title('Hip 1 linearised vs trajectory joint angles')
# plt.xlabel('Time (s)')
# plt.ylabel('Joint angle (rad)')
# plt.legend(['Linearised data','Trajectory data'])
# plt.show()

# plt.plot(ss_t,SS_K1,t,th_k1)
# plt.title('Knee 1 linearised vs trajectory joint angles')
# plt.xlabel('Time (s)')
# plt.ylabel('Joint angle (rad)')
# plt.legend(['Linearised data','Trajectory data'])
# plt.show()

print(f'Interpolated Joint size: {len(SS_BY)}, Interpolated t size: {len(ss_t)}')
print(f"hip[0]: {SS_BY[0]}, t[0]: {ss_t[0]}, hip[N]: {SS_BY[len(SS_BY)-1]}, t[N]: {ss_t[len(SS_BY)-1]}")
print(f'Original Joint size: {len(th_by)}, Original t size: {len(t)}')
print(f"hip[0]: {th_by[0]}, t[0]: {t[0]}, hip[N]: {th_by[len(th_by)-1]}, t[N]: {t[len(th_by)-1]}\n")

np.set_string_function(lambda x: repr(x).replace('(', '').replace(')', '').replace('array', '').replace("       ", ' ').replace('[','{').replace(']','}').replace('\n','') , repr=False)
print(f'float SS_servo0[] = {SS_H1};\nfloat SS_servo1[] = {SS_K1};\nfloat SS_servo2[] = {SS_H2};\nfloat SS_servo3[] = {SS_K2};\nfloat SS_servo4[] = {SS_H3};\nfloat SS_servo5[] = {SS_K3};\nfloat SS_servo6[] = {SS_H4};\nfloat SS_servo7[] = {SS_K4};')


# _______________________________2D Quad Plotting Function________________________
def TwoD_plot_robot(i,ax):
    ax.clear()
    ax.set_xlabel('x')
    ax.set_ylabel('z')
    ax.set_xlim(-0.3, 1.1) 
    ax.set_ylim(-0.0, 0.4)

    rx=[[1, 0, 0], [0, np.cos(SS_BX[i]), -np.sin(SS_BX[i])], [0, np.sin(SS_BX[i]), np.cos(SS_BX[i])]]
    ry=[[np.cos(SS_BY[i]), 0, np.sin(SS_BY[i])], [0, 1, 0], [-np.sin(SS_BY[i]), 0, np.cos(SS_BY[i])]]
    rz=[[np.cos(SS_BZ[i]), -np.sin(SS_BZ[i]), 0], [np.sin(SS_BZ[i]), np.cos(SS_BZ[i]), 0], [0, 0, 1]]
    rxy=np.matmul(rx,ry)
    rxyz=np.matmul(rxy,rz)

    Hip11=[[0.5*l_b],[0.5*l_w],[0]]
    Hip22=[[0.5*l_b],[-0.5*l_w],[0]]
    Hip33=[[-0.5*l_b],[0.5*l_w],[0]]
    Hip44=[[-0.5*l_b],[-0.5*l_w],[0]]

    Knee11=[[0.5*l_b+l_f1*np.sin(SS_H1[i])],[0.5*l_w],[-l_f1*np.cos(SS_H1[i])]]
    Knee22=[[0.5*l_b+l_f1*np.sin(SS_H2[i])],[-0.5*l_w],[-l_f1*np.cos(SS_H2[i])]]
    Knee33=[[-0.5*l_b+l_f1*np.sin(SS_H3[i])],[0.5*l_w],[-l_f1*np.cos(SS_H3[i])]]
    Knee44=[[-0.5*l_b+l_f1*np.sin(SS_H4[i])],[-0.5*l_w],[-l_f1*np.cos(SS_H4[i])]]

    Foot11=[[0.5*l_b+l_f1*np.sin(SS_H1[i])+l_t1*np.sin(SS_K1[i])],[0.5*l_w],[-l_f1*np.cos(SS_H1[i])-l_t1*np.cos(SS_K1[i])]]
    Foot22=[[0.5*l_b+l_f1*np.sin(SS_H2[i])+l_t1*np.sin(SS_K2[i])],[-0.5*l_w],[-l_f1*np.cos(SS_H2[i])-l_t1*np.cos(SS_K2[i])]]
    Foot33=[[-0.5*l_b+l_f1*np.sin(SS_H3[i])+l_t1*np.sin(SS_K3[i])],[0.5*l_w],[-l_f1*np.cos(SS_H3[i])-l_t1*np.cos(SS_K3[i])]]
    Foot44=[[-0.5*l_b+l_f1*np.sin(SS_H4[i])+l_t1*np.sin(SS_K4[i])],[-0.5*l_w],[-l_f1*np.cos(SS_H4[i])-l_t1*np.cos(SS_K4[i])]]

    b = [[X[i]],[Y[i]],[Z[i]]]

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
    ax.plot([h1[0,0],k1[0,0]],[h1[2,0],k1[2,0]],color='xkcd:blue')

    #Plot Femur 2
    ax.plot([h2[0,0],k2[0,0]],[h2[2,0],k2[2,0]],color='xkcd:green')

    #Plot Femur 3
    ax.plot([h3[0,0],k3[0,0]],[h3[2,0],k3[2,0]],color='xkcd:red')

    #Plot Femur 4
    ax.plot([h4[0,0],k4[0,0]],[h4[2,0],k4[2,0]],color='xkcd:purple')

    #Plot Tibia 1
    ax.plot([k1[0,0],foot1[0,0]],[k1[2,0],foot1[2,0]],color='xkcd:black')

    #Plot Tibia 2
    ax.plot([k2[0,0],foot2[0,0]],[k2[2,0],foot2[2,0]],color='xkcd:black')

    #Plot Tibia 3
    ax.plot([k3[0,0],foot3[0,0]],[k3[2,0],foot3[2,0]],color='xkcd:black')

    #Plot Tibia 4
    ax.plot([k4[0,0],foot4[0,0]],[k4[2,0],foot4[2,0]],color='xkcd:black')

    #Plot surface
    ax.plot([-0.3,-l_b/2+distance_from_step,-l_b/2+distance_from_step,l_b/2+distance_from_step+0.4],[0,0,step_height,step_height])

# _________________________________Cascade Gait Plotting Fucntion___________________________
def plot_robot_sequence(i,ax,step):
    ax.set_ylim([0.0,2])
    offset = l_b*2
    ax.set_xlabel('x')
    ax.set_ylabel('z')

    rx=[[1, 0, 0], [0, np.cos(SS_BX[i]), -np.sin(SS_BX[i])], [0, np.sin(SS_BX[i]), np.cos(SS_BX[i])]]
    ry=[[np.cos(SS_BY[i]), 0, np.sin(SS_BY[i])], [0, 1, 0], [-np.sin(SS_BY[i]), 0, np.cos(SS_BY[i])]]
    rz=[[np.cos(SS_BZ[i]), -np.sin(SS_BZ[i]), 0], [np.sin(SS_BZ[i]), np.cos(SS_BZ[i]), 0], [0, 0, 1]]
    rxy=np.matmul(rx,ry)
    rxyz=np.matmul(rxy,rz)

    Hip11=[[0.5*l_b],[0.5*l_w],[0]]
    Hip22=[[0.5*l_b],[-0.5*l_w],[0]]
    Hip33=[[-0.5*l_b],[0.5*l_w],[0]]
    Hip44=[[-0.5*l_b],[-0.5*l_w],[0]]

    Knee11=[[0.5*l_b+l_f1*np.sin(SS_H1[i])],[0.5*l_w],[-l_f1*np.cos(SS_H1[i])]]
    Knee22=[[0.5*l_b+l_f1*np.sin(SS_H2[i])],[-0.5*l_w],[-l_f1*np.cos(SS_H2[i])]]
    Knee33=[[-0.5*l_b+l_f1*np.sin(SS_H3[i])],[0.5*l_w],[-l_f1*np.cos(SS_H3[i])]]
    Knee44=[[-0.5*l_b+l_f1*np.sin(SS_H4[i])],[-0.5*l_w],[-l_f1*np.cos(SS_H4[i])]]

    Foot11=[[0.5*l_b+l_f1*np.sin(SS_H1[i])+l_t1*np.sin(SS_K1[i])],[0.5*l_w],[-l_f1*np.cos(SS_H1[i])-l_t1*np.cos(SS_K1[i])]]
    Foot22=[[0.5*l_b+l_f1*np.sin(SS_H2[i])+l_t1*np.sin(SS_K2[i])],[-0.5*l_w],[-l_f1*np.cos(SS_H2[i])-l_t1*np.cos(SS_K2[i])]]
    Foot33=[[-0.5*l_b+l_f1*np.sin(SS_H3[i])+l_t1*np.sin(SS_K3[i])],[0.5*l_w],[-l_f1*np.cos(SS_H3[i])-l_t1*np.cos(SS_K3[i])]]
    Foot44=[[-0.5*l_b+l_f1*np.sin(SS_H4[i])+l_t1*np.sin(SS_K4[i])],[-0.5*l_w],[-l_f1*np.cos(SS_H4[i])-l_t1*np.cos(SS_K4[i])]]

    b = [[X[i]+offset*step],[Y[i]],[Z[i]]]

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
    ax.plot([h1[0,0],k1[0,0]],[h1[2,0],k1[2,0]],color='xkcd:blue')

    #Plot Femur 2
    ax.plot([h2[0,0],k2[0,0]],[h2[2,0],k2[2,0]],color='xkcd:green')

    #Plot Femur 3
    ax.plot([h3[0,0],k3[0,0]],[h3[2,0],k3[2,0]],color='xkcd:red')

    #Plot Femur 4
    ax.plot([h4[0,0],k4[0,0]],[h4[2,0],k4[2,0]],color='xkcd:purple')

    #Plot Tibia 1
    ax.plot([k1[0,0],foot1[0,0]],[k1[2,0],foot1[2,0]],color='xkcd:blue')

    #Plot Tibia 2
    ax.plot([k2[0,0],foot2[0,0]],[k2[2,0],foot2[2,0]],color='xkcd:green')

    #Plot Tibia 3
    ax.plot([k3[0,0],foot3[0,0]],[k3[2,0],foot3[2,0]],color='xkcd:red')

    #Plot Tibia 4
    ax.plot([k4[0,0],foot4[0,0]],[k4[2,0],foot4[2,0]],color='xkcd:purple')

    #Plot surface
    # ax.plot([-0.3,-l_b/2+distance_from_step,-l_b/2+distance_from_step,l_b/2+distance_from_step+0.4],[0,0,step_height,step_height])


# _____________________________________Stills of linearised gaits________________________________________________
# Creates still_nr equally spaced still images of the gait
still_nr = 20

fig1, ax1 = plt.subplots(1,1)
update = lambda i: TwoD_plot_robot(i,ax1) #lambdify update function

animate = ani.FuncAnimation(fig1,update,range(0,len(SS_H1)),interval = 50,repeat=False)
animate.save(path+"../../../post_processing/image_sorting/"+trajectory_name+".gif", writer='PillowWriter', fps=10)
HTML(animate.to_jshtml())

__logger.info(f"Path: {path}")
result_path=path+"../../../post_processing/image_sorting/"
__logger.info(f"Results path: {result_path}")

fig2, ax2 = plt.subplots(1,1)
for i in np.linspace(0,len(SS_H1)-1,still_nr):
    TwoD_plot_robot(int(i),ax2)
    plt.title({int(i)})
    plt.savefig(path+"..\..\..\post_processing\image_sorting\\"+trajectory_name+"_"+str(int(i))+".png", transparent=True, bbox_inches='tight') #bbox_inches is used to remove excess white around figure

fig3, ax3 = plt.subplots(1,1)    
sequence = (0,8,16,24,30,39)
step=range(len(sequence))
for g in step:
    plot_robot_sequence(sequence[g],ax3,step[g])

plt.savefig(path+"..\..\..\post_processing\image_sorting\\"+trajectory_name+"_cascade.png", transparent=True, bbox_inches='tight', dpi=500) #bbox_inches is used to remove excess white around figure, dpi(dots per inch) image quality

# 2cm:
#0,4,6,10,14, 20,26,30,34,39
# 0,6,14,26,34,39

# 1cm:
# 0,6,14,26,32,39

# 0cm:
# 0,8,16,24,30,39
