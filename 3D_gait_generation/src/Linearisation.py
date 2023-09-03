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

# Custom modules
# import log
# logger = log.setup_custom_logger()

# load arrays
Properties = pd.read_csv("../results/Step_up_hind.eba5216b-fe11-4404-9f17-8e4d565f873b/3D_Properties.csv")
Movement = pd.read_csv("../results/Step_up_hind.eba5216b-fe11-4404-9f17-8e4d565f873b/3D_col_ros.csv")

# Assigning values
step=0.01 #s

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

for n in range (1,len(t)):
    t[n]=t[n]+t[n-1]

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

plt.plot(ss_t,SS_H1,t,th_h1)
plt.title('Hip 1 linearised vs trajectory joint angles')
plt.xlabel('Time (s)')
plt.ylabel('Joint angle (rad)')
plt.legend(['Linearised data','Trajectory data'])
plt.show()

plt.plot(ss_t,SS_K1,t,th_k1)
plt.title('Knee 1 linearised vs trajectory joint angles')
plt.xlabel('Time (s)')
plt.ylabel('Joint angle (rad)')
plt.legend(['Linearised data','Trajectory data'])
plt.show()

print(f'Interpolated Joint size: {len(SS_BY)}, Interpolated t size: {len(ss_t)}')
print(f"hip[0]: {SS_BY[0]}, t[0]: {ss_t[0]}, hip[N]: {SS_BY[len(SS_BY)-1]}, t[N]: {ss_t[len(SS_BY)-1]}")
print(f'Original Joint size: {len(th_by)}, Original t size: {len(t)}')
print(f"hip[0]: {th_by[0]}, t[0]: {t[0]}, hip[N]: {th_by[len(th_by)-1]}, t[N]: {t[len(th_by)-1]}\n")

np.set_string_function(lambda x: repr(x).replace('(', '').replace(')', '').replace('array', '').replace("       ", ' ').replace('[','{').replace(']','}').replace('\n','') , repr=False)
print(f'float SS_servo0[] = {SS_H1};\nfloat SS_servo1[] = {SS_K1};\nfloat SS_servo2[] = {SS_H2};\nfloat SS_servo3[] = {SS_K2};\nfloat SS_servo4[] = {SS_H3};\nfloat SS_servo5[] = {SS_K3};\nfloat SS_servo6[] = {SS_H4};\nfloat SS_servo7[] = {SS_K4};')

# print(f'SS_K1:{SS_K1_Interpolated}')

