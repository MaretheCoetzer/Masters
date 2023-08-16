#!/usr/bin/env python
# coding: utf-8

# # 2D Array format
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
# <br>[X Angle],[Z Angle],[Hip 1 Angle],[Knee 1 Angle],[Hip 2 Angle],[Knee 2 Angle],[Hip 3 Angle],[Knee 3 Angle],[Hip 4 Angle],[Knee 4 Angle],
#            <br> [X Velocity],[Z Velocity],[Hip 1 Velocity],[Knee 1 Velocity],[Hip 2 Velocity],[Knee 2 Velocity],[Hip 3 Velocity],[Knee 3 Velocity],[Hip 4 Velocity],[Knee 4 Velocity],
#            <br> [GRF 1x],[GRF 2x],[GRF 3x],[GRF 4x],[GRF 1z],[GRF 2z],[GRF 3z],[GRF 4z]

import numpy as np
import pandas as pd
from numpy import asarray
import matplotlib.pyplot as plt
import matplotlib.animation as ani
from IPython.display import HTML

trajectory_name = 'TwoD_SS_1'
base_path = '/Users/Marethe/Documents/GitHub/Masters/2D_Quad_SS_Walk/'

# load arrays
Properties = pd.read_csv(r'C:'+base_path+trajectory_name+'_Properties.csv')
Torque = pd.read_csv(r'C:'+base_path+trajectory_name+'_Torque.csv')
Movement = pd.read_csv(r'C:'+base_path+trajectory_name+'_col_ros.csv')

# Assigning values
step=0.01 #s

# ___________________________________Loading Values_______________________________________________
N = Properties.iloc[0,0]

x = Movement.iloc[:,0]
end=len(x)-3
x = Movement.iloc[0:end,0]
z = Movement.iloc[0:end,1]
th_by = Movement.iloc[0:end,2]
th_h1 = Movement.iloc[0:end,3]
th_k1 = Movement.iloc[0:end,4]
th_h2 = Movement.iloc[0:end,5]
th_k2 = Movement.iloc[0:end,6]
th_h3 = Movement.iloc[0:end,7]
th_k3 = Movement.iloc[0:end,8]
th_h4 = Movement.iloc[0:end,9]
th_k4 = Movement.iloc[0:end,10]
t = Movement.iloc[0:end,11]

for n in range (1,len(t)):
    t[n]=t[n]+t[n-1]

l_b = Properties.iloc[0,10]
l_f1 = Properties.iloc[0,11]
l_t1 = Properties.iloc[0,12]
l_f2 = Properties.iloc[0,13]
l_t2 = Properties.iloc[0,14]
l_f3 = Properties.iloc[0,15]
l_t3 = Properties.iloc[0,16]
l_f4 = Properties.iloc[0,17]
l_t4 = Properties.iloc[0,18]

Torque_h1 = Torque.iloc[:,0]
Torque_k1 = Torque.iloc[:,1]
Torque_h2 = Torque.iloc[:,2]
Torque_k2 = Torque.iloc[:,3]
Torque_h3 = Torque.iloc[:,4]
Torque_k3 = Torque.iloc[:,5]
Torque_h4 = Torque.iloc[:,6]
Torque_k4 = Torque.iloc[:,7]

# __________________________________Linearization of Data__________________________________________________
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
[Z,ss_t] = Interpolater(z,t,step)
[SS_BY,ss_t] = Interpolater(th_by,t,step)
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


# ________________________________________Torque Plots________________________________________
# f, axs = plt.subplots(2) #create axes

# axs[0].plot(Torque_h1)
# axs[0].plot(Torque_k1)
# axs[0].plot(Torque_h2)
# axs[0].plot(Torque_k2)
# axs[0].plot(Torque_h3)
# axs[0].plot(Torque_k3)
# axs[0].plot(Torque_h4)
# axs[0].plot(Torque_k4)
# axs[0].set_title("Torques")
# axs[0].legend(["Torque_h1", "Torque_k1","Torque_h2", "Torque_k2","Torque_h3", "Torque_k3","Torque_h4", "Torque_k4"],loc='upper right')


# _____________________________________________Quad Plotting Function________________________________________
def plot_robot(i,ax): #update function for animation
    ax.clear()
    ax.set_xlim([-0.5,1])
    ax.set_ylim([0.0,0.7])
    
   #plot body
    legb_xt = X[i] + 0.5*l_b*np.sin(SS_BY[i])
    legb_zt = Z[i] + 0.5*l_b*np.cos(SS_BY[i])
    legb_xb = X[i] - 0.5*l_b*np.sin(SS_BY[i])
    legb_zb = Z[i] - 0.5*l_b*np.cos(SS_BY[i])
    ax.plot([legb_xt,legb_xb],[legb_zt,legb_zb],color='xkcd:black')

    #Plot Femur 1
    legf1_xt = X[i] + 0.5*l_b*np.sin(SS_BY[i]) 
    legf1_zt = Z[i] + 0.5*l_b*np.cos(SS_BY[i])
    legf1_xb = X[i] + 0.5*l_b*np.sin(SS_BY[i]) - l_f1*np.sin(SS_H1[i])
    legf1_zb = Z[i] + 0.5*l_b*np.cos(SS_BY[i]) - l_f1*np.cos(SS_H1[i])
    ax.plot([legf1_xt,legf1_xb],[legf1_zt,legf1_zb],color='xkcd:blue')

    #Plot Tibia 1
    legt1_xb = X[i] + 0.5*l_b*np.sin(SS_BY[i]) - l_f1*np.sin(SS_H1[i]) - l_t1*np.sin(SS_K1[i])
    legt1_zb = Z[i] + 0.5*l_b*np.cos(SS_BY[i]) - l_f1*np.cos(SS_H1[i]) - l_t1*np.cos(SS_K1[i])
    ax.plot([legf1_xb,legt1_xb],[legf1_zb,legt1_zb],color='xkcd:black')

    #Plot Femur 2
    legf2_xt = X[i] + 0.5*l_b*np.sin(SS_BY[i]) 
    legf2_zt = Z[i] + 0.5*l_b*np.cos(SS_BY[i])
    legf2_xb = X[i] + 0.5*l_b*np.sin(SS_BY[i]) - l_f2*np.sin(SS_H2[i])
    legf2_zb = Z[i] + 0.5*l_b*np.cos(SS_BY[i]) - l_f2*np.cos(SS_H2[i])
    ax.plot([legf2_xt,legf2_xb],[legf2_zt,legf2_zb],color='xkcd:green')

    #Plot Tibia 2
    legt2_xb = X[i] + 0.5*l_b*np.sin(SS_BY[i]) - l_f2*np.sin(SS_H2[i]) - l_t2*np.sin(SS_K2[i])
    legt2_zb = Z[i] + 0.5*l_b*np.cos(SS_BY[i]) - l_f2*np.cos(SS_H2[i]) - l_t2*np.cos(SS_K2[i])
    ax.plot([legf2_xb,legt2_xb],[legf2_zb,legt2_zb],color='xkcd:black')

    #Plot Femur 3
    legf3_xt = X[i] - 0.5*l_b*np.sin(SS_BY[i]) 
    legf3_zt = Z[i] - 0.5*l_b*np.cos(SS_BY[i])
    legf3_xb = X[i] - 0.5*l_b*np.sin(SS_BY[i]) - l_f3*np.sin(SS_H3[i])
    legf3_zb = Z[i] - 0.5*l_b*np.cos(SS_BY[i]) - l_f3*np.cos(SS_H3[i])
    ax.plot([legf3_xt,legf3_xb],[legf3_zt,legf3_zb],color='xkcd:red')

    #Plot Tibia 3
    legt3_xb = X[i] - 0.5*l_b*np.sin(SS_BY[i]) - l_f3*np.sin(SS_H3[i]) - l_t3*np.sin(SS_K3[i])
    legt3_zb = Z[i] - 0.5*l_b*np.cos(SS_BY[i]) - l_f3*np.cos(SS_H3[i]) - l_t3*np.cos(SS_K3[i])
    ax.plot([legf3_xb,legt3_xb],[legf3_zb,legt3_zb],color='xkcd:black')
    
    #Plot Femur 4
    legf4_xt = X[i] - 0.5*l_b*np.sin(SS_BY[i]) 
    legf4_zt = Z[i] - 0.5*l_b*np.cos(SS_BY[i])
    legf4_xb = X[i] - 0.5*l_b*np.sin(SS_BY[i]) - l_f4*np.sin(SS_H4[i])
    legf4_zb = Z[i] - 0.5*l_b*np.cos(SS_BY[i]) - l_f4*np.cos(SS_H4[i])
    ax.plot([legf4_xt,legf4_xb],[legf4_zt,legf4_zb],color='xkcd:purple')

    #Plot Tibia 4
    legt4_xb = X[i] - 0.5*l_b*np.sin(SS_BY[i]) - l_f4*np.sin(SS_H4[i]) - l_t4*np.sin(SS_K4[i])
    legt4_zb = Z[i] - 0.5*l_b*np.cos(SS_BY[i]) - l_f4*np.cos(SS_H4[i]) - l_t4*np.cos(SS_K4[i])
    ax.plot([legf4_xb,legt4_xb],[legf4_zb,legt4_zb],color='xkcd:black')


# ___________________________________Videos of linearised gaits_____________________________________________
fig1, ax1 = plt.subplots(1,1) #create axes
#ax1.set_aspect('equal')       
update = lambda i: plot_robot(i,ax1) #lambdify update function

animate = ani.FuncAnimation(fig1,update,range(1,N+1),interval = 50,repeat=False)
# animate.save(r"C:"+base_path+trajectory_name+".gif", writer='PillowWriter', fps=10)
HTML(animate.to_jshtml())

# _____________________________________Stills of linearised gaits________________________________________________
fig2, ax2 = plt.subplots(1,1)

# Creates still_nr equally spaced still images of the gait
still_nr = 10
for i in np.linspace(0,len(SS_H1)-1,still_nr):
    plot_robot(int(i),ax2)
    plt.savefig(r"C:"+base_path+trajectory_name+"_"+str(int(i))+".png", transparent=True, bbox_inches='tight') #bbox_inches is used to remove excess white around figure
    



