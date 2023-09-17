#!/usr/bin/env python

# from std_msgs.msg import String
# from std_msgs.msg import Float64
import pandas as pd
import numpy as np
import time
import matplotlib.pyplot as plt

step = 0.01 #s

Properties = pd.read_csv("../results/3D_Properties.csv")
# SS = pd.read_csv("3D_col_ros.csv")
SS = pd.read_csv("../results/3D_col_ros.csv")

N = Properties.iloc[0,0] #Amount of nodes
l_b = Properties.iloc[0,10] #Body length
l_w = Properties.iloc[0,11] #Body width
l_f = Properties.iloc[0,12] #Femur length
l_t = Properties.iloc[0,13] #Tibia length

SS_h1 = SS.iloc[:,6]
SS_stop=len(SS_h1)-3
SS_by = SS.iloc[0:SS_stop,4]
SS_h1 = SS.iloc[0:SS_stop,6]
SS_k1 = SS.iloc[0:SS_stop,7]
SS_h2 = SS.iloc[0:SS_stop,8]
SS_k2 = SS.iloc[0:SS_stop,9]
SS_h3 = SS.iloc[0:SS_stop,10]
SS_k3 = SS.iloc[0:SS_stop,11]
SS_h4 = SS.iloc[0:SS_stop,12]
SS_k4 = SS.iloc[0:SS_stop,13]
SS_t = SS.iloc[0:SS_stop,14]

print(f"SS_h1[0]: {SS_h1[0]}")

# print(f'initial t size: {len(SS_t)}')
# print(f"{0} t={SS_t[0]}")
SS_t[0]=SS_t[0]
for n in range (1,len(SS_t)):
    SS_t[n]=SS_t[n]+SS_t[n-1]
    # print(f"{n} t={SS_t[n]}")
# print(f'Edited t size: {len(SS_t)}')

def Interpolater(joint,t,step):
    # print(f'initial hip array size: {size}')
    n = 1       #original joint and time step counter
    i = 1       #new linear array step counter
    nodes = len(t)-1
    # print(f"timesteps: {timesteps}")
    linear_angle=np.array([])
    linear_angle=np.append(linear_angle,joint[0])
    linear_time=np.array([])
    linear_time=np.append(linear_time,t[0]) 
    linear_time=np.append(linear_time,step)                                                                                                                                                     
    while(linear_time[i]<t[nodes]):
        if (linear_time[i]<t[n]):
            new_angle=joint[n-1]+(linear_time[i]-t[n-1])*(joint[n]-joint[n-1])/(t[n]-t[n-1])
            linear_angle=np.append(linear_angle,new_angle)
            # print(f"hip[n-1]: {joint[n-1]}, t[n-1]: {t[n-1]}")
            # print(f"hip[n]: {joint[n]}, t[n]: {t[n]}")
            # print(f'SS_time 1: {linear_time[i-1]} SS_Angle 1: {linear_angle[i-1]}')
            # print(f'SS_time 2: {linear_time[i]} SS_Angle 2: {linear_angle[i]}')
            linear_time=np.append(linear_time,linear_time[i]+step)
            i+=1
            # print(f"i: {i}, n: {n}")
        else:
            n+=1
            # print(f"n: {n}")
    # print(f'end time steps: {len(linear_time)}')
    linear_time=linear_time[:-1]
    return linear_angle,linear_time

[SS_BY,ss_t] = Interpolater(SS_by,SS_t,step)
[SS_H1,ss_t] = Interpolater(SS_h1,SS_t,step)
[SS_K1,ss_t] = Interpolater(SS_k1,SS_t,step)
[SS_H2,ss_t] = Interpolater(SS_h2,SS_t,step)
[SS_K2,ss_t] = Interpolater(SS_k2,SS_t,step)
[SS_H3,ss_t] = Interpolater(SS_h3,SS_t,step)
[SS_K3,ss_t] = Interpolater(SS_k3,SS_t,step)
[SS_H4,ss_t] = Interpolater(SS_h4,SS_t,step)
[SS_K4,ss_t] = Interpolater(SS_k4,SS_t,step)

print(f'Interpolated Joint size: {len(SS_H1)}, Interpolated t size: {len(ss_t)}')
print(f"hip[0]: {SS_H1[0]}, t[0]: {ss_t[0]}")
print(f"hip[1]: {SS_H1[1]}, t[1]: {ss_t[1]}")
print(f"hip[2]: {SS_H1[2]}, t[1]: {ss_t[2]}")

print(f'SS: Original: {len(SS_h1)},   Interpolated: {len(SS_H1)}')
print(f'Original Joint size: {len(SS_h1)}, Original t size: {len(SS_t)}')

# plt.plot(ss_t,SS_H1)
# plt.title('SS_H1 interpolated')
# plt.xlabel('time (s)')
# plt.ylabel('Angle')
# plt.show()

# plt.plot(np.linspace(0,1,len(SS_H1)),SS_H1)
fig1,ax1= plt.subplots(2)
ax1[0].plot(SS_H1)
ax1[1].plot(SS_K1)
plt.show()

#Parabola equation
# for i in range(0,11):
#     SS_H1[i]=SS_H1[i]+((10/180*np.pi)/5**2)*(i-5)**2-10/180*np.pi
#     SS_K1[i]=SS_K1[i]-((10/180*np.pi)/5**2)*(i-5)**2+10/180*np.pi
    
# fig2,ax2= plt.subplots(2)
# ax2[0].plot(SS_H1)
# ax2[1].plot(SS_K1)
# plt.show()

plt.plot(ss_t,SS_H1,SS_t,SS_h1)
plt.title('Hip 1 interpolated')
plt.xlabel('time (s)')
plt.ylabel('Angle')
plt.legend(['Interpolated','Original'])
plt.show()

plt.plot(ss_t,SS_K1,SS_t,SS_k1)
plt.title('knee 1 interpolated')
plt.xlabel('time (s)')
plt.ylabel('Angle')
plt.legend(['Knee','Hip'])
plt.show()

# plt.plot(SS_t,SS_k3,SS_t,SS_h3)
# plt.title('Leg 3 interpolated')
# plt.xlabel('time (s)')
# plt.ylabel('Angle')
# plt.legend(['Knee','Hip'])
# plt.show()

# plt.plot(SS_t,SS_k4,SS_t,SS_h4)
# plt.title('Leg 4 interpolated')
# plt.xlabel('time (s)')
# plt.ylabel('Angle')
# plt.legend(['Knee','Hip'])
# plt.show()


SS_H1_Interpolated = {'SS_H1':[SS_H1[i] for i in range(1,len(SS_H1))]}
SS_K1_Interpolated = {'SS_K1':[SS_K1[i] for i in range(1,len(SS_K1))]}
SS_H2_Interpolated = {'SS_H2':[SS_H2[i] for i in range(1,len(SS_H2))]}
SS_K2_Interpolated = {'SS_K2':[SS_K2[i] for i in range(1,len(SS_K2))]}
SS_H3_Interpolated = {'SS_H3':[SS_H3[i] for i in range(1,len(SS_H3))]}
SS_K3_Interpolated = {'SS_K3':[SS_K3[i] for i in range(1,len(SS_K3))]}
SS_H4_Interpolated = {'SS_H4':[SS_H4[i] for i in range(1,len(SS_H4))]}
SS_K4_Interpolated = {'SS_K4':[SS_K4[i] for i in range(1,len(SS_K4))]}
print(f'SS_H1:{SS_H1_Interpolated}')
print(f'SS_K1:{SS_K1_Interpolated}')
print(f'SS_H2:{SS_H2_Interpolated}')
print(f'SS_K2:{SS_K2_Interpolated}')
print(f'SS_H3:{SS_H3_Interpolated}')
print(f'SS_K3:{SS_K3_Interpolated}')
print(f'SS_H4:{SS_H4_Interpolated}')
print(f'SS_K4:{SS_K4_Interpolated}')
SSH1 = pd.DataFrame(SS_H1_Interpolated)
SSK1 = pd.DataFrame(SS_K1_Interpolated)
SSH2 = pd.DataFrame(SS_H2_Interpolated)
SSK2 = pd.DataFrame(SS_K2_Interpolated)
SSH3 = pd.DataFrame(SS_H3_Interpolated)
SSK3 = pd.DataFrame(SS_K3_Interpolated)
SSH4 = pd.DataFrame(SS_H4_Interpolated)
SSK4 = pd.DataFrame(SS_K4_Interpolated)
SSH1.to_csv ('3D_8_SS_H1_Interpolated_0.01s.txt', index = False, header=True)
SSK1.to_csv ('3D_8_SS_K1_Interpolated_0.01s.txt', index = False, header=True)
SSH2.to_csv ('3D_8_SS_H2_Interpolated_0.01s.txt', index = False, header=True)
SSK2.to_csv ('3D_8_SS_K2_Interpolated_0.01s.txt', index = False, header=True)
SSH3.to_csv ('3D_8_SS_H3_Interpolated_0.01s.txt', index = False, header=True)
SSK3.to_csv ('3D_8_SS_K3_Interpolated_0.01s.txt', index = False, header=True)
SSH4.to_csv ('3D_8_SS_H4_Interpolated_0.01s.txt', index = False, header=True)
SSK4.to_csv ('3D_8_SS_K4_Interpolated_0.01s.txt', index = False, header=True)
