import pandas as pd
import cloudpickle as cp 
import numpy as np

#Import support files:
import config_parser
import constants

def get_initial_seed(m,run_config,resource):
    movement = pd.read_csv('../resources/'+resource)
    end = movement.iloc[0,:]
    
    for n in range(1,run_config.num_nodes+1):
        for c in range (1, constants.cN+1): 
            m.q[n,c,'x'].value = 0.00   
            m.q[n,c,'y'].value = 0.00         
            m.q[n,c,'z'].value  = end[2]
            m.q[n,c,'theta_bx'].value = end[3]#np.random.uniform(np.pi/4,3*np.pi/4)  
            m.q[n,c,'theta_by'].value = end[4]#np.random.uniform(np.pi/4,3*np.pi/4)  
            m.q[n,c,'theta_bz'].value = end[5]#np.random.uniform(np.pi/4,3*np.pi/4)  
            m.q[n,c,'theta_h1'].value = -end[6]#np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_k1'].value = -end[7]#np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_h2'].value = -end[8]#np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_k2'].value = -end[9]#np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_h3'].value = end[10]#np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_k3'].value = end[11]#np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_h4'].value = end[12]#np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_k4'].value = end[13]#np.random.uniform(-np.pi/2,np.pi/2)
            
            #Setting everything else to 0.01
            for dof in constants.DOFs:
                m.dq[n,c , dof].value = 0.01
                m.ddq[n,c , dof].value = 0.01
                
            m.GRF1[n,c,'Z','ps'].value = 0.01
            m.GRF1[n,c,'X','ps'].value = 0.01 
            m.GRF1[n,c,'X','ng'].value = 0.01 
            
            m.GRF2[n,c,'Z','ps'].value = 0.01
            m.GRF2[n,c,'X','ps'].value = 0.01 
            m.GRF2[n,c,'X','ng'].value = 0.01 
            
            m.GRF3[n,c,'Z','ps'].value = 0.01
            m.GRF3[n,c,'X','ps'].value = 0.01 
            m.GRF3[n,c,'X','ng'].value = 0.01 
            
            m.GRF4[n,c,'Z','ps'].value = 0.01
            m.GRF4[n,c,'X','ps'].value = 0.01 
            m.GRF4[n,c,'X','ng'].value = 0.01
    return m

def get_refined_seed(m,run_config):
    with open(run_config.get_model_path(), mode='rb') as file:
        m_instance = cp.load(file)

    m.iterations_complete = m_instance.iterations_complete
    m.eps.value = m_instance.eps.value

    for n in range(1,run_config.num_nodes+1):
        for c in range (1, constants.cN+1): 
            m.q[n,c,'x'].value = m_instance.q[n,c,'x'].value  
            m.q[n,c,'y'].value = m_instance.q[n,c,'y'].value         
            m.q[n,c,'z'].value  = m_instance.q[n,c,'z'].value
            m.q[n,c,'theta_bx'].value = m_instance.q[n,c,'theta_bx'].value
            m.q[n,c,'theta_by'].value = m_instance.q[n,c,'theta_by'].value
            m.q[n,c,'theta_bz'].value = m_instance.q[n,c,'theta_bz'].value
            m.q[n,c,'theta_h1'].value = m_instance.q[n,c,'theta_h1'].value
            m.q[n,c,'theta_k1'].value = m_instance.q[n,c,'theta_k1'].value
            m.q[n,c,'theta_h2'].value = m_instance.q[n,c,'theta_h2'].value
            m.q[n,c,'theta_k2'].value = m_instance.q[n,c,'theta_k2'].value
            m.q[n,c,'theta_h3'].value = m_instance.q[n,c,'theta_h3'].value
            m.q[n,c,'theta_k3'].value = m_instance.q[n,c,'theta_k3'].value
            m.q[n,c,'theta_h4'].value = m_instance.q[n,c,'theta_h4'].value
            m.q[n,c,'theta_k4'].value = m_instance.q[n,c,'theta_k4'].value
            
            m.dq[n,c,'x'].value = m_instance.dq[n,c,'x'].value  
            m.dq[n,c,'y'].value = m_instance.dq[n,c,'y'].value         
            m.dq[n,c,'z'].value  = m_instance.dq[n,c,'z'].value
            m.dq[n,c,'theta_bx'].value = m_instance.dq[n,c,'theta_bx'].value
            m.dq[n,c,'theta_by'].value = m_instance.dq[n,c,'theta_by'].value
            m.dq[n,c,'theta_bz'].value = m_instance.dq[n,c,'theta_bz'].value
            m.dq[n,c,'theta_h1'].value = m_instance.dq[n,c,'theta_h1'].value
            m.dq[n,c,'theta_k1'].value = m_instance.dq[n,c,'theta_k1'].value
            m.dq[n,c,'theta_h2'].value = m_instance.dq[n,c,'theta_h2'].value
            m.dq[n,c,'theta_k2'].value = m_instance.dq[n,c,'theta_k2'].value
            m.dq[n,c,'theta_h3'].value = m_instance.dq[n,c,'theta_h3'].value
            m.dq[n,c,'theta_k3'].value = m_instance.dq[n,c,'theta_k3'].value
            m.dq[n,c,'theta_h4'].value = m_instance.dq[n,c,'theta_h4'].value
            m.dq[n,c,'theta_k4'].value = m_instance.dq[n,c,'theta_k4'].value

            m.ddq[n,c,'x'].value = m_instance.ddq[n,c,'x'].value  
            m.ddq[n,c,'y'].value = m_instance.ddq[n,c,'y'].value         
            m.ddq[n,c,'z'].value  = m_instance.ddq[n,c,'z'].value
            m.ddq[n,c,'theta_bx'].value = m_instance.ddq[n,c,'theta_bx'].value
            m.ddq[n,c,'theta_by'].value = m_instance.ddq[n,c,'theta_by'].value
            m.ddq[n,c,'theta_bz'].value = m_instance.ddq[n,c,'theta_bz'].value
            m.ddq[n,c,'theta_h1'].value = m_instance.ddq[n,c,'theta_h1'].value
            m.ddq[n,c,'theta_k1'].value = m_instance.ddq[n,c,'theta_k1'].value
            m.ddq[n,c,'theta_h2'].value = m_instance.ddq[n,c,'theta_h2'].value
            m.ddq[n,c,'theta_k2'].value = m_instance.ddq[n,c,'theta_k2'].value
            m.ddq[n,c,'theta_h3'].value = m_instance.ddq[n,c,'theta_h3'].value
            m.ddq[n,c,'theta_k3'].value = m_instance.ddq[n,c,'theta_k3'].value
            m.ddq[n,c,'theta_h4'].value = m_instance.ddq[n,c,'theta_h4'].value
            m.ddq[n,c,'theta_k4'].value = m_instance.ddq[n,c,'theta_k4'].value
                
            m.GRF1[n,c,'Z','ps'].value = m_instance.GRF1[n,c,'Z','ps'].value 
            m.GRF1[n,c,'X','ps'].value = m_instance.GRF1[n,c,'X','ps'].value
            m.GRF1[n,c,'X','ng'].value = m_instance.GRF1[n,c,'X','ng'].value

            m.GRF2[n,c,'Z','ps'].value = m_instance.GRF2[n,c,'Z','ps'].value 
            m.GRF2[n,c,'X','ps'].value = m_instance.GRF2[n,c,'X','ps'].value
            m.GRF2[n,c,'X','ng'].value = m_instance.GRF2[n,c,'X','ng'].value

            m.GRF3[n,c,'Z','ps'].value = m_instance.GRF3[n,c,'Z','ps'].value 
            m.GRF3[n,c,'X','ps'].value = m_instance.GRF3[n,c,'X','ps'].value
            m.GRF3[n,c,'X','ng'].value = m_instance.GRF3[n,c,'X','ng'].value

            m.GRF4[n,c,'Z','ps'].value = m_instance.GRF4[n,c,'Z','ps'].value 
            m.GRF4[n,c,'X','ps'].value = m_instance.GRF4[n,c,'X','ps'].value
            m.GRF4[n,c,'X','ng'].value = m_instance.GRF4[n,c,'X','ng'].value
    return m

def get_random_seed(m,run_config):
    for n in range(1,constants.N+1):
        for c in range (1, constants.cN+1): 
            #Randomizing Q Matrix
            m.q[n,c,'x'].value = 0.01   
            m.q[n,c,'y'].value = 0.01         
            m.q[n,c,'z'].value  = 0.01
            m.q[n,c,'theta_bx'].value = np.random.uniform(np.pi/4,3*np.pi/4)  
            m.q[n,c,'theta_by'].value = np.random.uniform(np.pi/4,3*np.pi/4)  
            m.q[n,c,'theta_bz'].value = np.random.uniform(np.pi/4,3*np.pi/4)  
            m.q[n,c,'theta_h1'].value = np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_k1'].value = np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_h2'].value = np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_k2'].value = np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_h3'].value = np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_k3'].value = np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_h4'].value = np.random.uniform(-np.pi/2,np.pi/2)
            m.q[n,c,'theta_k4'].value = np.random.uniform(-np.pi/2,np.pi/2)
            
            #Setting everything else to 0.01
            for dof in constants.DOFs:
                m.dq[n,c , dof].value = 0.01
                m.ddq[n,c , dof].value = 0.01
                
            m.GRF1[n,c,'Z','ps'].value = 0.01
            m.GRF1[n,c,'X','ps'].value = 0.01 
            m.GRF1[n,c,'X','ng'].value = 0.01 
            
            m.GRF2[n,c,'Z','ps'].value = 0.01
            m.GRF2[n,c,'X','ps'].value = 0.01 
            m.GRF2[n,c,'X','ng'].value = 0.01 
            
            m.GRF3[n,c,'Z','ps'].value = 0.01
            m.GRF3[n,c,'X','ps'].value = 0.01 
            m.GRF3[n,c,'X','ng'].value = 0.01 
            
            m.GRF4[n,c,'Z','ps'].value = 0.01
            m.GRF4[n,c,'X','ps'].value = 0.01 
            m.GRF4[n,c,'X','ng'].value = 0.01
    return m
            