import numpy as np
import pandas as pd
import math
import time
import os
import pickle
import matplotlib.pyplot as plt

real_path = '/home/liam/Desktop/Test_data/testpath/' 
sim_path = '/home/liam/Desktop/Test_data/testpath/simulation/'


def preprocess(offset_factor=30, normalize_dataset=True, pickle_save=True):

    X_train = [] # Datasets where all the trajetory data are put
    Y_train = []

    for path_index in range(100):

        try: 
            X,Y = batch_loader(path_index)
        except Exception as e: 
            break
        
        start_point_sim=check_start(X, False)
        start_point_real=check_start(Y, True)

        if start_point_real == None or start_point_sim == None:
            continue 
    
        X=X[(start_point_sim-offset_factor):-1]
        Y=Y[(start_point_real-offset_factor):-1]


        X, Y = make_same_length(X, Y)

        #if valid_trajectory_check(X, Y, joint_index) == False: 
        #    continue
    
        X,Y = remove_name_columns(X,Y)

        if normalize_dataset==True: 
            X, Y = normalize_data(X, Y)
        
        X_train.append(X)
        Y_train.append(Y)

    if pickle_save == True: 
        pickle.dump(X_train, open("X_linear.p","wb"))
        pickle.dump(Y_train, open("Y_linear.p","wb"))

        print(len(X_train))

    return X_train, Y_train
                
def batch_loader(i):

    name = 'plan3_joint_iter' + str(i) +'.csv'

    path_real = real_path + name
    path_sim = sim_path + name
    Y = pd.read_csv(path_real) # 'traj_real'
    X = pd.read_csv(path_sim) # 'traj_sim'

    return X, Y
                
def check_start(traj,real, std_factor=1.6, interval_length=10): 
    if real == True: 
       position_data=traj.iloc[:, np.r_[13:20]].copy()
    else: 
       position_data=traj.iloc[:, np.r_[15:22]].copy()

    position_data=position_data.to_numpy()

    for i in range(interval_length, len(position_data)-interval_length): 

        std=np.mean(np.std(position_data[0:i, :], axis=0))
        mean=np.mean(position_data[0:i, :])

        if not mean-std*std_factor < np.mean(position_data[i, :]) < mean+std*std_factor: 
            for x in range(interval_length): 
                if check_interval(position_data[i+x, :], mean, std, std_factor) == False: 
                    break
                if x==interval_length-1: 
                    return i

def check_interval(data, mean, std, factor):

    #1std=32% 2std=5%, 3std=0.3% outlier, factor 2std is good I guess?
    
    if not mean-std*factor < np.mean(data) < mean+std*factor: 
        return True
    else: return False

def make_same_length(X, Y): 

    if len(Y) >= len(X): 
        Y=Y[0:len(X)]
    else: 
        X=X[0:len(Y)]

    return X,Y

def valid_trajectory_check(X, Y, joint_index):

    Y_position=Y.loc[:,'field.position' + str(joint_index)]
    X_position=X.loc[:,'field.position' + str(joint_index+2)]

    rms = (np.square(X_position - Y_position)).mean()
    print(rms)

    diff_sim = np.max(X_position) - np.min(X_position)
    diff_real = np.max(Y_position) - np.min(Y_position)

    if rms < 0.3 and diff_real > 0.05:
        return True

    return False

def remove_name_columns(X, Y):

    X=X.iloc[1:-1, np.r_[15:22, 24:31, 33:40]].copy()
    Y=Y.iloc[1:-1, np.r_[13:20, 22:29, 31:38]].copy()
    X=X.to_numpy()
    Y=Y.to_numpy() #Numpy is faster than pandas dataframe so conversion happens here

    return X, Y

def normalize_data(X, Y): 

    X[:, 0:14] += math.pi # Position = X[:, 0:7], velocity = X[:, 7:14], max=3.14, min = -3.14
    X[:, 0:14] /= (math.pi*2)
    X[:, 14:18] += 90 #Effort = X[:, 14:21], max_joint 0-4 = 90 Nm, min= -90 Nm
    X[:, 14:18] /= 180 
    X[:, 18:21] += 15 #Effort = X[:, 14:21], max_joint 5-7 = 15 Nm, min= -15 Nm
    X[:, 18:21] /= 30 

    Y[:, 0:14] += math.pi # Position = Y[:, 0:7], velocity = Y[:, 7:14], max=3.14, min = -3.14
    Y[:, 0:14] /= (math.pi*2)
    Y[:, 14:18] += 90 #Effort = Y[:, 14:21], max_joint 0-4 = 90 Nm, min= -90 Nm
    Y[:, 14:18] /= 180 
    Y[:, 18:21] += 15 #Effort = Y[:, 14:21], max_joint 5-7 = 15 Nm, min= -15 Nm
    Y[:, 18:21] /= 30 

    return X, Y


def test_pickledata(): 
  infile_X = open('X_linear.p','rb')
  infile_Y = open('Y_linear.p', 'rb')
  X = pickle.load(infile_X)
  Y = pickle.load(infile_Y)
  infile_X.close()
  infile_Y.close()

  for i in range(0, len(X)): 
    X_traj = X[i]
    Y_traj = Y[i]
    for k in range(7):
        position_sim = X_traj[:, k]
        position_real = Y_traj[:, k]

        plt.plot(position_real)
        plt.plot(position_sim)
        plt.show()


test_pickledata()
#preprocess(pickle_save=True)