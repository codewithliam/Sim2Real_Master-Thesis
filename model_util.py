import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
import time
import os
import pickle
from tensorflow.python import keras
from keras.utils import Sequence
from keras.models import Sequential
import tensorflow as tf
from keras.layers import LSTM, Dense, TimeDistributed
from keras.layers import Input
from keras.layers import Bidirectional
from keras.models import Model
from keras.layers import Dropout
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import mean_squared_error
import keras.backend as K 
from keras.optimizers import Adam

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' #This never works because tensorflow never shuts up

main_path= '/home/liam/Desktop/real_logs5 /'
path_vector= [
main_path + 'acc_scaling=0.4_30_reps/preprocess',
main_path + 'acc_scaling=0.6_30_reps/preprocess',
main_path + 'acc_scaling=0.8_30_reps/preprocess', #If new sets are created, just add the path to the path vector and it's handled
main_path + 'acc_scaling=1.0_30_reps/preprocess' ]



random_seed=np.random.seed(7)


def batch_loader(i, j, k):

    #This part is unnecressary to include in a feedforward step, but I timed it and it's quite fast so it doesn't slow anything down.

    path_real = path_vector[i] + '/traj_' + str(j) + '/joint_' + str(k)# +'.csv'
    path_sim = path_vector[i] + '/traj_' + str(j) + '/sim_' + str(k)# + '.csv'

    X = pd.read_csv(path_sim) # 'traj_sim'
    Y = pd.read_csv(path_real) # 'traj_real'

    X=X.iloc[1:-1, np.r_[16:23, 25:32, 34:41]].copy()
    
    X=X.iloc[1:-1, np.r_[14:21, 23:30, 32:39]].copy()

    Y=Y.iloc[1:-1, np.r_[14:21, 23:30, 32:39]].copy()

    X=X.to_numpy()
    Y=Y.to_numpy()
    
    X[:, 0:14] += math.pi
    X[:, 0:14] /= (math.pi*2)
    X[:, 14:18] += 90
    X[:, 14:18] /= 180
    X[:, 18:21] += 15 
    X[:, 18:21] /= 30 
    # NORMALIZE BETWEEN -1 AND 1 ?? 


    Y[:, 0:14] += math.pi
    Y[:, 0:14] /= (math.pi*2)
    Y[:, 14:18] += 90
    Y[:, 14:18] /= 180
    Y[:, 18:21] += 15 
    Y[:, 18:21] /= 30 
        
    return X, Y



def batch_creater(): 
    X_train = []
    Y_train = []
    
    for j in range(30):
        for i in range(len(path_vector)): #This is the paths with different acceleration_scaling
            for k in range(7):
                try:  
                    X,Y = batch_loader(i, j, k)
                    X_train.append(X)
                    Y_train.append(Y)
                except Exception as e: #In case a batch doesnt exist
                    continue

    X_train=np.asarray(X_train)
    Y_train=np.asarray(Y_train)

    return X_train, Y_train

def custom_loss(X):

    def loss(yTrue,yPred): 
        vec=K.square(K.mean(yTrue-(yPred+X), axis=1))
        return K.square(K.mean(yTrue-(yPred+X), axis=1))

    return loss

'''
X, Y = batch_creater()

pickle.dump(X, open("X.p","wb"))
pickle.dump(Y, open("Y.p","wb"))
'''

def benchmark(): 
    
    path_x= 'X.p'
    path_y= 'Y.p'

    infile_x = open(path_x,'rb')
    X_train = pickle.load(infile_x)
    infile_x.close()

    infile_y = open(path_y,'rb')
    Y_train = pickle.load(infile_y)
    infile_y.close()
    sum=0
    for i in range(len(X_train)):
        X = X_train[i]
        Y = Y_train[i]
        np.square(X-Y).shape
        sum+=np.mean(np.sqrt((np.square(X-Y))))
        #print(sum)

    #print(sum/len(X))


#benchmark()
        
X,Y=batch_creater()
         




