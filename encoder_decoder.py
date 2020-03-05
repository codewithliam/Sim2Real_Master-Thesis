from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
from time import time
import datetime
import os
from model_util import * 

from tensorflow.python import keras
from keras.utils import Sequence
from keras.models import Sequential
from keras.layers import LSTM, Dense, TimeDistributed
from keras.layers import Input
from keras.layers import Bidirectional
from keras.models import Model
from keras.layers import Dropout
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split
import keras.backend as K 
from keras.optimizers import Adam
from keras.callbacks import TensorBoard


os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' #This never works because tensorflow never shuts up

X, Y = batch_creater()

X_train, X_val, Y_train, Y_val = train_test_split(X, Y, test_size=0.2, random_state=42)

X_train, X_test, Y_train, Y_test = train_test_split(X_train, Y_train, test_size=0.1, random_state=42)


class Mygenerator(Sequence):
    def __init__(self, x_set, y_set, batch_size):
        self.x, self.y = x_set, y_set
        self.batch_size = batch_size

    def __len__(self):
        return int(np.ceil(len(self.x) / float(self.batch_size)))

    def __getitem__(self, idx):
        batch_x = self.x[idx]
        batch_y = self.y[idx]

        # read your data here using the batch lists, batch_x and batch_y
        #x = [x for x in batch_x] 
        #y = [y for y in batch_y]
        x=np.reshape(batch_x, (1, batch_x.shape[0], batch_x.shape[1]))
        y=np.reshape(batch_y, (1, batch_y.shape[0], batch_y.shape[1]))

        return x, y


X1=Input(batch_shape=(1, None, 21))

output=Bidirectional(LSTM(100, return_sequences=True, activation='tanh', recurrent_dropout=0.1, stateful=True), input_shape=(None, 21),  merge_mode='sum')(X1)
output= Dropout(0.1)(output)
output=Bidirectional(LSTM(100, return_sequences=True, activation='tanh', recurrent_dropout=0.1, stateful=True), merge_mode='sum')(output)
# TODO: Try repeatvector s
#output=Dropout(0.1)(output)
output= TimeDistributed(Dense(21))(output)

model=Model(X1, output)

model.compile(loss=custom_loss(X1), optimizer=Adam(lr=0.001, clipnorm=1.0, clipvalue=0.5))
#print(model.summary())

log_dir="logs/fit/"

tensorboard = TensorBoard(log_dir=log_dir)


history = model.fit_generator(Mygenerator(X_train, Y_train, 1), 
validation_data=Mygenerator(X_val, Y_val, 1), epochs=1, callbacks=[tensorboard])


for i in range (len(X_test)):
    X=X_test[i]
    Y=Y_test[i]

    X=np.reshape(X, (1, X.shape[0], X.shape[1]))
    Y=np.reshape(Y, (1, Y.shape[0], Y.shape[1]))


    predictions= model.predict_on_batch(X)+X

    for k in range(7): 
        predict_effort=predictions[0, :, 14+k] #14+k returns the effort of the joint in action
        sim_effort = X[0, :,14+k]
        true_effort=Y[0, :,14+k]

        plt.figure()
        effort=plt.plot(predict_effort, 'r', label='Prediction') # plotting t, a separately 
        sim=plt.plot(sim_effort, 'b', label='Simulation') # plotting t, b separately 
        true=plt.plot(true_effort, 'g', label= 'Ground_truth') # plotting t, c separately 
        plt.legend()
        plt.show()
