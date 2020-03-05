import os
import sys

import pandas as pd 
import numpy as np 
import seaborn as sns
import matplotlib.pyplot as plt
import pickle
import math
import time


main_path= '/home/liam/Desktop/real_logs5 /'
path_vector= [
main_path + 'acc_scaling=0.4_30_reps',
main_path + 'acc_scaling=0.6_30_reps',
main_path + 'acc_scaling=0.8_30_reps',
main_path + 'acc_scaling=1.0_30_reps' ]

def paths(i): 
    main_path=path_vector[i]
    folderpath=  main_path + '/traj_'
    Imagepath =  main_path + '/pictures/'
    Preprocesspath = main_path + '/preprocess'

    return folderpath, Imagepath, Preprocesspath


def csv_loader(folderpath, traj_index, real, joint_index): 
    
    if real==True: 
        Path = folderpath + str(traj_index) + '/joint_' +  str(joint_index) + '.csv'
    else: 
        Path = folderpath + str(traj_index) + '/sim_' +  str(joint_index) + '.csv'
    
    return Path


def create_time_column(traj): 
    d_time= traj.iloc[:,0]-traj.iloc[0,0]
    d_time=d_time/1e9
    return d_time

#sim_or_real_string=sys.argv[1]
sim_or_real_string='both'
string_vec=['field.position', 'field.velocity',  'field.effort']
name_vec = ['position', 'velocity', 'effort']


def create_pics():
    for i in range(30): #Is 10 now because there is 10 traj folders
        for j in range(7): 
            for x in range(3): 
                field_string=string_vec[x]
                field_name=name_vec[x]
                '''
                if sim_or_real_string == 'real': 
                    try: 
                        traj=pd.read_csv(csv_loader(i, True, j))
                    except Exception as e: 
                        break 
                    traj['seconds']=create_time_column(traj)
                    traj['index']=traj.index
                    sns.set()
                    real_string= field_string + str(j)
                    start_point=check_start2(i, True, j)
                    if start_point == None:
                        break
                    start_time=traj['seconds'][start_point]
                    plt.axvline(start_time, 0, 1, color='black')
                    ax=sns.lineplot(y=real_string, x='seconds', data=traj)
                    savepath= Imagepath + 'traj_' + str(i)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)
                    plt.savefig(savepath + '/joint' + str(j) + '_' + field_name)
                    plt.clf()

                if sim_or_real_string == 'sim': 
                    try: 
                        traj_sim=pd.read_csv(csv_loader(i, False, j))
                    except Exception as e: 
                        break 

                    traj_sim['seconds']=create_time_column(traj_sim)
                    traj_sim['index']=traj_sim.index
                    sim_string= field_string + str(j+2)
                    start_point=check_start2(i, False, j)
                    if start_point == None:
                        break
                    start_time=traj_sim['seconds'][start_point]
                    plt.axvline(start_time, 0, 1, color='black')
                    ax2=sns.lineplot(y=sim_string, x='seconds', data=traj_sim)
                    savepath= Imagepath + 'traj_' + str(i)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)
                    plt.savefig(savepath + '/sim' + str(j) + '_' + field_name)
                    plt.clf()
                    '''
                if sim_or_real_string == 'both': 

                    try: 
                        traj=pd.read_csv(csv_loader(i, True, j))
                    except Exception as e: 
                        break 
                    traj['seconds']=create_time_column(traj)
                    traj['index']=traj.index
                    sns.set()
                    real_string= field_string + str(j)
                    start_point=check_start2(i, True, j)
                    if start_point == None:
                        break
                    start_time_real=traj['seconds'][start_point]
                    #plt.axvline(start_time_real, 0, 1, color='black')
                    traj=traj[start_point:-1]
                    traj['seconds']=traj['seconds']-traj['seconds'][start_point]
                    traj['index']=traj['index'] - start_point


                    ax=sns.lineplot(y=real_string, x='index', data=traj)
                    savepath= Imagepath + 'traj_' + str(i)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)

                    try: 
                        traj_sim=pd.read_csv(csv_loader(i, False, j))
                    except Exception as e: 
                        break 
                    traj_sim['seconds']=create_time_column(traj_sim)
                    traj_sim['index']=traj_sim.index
                    sim_string= field_string + str(j+2)
                    start_point=check_start2(i, False, j)
                    if start_point == None:
                        break
                    start_time_sim=traj_sim['seconds'][start_point]
                    traj_sim=traj_sim[start_point:-1]
                    traj_sim['seconds']=traj_sim['seconds']-traj_sim['seconds'][start_point]
                    traj_sim['index']=traj_sim['index'] - start_point

                
                    #plt.axvline(start_time_sim, 0, 1, color='black')
                    ax2=sns.lineplot(y=sim_string, x='index', data=traj_sim)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)
                    plt.savefig(savepath + '/fig' + str(j) + '_' + field_name)
                    plt.clf()


def create_pics2():
    for i in range(30): #Is 10 now because there is 10 traj folders
        for j in range(7): 
            for x in range(3): 
                field_string=string_vec[x]
                field_name=name_vec[x]
                '''
                if sim_or_real_string == 'real': 
                    try: 
                        traj=pd.read_csv(csv_loader(i, True, j))
                    except Exception as e: 
                        break 
                    traj['seconds']=create_time_column(traj)
                    traj['index']=traj.index
                    sns.set()
                    real_string= field_string + str(j)
                    start_point=check_start2(i, True, j)
                    if start_point == None:
                        break
                    start_time=traj['seconds'][start_point]
                    plt.axvline(start_time, 0, 1, color='black')
                    ax=sns.lineplot(y=real_string, x='seconds', data=traj)
                    savepath= Imagepath + 'traj_' + str(i)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)
                    plt.savefig(savepath + '/joint' + str(j) + '_' + field_name)
                    plt.clf()

                if sim_or_real_string == 'sim': 
                    try: 
                        traj_sim=pd.read_csv(csv_loader(i, False, j))
                    except Exception as e: 
                        break 

                    traj_sim['seconds']=create_time_column(traj_sim)
                    traj_sim['index']=traj_sim.index
                    sim_string= field_string + str(j+2)
                    start_point=check_start2(i, False, j)
                    if start_point == None:
                        break
                    start_time=traj_sim['seconds'][start_point]
                    plt.axvline(start_time, 0, 1, color='black')
                    ax2=sns.lineplot(y=sim_string, x='seconds', data=traj_sim)
                    savepath= Imagepath + 'traj_' + str(i)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)
                    plt.savefig(savepath + '/sim' + str(j) + '_' + field_name)
                    plt.clf()
                    '''
                if sim_or_real_string == 'both': 

                    try: 
                        traj=pd.read_csv(csv_loader(i, True, j))
                    except Exception as e: 
                        break 
                    traj['seconds']=create_time_column(traj)
                    traj['index']=traj.index
                    sns.set()
                    real_string= field_string + str(j)
                    start_point=check_start2(i, True, j)
                    if start_point == None:
                        break
                    start_time_real=traj['seconds'][start_point]
                    end_point_real=check_end(i, True, j)
                    traj=traj[start_point:-1]
                    traj['seconds']=traj['seconds']-traj['seconds'][start_point]
                    plt.axvline(end_point_real, 0, 1, color='black')
                    ax=sns.lineplot(y=real_string, x='index', data=traj)
                    savepath= Imagepath + 'traj_' + str(i)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)

                    try: 
                        traj_sim=pd.read_csv(csv_loader(i, False, j))
                    except Exception as e: 
                        break 
                    traj_sim['seconds']=create_time_column(traj_sim)
                    traj_sim['index']=traj_sim.index
                    sim_string= field_string + str(j+2)
                    start_point=check_start2(i, False, j)
                    if start_point == None:
                        break
                    start_time_sim=traj_sim['seconds'][start_point]
                    traj_sim=traj_sim[start_point:-1]
                    traj_sim['seconds']=traj_sim['seconds']-traj_sim['seconds'][start_point]

                    end_point_sim=check_end(i,False, j)
                    plt.axvline(end_point_sim, 0, 1, color='red')
                    ax2=sns.lineplot(y=sim_string, x='index', data=traj_sim)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)
                    plt.savefig(savepath + '/fig' + str(j) + '_' + field_name)
                    plt.clf()



def create_pics_all():
    for i in range(30): #Is 10 now because there is 10 traj folders
        for j in range(7): 
            for x in range(3): 
                field_string=string_vec[x]
                field_name=name_vec[x]

                if sim_or_real_string == 'both': 

                    for k in range(7): 
                        try: 
                            traj=pd.read_csv(csv_loader(i, True, j))
                        except Exception as e: 
                            print(e)
                            break 
                        traj['seconds']=create_time_column(traj)
                        traj['index']=traj.index
                        sns.set()
                        real_string= field_string + str(k)
                        start_point=check_start2(i, True, j)
                        if start_point == None:
                            print('haj')
                            break
                        traj=traj[start_point:-1]
                        traj['seconds']=traj['seconds']-traj['seconds'][start_point]

                        ax=sns.lineplot(y=real_string, x='seconds', data=traj)
                        savepath= Imagepath + 'traj_' + str(i)
                        if os.path.exists(savepath) == False: 
                            os.makedirs(savepath)
                        try: 
                            traj_sim=pd.read_csv(csv_loader(i, False, j))
                        except Exception as e: 
                            break 
                        traj_sim['seconds']=create_time_column(traj_sim)
                        traj_sim['index']=traj_sim.index
                        sim_string= field_string + str(k+2)
                        start_point=check_start2(i, False, j)
                        if start_point == None:
                            break
                        traj_sim=traj_sim[start_point:-1]
                        traj_sim['seconds']=traj_sim['seconds']-traj_sim['seconds'][start_point]

                        
                        secs, positions, velocities, accelerations = plan_process(i,j)
                        if x == 0: 
                            plt.plot(secs, positions[:,k], 'bo')
                        if x == 1: 
                            plt.plot(secs, velocities[:,k], 'bo')
                        if x == 2: 
                            plt.plot(secs, accelerations[:,k], 'bo')
                    
                        #plt.axvline(start_time_sim, 0, 1, color='black')
                        ax2=sns.lineplot(y=sim_string, x='seconds', data=traj_sim)
                        sns.set()
                        if os.path.exists(savepath) == False : 
                            os.makedirs(savepath)
                        plt.savefig(savepath + '/move_joint' + str(j) + '_joint_' + str(k) + field_name)
                        plt.clf()


                

            '''
            traj_sim=pd.read_csv(csv_loader(i, False, j))
            traj_sim['seconds']=create_time_column(traj_sim)
            traj_sim['index']=traj_sim.index
            sim_string= 'field.position' + str(j+2)
            ax2=sns.lineplot(y=sim_string, x='seconds', data=traj_sim)
            savepath= Imagepath + 'traj_' + str(i)
            if os.path.exists(savepath) == False : 
                os.makedirs(savepath)
            plt.savefig(savepath + '/sim' + str(j))
            plt.clf()
            '''

def align():
    for i in range(30): #Is 10 now because there is 10 traj folders
        for j in range(7): 
            for x in range(3): 
                field_string=string_vec[x]
                field_name=name_vec[x]

                if sim_or_real_string == 'both': 
                    try: 
                        traj=csv_loader(i, True, j)
                    except Exception as e:
                        break 
                    traj['seconds']=create_time_column(traj)
                    traj['index']=traj.index
                    sns.set()
                    real_string= field_string + str(j)
                    start_point=check_start2(i, True, j)
                    if start_point == None:
                        break
                    start_time_real=traj['seconds'][start_point]
                    #plt.axvline(start_time_real, 0, 1, color='black')
                    traj=traj[start_point:-1]
                    traj['seconds']=traj['seconds']-traj['seconds'][start_point]

                    savepath= Preprocesspath + '/traj_' + str(i)
                    if os.path.exists(savepath) == False : 
                        os.makedirs(savepath)
                    traj.to_csv(savepath + '/joint_' + str(j))
                    
                    try: 
                        traj_sim=pd.read_csv(csv_loader(i, False, j))
                    except Exception as e: 
                        break 
                    traj_sim['seconds']=create_time_column(traj_sim)
                    traj_sim['index']=traj_sim.index
                    sim_string= field_string + str(j+2)
                    start_point=check_start2(i, False, j)
                    if start_point == None:
                        break
                    start_time_sim=traj_sim['seconds'][start_point]
                    traj_sim=traj_sim[start_point:-1]
                    traj_sim['seconds']=traj_sim['seconds']-traj_sim['seconds'][start_point]

                    traj['seconds']=create_time_column(traj)
                    traj['index']=traj.index
                    sns.set()
                    real_string= field_string + str(j)
                    start_point=check_start2(i, True, j)
                    savepath= Preprocesspath + '/traj_' + str(i)
                    if os.path.exists(savepath) == False :
                        os.makedirs(savepath)
                    traj.to_csv(savepath + '/joint_' + str(j))
                    

                    try: 
                        traj_sim=pd.read_csv(csv_loader(i, False, j))
                    except Exception as e: 
                        break 
                    traj_sim['seconds']=create_time_column(traj_sim)

                    traj_sim['index']=traj_sim.index
                    sim_string= field_string + str(j+2)
                    start_point=check_start2(i, False, j)
                    if start_point == None:
                        break
                    #start_time_sim=traj_sim['seconds'][start_point]
                    #plt.axvline(start_time_real, 0, 1, color='black')
                    traj_sim=traj_sim[start_point:-1]
                    traj_sim['seconds']=traj_sim['seconds']-traj_sim['seconds'][start_point]
                    if os.path.exists(savepath) == False:
                        os.makedirs(savepath)
                    traj.to_csv(savepath + '/sim_' + str(j))



def preprocess():
    offset_factor=30
    for path_index in range(len(path_vector)):
        folderpath, Imagepath, Preprocesspath = paths(path_index)
        for i in range(30): #Is 10 now because there is 10 traj folders
            for j in range(7):                     
                try: 
                    traj=pd.read_csv(csv_loader(folderpath, i, True, j))
                except Exception as e:
                    print(e) 
                    break 
            
                traj['seconds']=create_time_column(traj)
                traj['index']=traj.index
                start_point=check_start2(folderpath, i, True, j)
                if start_point == None:
                    break
                try: 
                    traj=traj[start_point-offset_factor:-1]
                    traj['seconds']=traj['seconds']-traj['seconds'][start_point-offset_factor]
                except Exception as e: 
                    break

                try: 
                    traj_sim=pd.read_csv(csv_loader(folderpath, i, False, j))
                except Exception as e:
                    break 

                traj_sim['seconds']=create_time_column(traj_sim)
                traj_sim['index']=traj_sim.index
                start_point=check_start2(folderpath, i, False, j)
                if start_point == None:
                    break
                try:
                    traj_sim=traj_sim[start_point-offset_factor:-1]
                    traj_sim['seconds']=traj_sim['seconds']-traj_sim['seconds'][start_point-offset_factor]
                except Exception as e: 
                    break

                    
                data_pos=traj.loc[:,'field.position' + str(j)]
                data_pos=data_pos[offset_factor:-1]
                data_sim_pos=traj_sim.loc[:,'field.position' + str(j+2)]
                data_sim_pos=data_sim_pos[offset_factor:-1]


                diff_real = np.max(data_pos) - np.min(data_pos)
                diff_sim = np.max(data_sim_pos) - np.min(data_sim_pos)

                if abs(diff_real-diff_sim) < 0.2: 
                    
                    if len(traj) >= len(traj_sim): 
                        traj=traj[0:len(traj_sim)]
                    else: 
                        traj_sim=traj_sim[0:len(traj)]
                    
                    savepath, imagepath, Preprocesspath = paths(path_index)
                    savepath= Preprocesspath + '/traj_' + str(i)
                    if os.path.exists(savepath) == False :
                        os.makedirs(savepath)
                    traj.to_csv(savepath + '/joint_' + str(j))
                    traj_sim.to_csv(savepath + '/sim_' + str(j))
                    
                    '''
                    start_point_sim=check_start2(i, False, j)
                    end_point_sim=check_end(i, False, j, data_sim)
                    start_point_real=check_start2(i, True, j)
                    end_point_real=check_end(i, True, j, data)
                    '''
    





def create_mean(values, x):
    mean_values = []
    for i in range((int(len(values)/x))):
        mean_values.append(np.mean(values[i*x:(i+1)*x])) 
    return mean_values

def check_start2(folderpath, i, real, j): 
    traj=pd.read_csv(csv_loader(folderpath, i, real, j))
    traj['seconds']=create_time_column(traj)
    string='field.position' + str(j + (1- int(real) )*2)
    data=traj.loc[:,string] 

    for i in range(10, len(data)-10): 
    
        std=np.std(data[0:i])
        mean=np.mean(data[0:i])

        if not mean-std*1.6 < data[i] < mean+std*1.6: 
            for x in range(10): 
                if check_interval(data[i+x], mean, std, 1.6) == False: 
                    break
                if x==9: 
                    return i


def check_end(i, real, j, data): 
    '''
    traj=pd.read_csv(csv_loader(i, real, j))
    traj['seconds']=create_time_column(traj)
    string='field.position' + str(j + (1- int(real) )*2)
    data=traj.loc[:,string] 
    '''
    l= len(data)

    for i in range(l-10, 10, -1): 
    
        std=np.std(data[i:l])
        mean=np.mean(data[i:l])

        if not mean-std*3.6 < data[i] < mean+std*3.6: 
            for x in range(10): 
                if check_interval(data[i-x], mean, std, 3.6) == False: 
                    break
                if x==9: 
                    return i


def check_interval(data, mean, std, factor):

    #1std=32% 2std=5%, 3std=0.3% outlier, factor 2std is good I guess?
    
    if not mean-std*factor < data < mean+std*factor: 
        return True
    else: return False

def plan_retrieve(traj, joint):

  filename=folderpath + str(traj) + '/plan_' + str(joint)
  infile = open(filename,'rb')
  plan = pickle.load(infile)
  infile.close()
  return plan

def plan_process(traj, joint):
    plan=plan_retrieve(traj, joint)
    seconds=[]

    seconds = [plan.joint_trajectory.points[i].time_from_start.nsecs/1e9 for i in range(len(plan.joint_trajectory.points))]
    positions = [plan.joint_trajectory.points[i].positions for i in range(len(plan.joint_trajectory.points))]
    velocities = [plan.joint_trajectory.points[i].velocities for i in range(len(plan.joint_trajectory.points))]
    accelerations= [plan.joint_trajectory.points[i].accelerations for i in range(len(plan.joint_trajectory.points))]

    seconds=np.asarray(seconds)
    positions=np.asarray(positions)
    velocities=np.asarray(velocities)
    accelerations=np.asarray(accelerations)


    space=np.linspace(seconds[0], seconds[-1], 1000)


    return seconds, positions, velocities, accelerations


def test_pickledata(): 
  infile_X = open('X_test.p','rb')
  infile_Y = open('Y_test.p', 'rb')
  X = pickle.load(infile_X)
  Y = pickle.load(infile_Y)
  infile_X.close()
  infile_Y.close()

  for i in range(40, len(X)): 
    X_traj = X[i]
    Y_traj = Y[i]
    for k in range(7):
        position_sim = X_traj[:, k]
        position_real = Y_traj[:, k]

        plt.plot(position_real)
        plt.plot(position_sim)
        plt.show()


            


#check_start(1, True, 0)
#plan_process(3,1)
#create_pics_all()
#create_pics()
preprocess()
#align()
