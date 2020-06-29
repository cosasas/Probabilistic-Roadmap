import random
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial import cKDTree
from scipy.spatial import distance
import copy
import matplotlib.animation as animation



SAMPLES = 400

line_with_distance = [] 

def closest_point_inLine(begin, end, obj):
    line_vector = end - begin    
    line_vector_fromObstacle = obj - begin     
    projection = (np.dot(line_vector_fromObstacle, line_vector) / np.dot(line_vector, line_vector))

    if projection < 0:               
    # then closest point is the start of the segment
        closest = begin  
    elif projection > 1:
    # then closest point is the end of the segment
        closest = end
    else:
        closest = begin + projection * line_vector
    
    distance_closest_point_toObstacle = distance.euclidean(closest, obj)
    return distance_closest_point_toObstacle

def check_dublicate_line(tmp):
    global line_with_distance    
    
    drop = False
    size = 0
    tmp_stacked_line = []
    if line_with_distance:
        tmp_stacked_line = np.stack(line_with_distance)

        size = np.size(tmp_stacked_line)//5    
        for i in range(0,size):
            if tmp[0] == tmp_stacked_line[i][2] and tmp[1] == tmp_stacked_line[i][3]:
                drop = True
            if tmp[0] == tmp_stacked_line[i][0] and tmp[0] == tmp_stacked_line[i][2]:   
                drop = True
    return drop

def line_collision(x_values2, y_values2):
    global line_with_distance    
    line_coordinates = np.concatenate((x_values2, y_values2)) 
     
    obj_circle1 = (1.5, 2.1)
    obj_circle2 = (1, 1.7)
    
    begin = line_coordinates[1]
    end = line_coordinates[0]
    
    distance_closest_point_toObstacle = closest_point_inLine(begin, end, obj_circle1)
    distance_closest_point_toObstacle2 = closest_point_inLine(begin, end, obj_circle2)
    
    bool_collision_answer = False
    if distance_closest_point_toObstacle < 0.25 or distance_closest_point_toObstacle2 < 0.27:
        bool_collision_answer = True
    if not bool_collision_answer:
        line_distance = distance.euclidean(begin, end)
        tmp = np.hstack((begin, end, line_distance)) # matrix form -> [X1,Y1,X2,Y2,distance]
        drop = check_dublicate_line(tmp)
        if not drop:
            line_with_distance.append(tmp)
          
    return bool_collision_answer
    
def find_samples_start_and_goal(SampleArrayXY):

    start_position = (0.5, 2.7)
    goal_position = (3.5, 0.3)
    
    size = np.size(SampleArrayXY)//2
    total_start = 0
    total_goal = 0
    samples_atStart = []
    samples_atGoal = []
    for i in range(0,size):
        dst = distance.euclidean(SampleArrayXY[i], start_position) # check collision with euclidean distance
        dst2 = distance.euclidean(SampleArrayXY[i], goal_position)
    
        if dst < 0.15:
            print("sample ", SampleArrayXY[i] , " at start")
            total_start = total_start + 1
            samples_atStart.append(SampleArrayXY[i])
        if dst2 < 0.15:
            print("sample ", SampleArrayXY[i] ," at goal")
            total_goal = total_goal + 1
            samples_atGoal.append(SampleArrayXY[i])
    print("total start ", total_start)
    print("total goal ", total_goal)
    
    return samples_atStart, samples_atGoal
        
def split(array, ii, B):
    
    split0 = array
    split0 = np.reshape(split0, (-1,2))

    x_values = copy.copy(B)
    y_values = copy.copy(split0)
    
    bool_collision_answer = line_collision(x_values, y_values)
 
    temp = x_values[0][1]
    x_values[0][1] = y_values[0][0]
    y_values[0][0] = temp
    
    return x_values[0], y_values[0], bool_collision_answer
    
def connect_nodes2(array):
    array = np.reshape(array, (np.size(array)//2,2))
    tree = cKDTree(array)
    size = np.size(array)//2
    for item in array:
        #TheResult = tree.query(item)  #, k=8, distance_upper_bound=1000)
        dd, ii = tree.query(item, k=[2,4,8])
        B = np.reshape(item, (-1, 2))
        
        x_values2, y_values2, bool_collision_answer2 = split(array[ii[0]], ii, B)
        x_values3, y_values3, bool_collision_answer3 = split(array[ii[1]], ii, B)
        x_values4, y_values4, bool_collision_answer4 = split(array[ii[2]], ii, B)
        
        if not bool_collision_answer2: # it's true for collision
            plt.plot(x_values2, y_values2,"y")
            b = np.concatenate((x_values2, y_values2))
            plt.pause(0.001)

        if not bool_collision_answer3:
            plt.plot(x_values3, y_values3,"c")
            b = np.concatenate((x_values3, y_values3))
            plt.pause(0.001)

        if not bool_collision_answer4:
            plt.plot(x_values4, y_values4, "r")
            b = np.concatenate((x_values4, y_values4))
            plt.pause(0.001)

    
def sample_collision(x,y):
    
    sample_coordinates = (x, y)    
    obj_circle1 = (1.5, 2.1)
    obj_circle2 = (1, 1.7)
    #print(sample_coordinates)
    dst = distance.euclidean(sample_coordinates, obj_circle1) # check collision with euclidean distance
    dst2 = distance.euclidean(sample_coordinates, obj_circle2)
    
    if dst > 0.3 and dst2 > 0.3:
        collision_free = 1
    else:
        collision_free = 0 
    
    return collision_free

def random_samples(ox, oy, numberOfSamples):
    
    max_x = max(ox)-0.05
    max_y = max(oy)-0.05
    min_x = min(ox)+0.05
    min_y = min(oy)+0.05
    
    sample_x, sample_y = [], []
    collision_free = 0
    while len(sample_x) <= numberOfSamples:
        while collision_free == 0:
            randomSample_x = (max_x - min_x) * np.random.random_sample() + min_x
            randomSample_y = (max_y - min_y) * np.random.random_sample() + min_y
            collision_free = sample_collision(randomSample_x, randomSample_y)
            
        sample_x.append(randomSample_x)
        sample_y.append(randomSample_y)
        collision_free = 0
        
    stack_xySamples = np.vstack((sample_x, sample_y))
    
    return sample_x, sample_y
     
def main():


    ox = []
    oy = []

    for i in range(400):
        ox.append(i*0.01)
        oy.append(0.0)
    for i in range(400):
        ox.append(i*0.01)
        oy.append(3.0)
    for i in range(300):
        oy.append(i*0.01)
        ox.append(0.0)        
    for i in range(300):
        oy.append(i*0.01)
        ox.append(4.0)
        
    plt.show()
    plt.plot(ox, oy, ".k")

    circle1 = plt.Circle((1, 1.7), 0.25, color='red')
    circle2 = plt.Circle((1.5, 2.1), 0.25, color='red')
    circle3 = plt.Circle((0.5, 2.7), 0.15, color='blue')
    circle4 = plt.Circle((3.5, 0.3), 0.15, color='green')

    fig = plt.gcf()
    ax = fig.gca()
    ax.add_artist(circle1)
    ax.add_artist(circle2)
    ax.add_artist(circle3)
    ax.add_artist(circle4)

    sample_x, sample_y = random_samples(ox, oy, SAMPLES)
    
    plt.plot(sample_x, sample_y, ".k")
    
    plt.grid(True)

    SampleArrayXY = np.vstack((sample_x, sample_y)).T

    SampleArrayXY = np.array(SampleArrayXY)
    connect_nodes2(SampleArrayXY)
    
    samples_atStart, samples_atGoal = find_samples_start_and_goal(SampleArrayXY)
    ################################
    all_lines_from_graph = np.stack(line_with_distance) # use this for a_star
    ###############################
   
if __name__ == '__main__':
    main()