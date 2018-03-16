
# from Medial-Axis Lesson 6: From Grids to Graphs
#  section 13. Medial Axis Exercise
import numpy as np
import math

def find_skel_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells),   axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    
    return near_start, near_goal

def skel_heuristic_func(position, goal_position):
    return np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)
