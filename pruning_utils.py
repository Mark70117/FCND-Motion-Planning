# based on A-Star-City-Solution Lesson 6: From Grids to Graphs
#  section 5. collinearity exercise & section 9. putting it all together
# IYPPA-6 -- begin
import numpy as np
import math

def collinearity_2D(p1, p2, p3):
    collinear = False
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    if det == 0 :
        collinear = True

    return collinear

def prune_path(path):
    pruned_path = [p for p in path]
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        
        if collinearity_2D(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

# IYPPA-6 -- end 

