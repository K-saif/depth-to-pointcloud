import numpy as np

def disparity_to_depth(disparity, fx, baseline):
    depth = (fx * baseline) / disparity
    return depth
