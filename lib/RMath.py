def minClamp(min, val):
    return min if val < min else val

def maxClamp(max, val):
    return max if val > max else val

def clamp(min, max, val):
    return maxClamp(max, minClamp(min, val))