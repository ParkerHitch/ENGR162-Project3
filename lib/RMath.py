def minClamp(min, val):
    return min if val < min else val

def maxClamp(max, val):
    return max if val > max else val

def clamp(min, max, val):
    return maxClamp(max, minClamp(min, val))

def mod(a, b):
    return ((a % b) + b) % b

def in2cm(inches):
    return inches * 2.54

def cm2in(centimeters):
    return centimeters / 2.54