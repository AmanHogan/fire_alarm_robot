"""Acts as a replacement for numpy for micropython. Performs basic trig and matrix operations"""
import math
from globals import *
##################################################
# micro_numpy : Acts as a replacement 
# for numpy for micropython. Performs basic trig
# and matrix operations
# 10/17/2023
##################################################


########## Matrix Math and Matrix Operations #########################################

def dot(a, b) -> list[list[float]]:
    """Multiplies (dot products) two matricies and returns result

    Args:
        a (list[list[float]]): 2D Matrix
        b (list[list[float]]): 2D Matrix

    Returns:
        list[list[float]]: Product of two matricies
    """
    result = [[0.0000, 0.0000, 0.0000], [0.0000, 0.0000, 0.0000], [0.0000, 0.0000, 0.0000]]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                result[i][j] += a[i][k] * b[k][j]
    return result

def eye(n):
    """Returns identity matrix

    Args:
        n (float):

    Returns:
        list[list[float]]: 2D identity matrix
    """
    result = [[1.0000 if i == j else 0.0000 for i in range(n)] for j in range(n)]
    return result

########## Conversions and basic operations ###############################################
def deg2rad(deg):
    return math.radians(deg)

def rad2deg(rad):
    return math.degrees(rad)

def sqrt(x):
    return math.sqrt(x)

def array(data):
    """Returns a matrix using the data sent in. Matrix must be in 2d format: [[], [], []]
    """
    if isinstance(data, list):
        return data
    else:
        raise ValueError("Input must be a list")
    
def hypot(x, y):
  """Calculates the Euclidean distance between two points.

  Args:
    x: The x-coordinate of the first point.
    y: The y-coordinate of the first point.

  Returns:
    The Euclidean distance between the two points.
  """

  return math.sqrt(x**2 + y**2)

################## Trigonometric functions ######################################

def arctan2(y, x):
    return math.atan2(y, x)

def cos(x):
    return math.cos(x)

def sin(x):
    return math.cos(x)

##################################################################################

# Example Usage
if __name__ == '__main__':

    # Create two matrices
    a = array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    b = array([[10, 11, 12], [13, 14, 15], [16, 17, 18]])

    # Calculate the dot product of the two matrices
    result = dot(a, b)

    # Print the result
    print("Dot product: ")
    print(result)
    print("-----------------------")

    # Create a 3x3 identity matrix
    identity_matrix = eye(3)

    # Print the identity matrix
    print("Identity Matrix: ")
    print(identity_matrix)
    print("-----------------------")