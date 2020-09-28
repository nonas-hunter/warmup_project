#!/usr/bin/env python3
import numpy
import math

def point_to_global(x, y):
        theta = math.radians(0)
        point = [x, y, 1]
        rot_matrix = [[math.cos(theta), -math.sin(theta), 0], \
                                  [math.sin(theta), math.cos(theta), 0], \
                                  [0, 0, 1]]
        trans_matrix = [[1, 0 ,0], \
                                    [0, 1, 0], \
                                    [0, 0, 1]]
        temp1 = multiply(rot_matrix, point)
        temp2 = multiply(trans_matrix, temp1)
        return temp2[0], temp2[1]

def multiply(matrix, vector):
        output = [0,0,0]
        for i in range(len(matrix)):
            for j in range(len(matrix[i])):
                output[i] += matrix[i][j] * vector[j]
        return output

if __name__ == "__main__":
    print(point_to_global(1,0))