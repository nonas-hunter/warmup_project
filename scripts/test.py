#!/usr/bin/env python3
import numpy
import math

def point_to_global(x, y):
        theta = 0
        x_NEATO = 1
        y_NEATO = 2
        point = numpy.array([[x], [y], [1]])
        rot_matrix = numpy.array([[math.cos(theta), math.sin(theta), 0], \
                                 [-math.sin(theta), math.cos(theta), 0], \
                                 [0, 0, 1]])
        trans_matrix = numpy.array([[1, 0 ,x_NEATO], \
                                   [0, 1, y_NEATO], \
                                   [0, 0, 1]])
        temp = numpy.dot(rot_matrix, point)
        print(temp)
        temp2 = numpy.dot(trans_matrix, temp)
        print(temp2)
        return (temp2[0,0], temp2[1,0])


if __name__ == "__main__":
    print(point_to_global(1,0))