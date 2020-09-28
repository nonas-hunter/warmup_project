#!/usr/bin/env python3
import math

class PotentialField:
    GOAL = True
    OBSTACLE = False

    def __init__(self):
        self.points = []
        self.alpha = 0.5
        self.beta = 0.5
        self.inf = 100
        
    
    def add_point(self, x_object, y_object, state=False, radius=0.1, spread=3):   
        self.points.append({"x":x_object, "y":y_object, "radius":radius, "type":state, "spread":spread}) 

    def calc_gradient(self, x_agent, y_agent, point):
        distance, angle = self.calc_distance_angle(x_agent, y_agent, point["x"], point["y"])
        x, y = (0, 0)
        if point["type"] == True:
            if distance < point["radius"]:
                x, y = (0, 0)
            elif point["radius"] <= distance <= point["radius"] + point["spread"]:
                x = self.alpha * (distance - point["radius"]) * math.cos(angle)
                y = self.alpha * (distance - point["radius"]) * math.sin(angle)
            elif distance > point["radius"] + point["spread"]:
                x = self.alpha * point["spread"] * math.cos(angle)
                y = self.alpha * point["spread"]* math.sin(angle)
                
        elif point["type"] == False:
            if distance < point["radius"] :
                x = -self.sign(math.cos(angle)) * self.inf
                y = -self.sign(math.sin(angle)) * self.inf
            elif point["radius"] <= distance <= point["radius"] + point["spread"]:
                x = -self.beta * (point["spread"] + point["radius"] - distance) * math.cos(angle)
                y = -self.beta * (point["spread"] + point["radius"] - distance) * math.sin(angle)
            elif distance > point["radius"] + point["spread"]:
                x, y = (0, 0)
        return (x, y)

    def calc_output(self, x_agent, y_agent):
        delta_x = 0
        delta_y = 0
        for point in self.points:
            x, y = self.calc_gradient(x_agent, y_agent, point)
            delta_x += x
            delta_y += y
            # print("x: " + str(x))
            # print("y: " + str(y))
        # print("x: " + str(delta_x))
        # print("y: " + str(delta_y))
        if delta_x == 0 and delta_y == 0:
            velocity = 0
        else:
            velocity = math.sqrt((delta_x**2) + (delta_y**2)) / abs(math.sqrt((delta_x**2) + (delta_y**2)))
        angle = math.atan2(delta_y, delta_x)
        
        return (velocity, angle)
            
    def calc_distance_angle(self, x_agent, y_agent, x_object, y_object):
        angle = math.atan2((y_object - y_agent), (x_object - x_agent))
        distance = math.sqrt((x_object - x_agent)**2 + (y_agent - y_object)**2)
        return (distance, angle)
    
    def sign(self, num):
        if num == 0:
            return 1
        return num / abs(num)
    def clear(self):
        self.points = []

if __name__ == "__main__":
    test = PotentialField()
    test.add_point(0.5,0)
    print(test.calc_output(0,0))
