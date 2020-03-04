import random
import math

XDIM = 2.50
YDIM = 2.50

class Workspace_Objects(object):    
    def __init__(self,n):
        self.n = n
        self.objs = {}
        for i in range(0,self.n):
            new_obj_index = i+1
            self.objs["object_"+str(20-new_obj_index)] = self.random_stick()

    def random_stick(self):
        t = 0.0001
        length = min( XDIM, YDIM)/3.0
        x = (XDIM - 2*length)*random.random()+length
        y = (YDIM - 2*length)*random.random()+length
        theta = 2*math.pi*random.random()
        a = math.tan(theta)
        b = y-a*x
        x_min = x
        x_max = x + length/math.sqrt(1+a**2)
        y_min = y
        y_max = x_max *a + b
        p1 = (x_min + t*math.cos(theta+math.pi/2.0), y_min + t*math.sin(theta+math.pi/2.0))
        p2 = (x_min + t*math.cos(theta-math.pi/2.0), y_min + t*math.sin(theta-math.pi/2.0))
        p3 = (x_max + t*math.cos(theta-math.pi/2.0), y_max + t*math.sin(theta-math.pi/2.0))
        p4 = (x_max + t*math.cos(theta+math.pi/2.0), y_max + t*math.sin(theta+math.pi/2.0)) 
        return [(p1[0], p1[1]), (p2[0], p2[1]), (p3[0], p3[1]), (p4[0], p4[1])]

        # self.P1 = vg.Point(self.x_min, self.a*self.x_min+self.b)
        # self.P2 = vg.Point(self.x_max, self.a*self.x_max+self.b)