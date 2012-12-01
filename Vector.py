from pyglet.gl import *
from math import sqrt

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def cross(self, other):
        return self.x*other.y - self.y*other.x
        
    def dot(self, other):
        return self.x*other.x + self.y*other.y
        
    def add(self, other):
        return Vector(self.x+other.x, self.y+other.y)
        
    def subtract(self, other):
        return Vector(self.x-other.x, self.y-other.y)
        
    def multiply(self, value):
        return Vector(self.x * value, self.y * value)
        
    def perpendicularVector(self):
        d = sqrt(self.x*self.x + self.y*self.y)
        if d == 0:
            return Vector(1, 0)
        return Vector(-self.y / d, self.x / d)
        
    def length(self):
        return sqrt(self.dot(self))
        
    def cosAngle(self, other):
        return self.dot(other) / ( self.length() * other.length() )
        
    def log(self):
        print str(self.x) + ", " + str(self.y)
        
    def draw(self):
        glBegin(GL_POINTS)
        glVertex2f(self.x, self.y)
        glEnd()