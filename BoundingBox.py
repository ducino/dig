from pyglet.gl import *
from Vector import Vector

class BoundingBox:
    # improve speed by storing extents and center?
    def __init__(self, minX, minY, maxX, maxY):
        self.minX = minX
        self.minY = minY
        self.maxX = maxX
        self.maxY = maxY
        
    def add(self, x, y):
        if self.minX > x:
            self.minX = x
        if self.maxX < x:
            self.maxX = x
        if self.minY > y:
            self.minY = y
        if self.maxY < y:
            self.maxY = y
            
    def draw(self):
        glColor3ub(0x10, 0xA7, 0xE3)
        glBegin(GL_POLYGON)
        glVertex2f(self.minX, self.minY)
        glVertex2f(self.maxX, self.minY)
        glVertex2f(self.maxX, self.maxY)
        glVertex2f(self.minX, self.maxY)           
        glEnd()
        
    def center(self):
        return Vector( (self.minX+self.maxX)/2, (self.minY+self.maxY)/2)
        
    def extents(self):
        return Vector( (self.maxX-self.minX)/2, (self.maxY-self.minY)/2 )
        
    def overlaps(self, other):
        # http://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?page=3
        t = other.center().subtract(self.center())
        
        selfExtents = self.extents()
        otherExtents = other.extents()

        return abs(t.x) <= (selfExtents.x+otherExtents.x) and abs(t.y) <= (selfExtents.y+otherExtents.y)
     