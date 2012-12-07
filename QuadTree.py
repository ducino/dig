import sys
from pyglet.gl import *
from BoundingBox import BoundingBox
from Vector import Vector

class QuadNode:
    def __init__(self, depth=0, bbox=None):
        self.ne = self.se = self.sw = self.nw = None
        self.depth = depth
        
        if depth == 0:
            # Let's say our world can only be a third of the maximal float value
            max = 1500#sys.float_info.max/3
            self.bbox = BoundingBox(-max, -max, max, max)
        else:
            if not bbox:
                raise ValueError("No bounding box given")
            self.bbox = bbox
        
        self.items = []
        
    #
    # Add an item to the quad tree
    # item needs to have a member bbox containing it's bounding box
    #
    def add(self, item):
        if self.depth < 8 and self.ne == None:
            cx = self.bbox.center.x
            cy = self.bbox.center.y
        
            self.ne = QuadNode(self.depth + 1, BoundingBox(cx, cy, self.bbox.maxX, self.bbox.maxY))
            self.se = QuadNode(self.depth + 1, BoundingBox(cx, self.bbox.minY, self.bbox.maxX, cy))
            self.sw = QuadNode(self.depth + 1, BoundingBox(self.bbox.minX, self.bbox.minY, cx, cy))
            self.nw = QuadNode(self.depth + 1, BoundingBox(self.bbox.minX, cy, cx, self.bbox.maxY))
            
        if self.depth == 8:
            self.items.append(item)
            return
        
        if self.ne.bbox.overlaps(item.bbox):
            self.ne.add(item)
        
        if self.se.bbox.overlaps(item.bbox):
            self.se.add(item)
        
        if self.sw.bbox.overlaps(item.bbox):
            self.sw.add(item)
        
        if self.nw.bbox.overlaps(item.bbox):
            self.nw.add(item)
            
    #
    # Remove an item from the quad tree
    # This only works if the item still has the same bounding box as it had at the time of adding
    #
    def remove(self, item):
        if self.depth == 8:
            self.items.remove(item)
            return
        
        if self.ne.bbox.overlaps(item.bbox):
            self.ne.remove(item)
        
        if self.se.bbox.overlaps(item.bbox):
            self.se.remove(item)
        
        if self.sw.bbox.overlaps(item.bbox):
            self.sw.remove(item)
        
        if self.nw.bbox.overlaps(item.bbox):
            self.nw.remove(item)
        
            
    def drawCollision(self, item):
        self.bbox.draw()
        
        if self.depth == 8:
            return
        
        if self.ne.bbox.overlaps(item.bbox):
            self.ne.drawCollision(item)
        
        if self.se.bbox.overlaps(item.bbox):
            self.se.drawCollision(item)
        
        if self.sw.bbox.overlaps(item.bbox):
            self.sw.drawCollision(item)
        
        if self.nw.bbox.overlaps(item.bbox):
            self.nw.drawCollision(item)
        
    def collision(self, object):
        offsets = []
        objects = []
        
        self.__collision__(object, offsets, objects)
        
        if len(offsets) == 0:
            return False, None
        
        max = 0
        result = None
        # for offset in offsets:
            # if offset.length() > max:
                # result = offset
                # max = offset.length()
        
        obj = None
        for i in range(len(offsets)):
            offset = offsets[i]
            if offset.length() > max:
                obj = objects[i]
                result = offset
                max = offset.length()
                
        #obj.draw()
                
        p = Vector( (object.bbox.minX+object.bbox.maxX)/2, object.bbox.minY)
        t = p.add(result)
        glPointSize(10)
        glBegin(GL_POINTS)
        glVertex2f(p.x, p.y)
        glEnd()
        glPointSize(1)
        
        glLineWidth(10)
        glBegin(GL_LINES)
        glVertex2f(p.x, p.y)
        glVertex2f(t.x, t.y)
        glEnd()
        glLineWidth(1)
                
        return True, result
            
    
    def __collision__(self, object, offsets, objects):
    
        if self.depth == 8:
            for item in self.items:
                if objects.count(item) == 0:
                    collides, offset = item.collision(object)
                    if collides:
                        offsets.append(offset)
                    objects.append(item)
        
        if self.ne and self.ne.bbox.overlaps(object.bbox):
            self.ne.__collision__(object, offsets, objects)
        
        if self.se and self.se.bbox.overlaps(object.bbox):
            self.se.__collision__(object, offsets, objects)
        
        if self.sw and self.sw.bbox.overlaps(object.bbox):
            self.sw.__collision__(object, offsets, objects)
        
        if self.nw and self.nw.bbox.overlaps(object.bbox):
            self.nw.__collision__(object, offsets, objects)
            
            
    def draw(self, view):
        drawn = []
        self.__draw__(view, drawn)
        # print len(drawn)
        
    def __draw__(self, view, drawn):
        if self.depth == 8:
            for item in self.items:
                if drawn.count(item) == 0:
                    item.draw()
                    drawn.append(item)
                
        if self.ne and self.ne.bbox.overlaps(view.bbox):
            self.ne.__draw__(view, drawn)
        
        if self.se and self.se.bbox.overlaps(view.bbox):
            self.se.__draw__(view, drawn)
        
        if self.sw and self.sw.bbox.overlaps(view.bbox):
            self.sw.__draw__(view, drawn)
        
        if self.nw and self.nw.bbox.overlaps(view.bbox):
            self.nw.__draw__(view, drawn)
        