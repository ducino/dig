from pyglet.gl import *
from Vector import Vector
from BoundingBox import BoundingBox
import random

class Polygon:
    def __init__(self):
        self.vertices = []
    
    def addVertex(self, p):
        self.vertices.append(p)
        self.updateBoundingBox()
        
    def draw(self):
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        # glColor3d (.9, .9, .9)
        glColor3d (.1, .1, .1)
        self.__privateDraw__()
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
        glEnable(GL_POLYGON_OFFSET_LINE);
        glColor3d (1., 1., 1.);
        glPolygonOffset(-1.,-1.);
        self.__privateDraw__()
        
    def __privateDraw__(self):
        glBegin(GL_POLYGON)
        size = len(self.vertices)
        for vertex in range(size):
            glVertex2f(self.vertices[vertex].x, self.vertices[vertex].y)
            glVertex2f(self.vertices[(vertex+1)%size].x, self.vertices[(vertex+1)%size].y)
        glEnd()
        
    def collision(self, actor):
    
        if not actor.bbox.overlaps(self.bbox):
            return False, Vector(0,0)
        # Line segment 1: p + t*r
        # Line segment 2: q + u*s
        p = Vector(actor.location.x, actor.location.y-actor.height/2)
        r = Vector(0, actor.height-1)

        size = len(self.vertices)
        for vertex in range(size):
            # t = (q - p) x s /(r x s)
            # u = (q - p) x r /(r x s)
            q = self.vertices[vertex]
            qPlusS = self.vertices[(vertex+1)%size]
            
            s = qPlusS.subtract(q)
            rCrossS = r.cross(s)
            qMinusP = q.subtract(p)
            rhsT = qMinusP.cross(s)
            t = rhsT/rCrossS
            rhsR = qMinusP.cross(r)
            u = rhsR/rCrossS
            
            if t >= 0 and t < 1 and u >= 0 and u < 1:
                return True, p.add(r.multiply(t)).add(r.multiply(0.5))
        return False, Vector(0,0)
    
    def cut(self, other):
        sizeSelf = len(self.vertices)
        sizeOther = len(other.vertices)
        # Line segment 1: p + t*r
        # Line segment 2: q + u*s
        for vertexSelf in range(sizeSelf):
            p = self.vertices[vertexSelf]
            pPlusR = self.vertices[(vertexSelf+1)%sizeSelf]
            r = pPlusR.subtract(p)
            for vertexOther in range(sizeOther):
                # t = (q - p) x s /(r x s)
                # u = (q - p) x r /(r x s)
                q = other.vertices[vertexOther]
                qPlusS = other.vertices[(vertexOther+1)%sizeOther]
                
                s = qPlusS.subtract(q)
                rCrossS = r.cross(s)
                qMinusP = q.subtract(p)
                rhsT = qMinusP.cross(s)
                t = rhsT/rCrossS
                rhsR = qMinusP.cross(r)
                u = rhsR/rCrossS
                
                if t >= 0 and t < 1 and u >= 0 and u < 1:
                    intersect = p.add(r.multiply(t))
                    glColor3f(0.8, 0.8, 0.2)
                    glPointSize(6)
                    intersect.draw()
    
    @staticmethod
    def __leftOf__(v1, v2):
        return v1.x < v2.x or (v1.x == v2.x and v1.y > v2.y)     
        
    def triangulate(self):
        # Monotone decomposition
        # v1; v2; : : : ; vni sorted left to right.
        # Push v1; v2 onto stack.
        # for i = 3 to n do
        #   if vi and top(stack) on same chain
        #       Add diagonals vivj; : : : ; vivk, where
        #       vk is last to admit legal diagonal
        #       Pop vj; : : : ; vk1 and Push vi
        #   else
        #       Add diagonals from vi to all vertices
        #       on the stack and pop them
        #       Save vtop; Push vtop and vi
        
        # Sort vertices left to right
        sortedVertices = []
        size = len(self.vertices)
        if size <= 3:
            return
        for vertex in range(size):
            v = self.vertices[vertex]
            sizeSorted = len(sortedVertices)
            found = False
            for index in range(sizeSorted):
                if Polygon.__leftOf__(v, self.vertices[sortedVertices[index]]):
                    sortedVertices.insert(index, vertex)
                    found = True
                    break
            if not found:
                sortedVertices.append(vertex)
                
        # draw sorted vertices
        glColor3f(1., 1., 0.)
        glBegin(GL_POLYGON)
        for vertex in range(size):
            v = self.vertices[sortedVertices[vertex]]
            glVertex2f(v.x, v.y)
        glEnd()
        
        stack = []
        stack.append(sortedVertices[0])
        stack.append(sortedVertices[1])
        for i in range(3, size):
            if self.__onSameEdge__(stack[0], sortedVertices[i]):
                print "pew"
                
            
    
    # check if two vertices are on the same edge, given their indices
    def __onSameEdge__(self, i1, i2):
        size = len(self.vertices)
        return abs(i1-i2)%size == 1
                    
                    
                    
        
    def magic(self):
        size = len(self.vertices)
        vertices = []
        for vertex in range(size):
            p1 = self.vertices[vertex]
            p2 = self.vertices[(vertex+1)%size]
            vertices.append(p1)
            factor = 10
            p = p1.subtract(p2).perpendicularVector()
            r = factor - random.random()*factor*2
            x = (p1.x + p2.x)/2 + r*p.x
            y = (p1.y + p2.y)/2 + r*p.y
            vertices.append(Vector(x, y))
        self.vertices = vertices
        self.updateBoundingBox()
        
    def updateBoundingBox(self):
        size = len(self.vertices)
        self.bbox = BoundingBox(0,0,0,0)
        for vertex in range(size):
            self.bbox.add(self.vertices[vertex].x, self.vertices[vertex].y)
            
       