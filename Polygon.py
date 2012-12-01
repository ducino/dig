from pyglet.gl import *
from Vector import Vector
from BoundingBox import BoundingBox
import random
from math import acos
import sys

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
        
    @staticmethod
    def __interiorAngleGreaterThanPi__(v1, v2, v3):
        cross = v1.subtract(v2).cross(v3.subtract(v2))
        if cross < 0:
            return True
        return False
   
    # Edge data structure, used for triangulation of the polygon
    class Edge:
        def __init__(self, v1, v2):
            self.v1 = v1
            self.v2 = v2
            self.left = None
            self.right = None
            self.helper = None
        
        def connectRight(self, other):
            self.right = other
            other.left = self
            
        def setHelper(self, v):
            self.helper = v
            
        def xDistTo(self, v):
            # y - y1 = m(x - x1)
            # x = x1 + (y - y1)/m
            m = (self.v1.y - self.v2.y) / (self.v1.x - self.v2.x)
            # glPointSize(10)
            # glBegin(GL_POINTS)
            # glVertex2f(v.x, v.y)
            # glEnd()
            # glPointSize(5)
            # glBegin(GL_POINTS)
            # glVertex2f(self.v1.x + (v.y - self.v1.y)/m, v.y)
            # glEnd()
            # self.mark()
            return abs( v.x - (self.v1.x + (v.y - self.v1.y)/m) )
            
        def mark(self):
            glLineWidth(10)
            glBegin(GL_LINES)
            glVertex2f(self.v1.x, self.v1.y)
            glVertex2f(self.v2.x, self.v2.y)
            glEnd()
            glLineWidth(1)
            
    
    # Diagonal data structure, used for triangulation of the polygon
    class Diagonal:
        # v1 and v2 are the indices in the vertices list
        def __init__(self, v1, v2):
            self.v1 = v1
            self.v2 = v2
        
    # Search tree, implemented initially as a flat array
    class Tree:
        def __init__(self):
            self.edges = []
        
        def add(self, edge):
            self.edges.append(edge)
         
        def remove(self, edge):
            self.edges.remove(edge)
            
        def leftOf(self, v):
            d = sys.float_info.max
            e = None
            for edge in self.edges:
                if (edge.v1.y > v.y and edge.v2.y <= v.y) or \
                    (edge.v1.y <= v.y and edge.v2.y > v.y):
                        de = edge.xDistTo(v)
                        if d > de:
                            d = de
                            e = edge
            return edge
            
    
    def triangulate(self):
        size = len(self.vertices)
        if size <= 3:
            return
        # Monotone decomposition
        # Computation Geometry Algorithms and Applications, page 50
        startVertex = 1
        endVertex = 2
        regularVertex = 3
        mergeVertex = 4
        splitVertex = 5
        
        # test1 = Vector(1, 0)
        # test2 = Vector(2, 0)
        # test3 = Vector(3, 0)
        # print "Length 1 " + str(test1.length())
        # print "Length 2 " + str(test2.length())
        # print "Length 3 " + str(test3.length())
        # print "Angle 1-2-3 : " + str(Polygon.__interiorAngleGreaterThanPi__(test1, test2, test3))
        
        vertexType = []
        # Determine the type for each vertex
        for vertex in range(size):
            v = self.vertices[vertex]
            vPrev = self.vertices[(vertex-1+size)%size]
            vNext = self.vertices[(vertex+1)%size]
            if v.above(vPrev) and v.above(vNext):
                if Polygon.__interiorAngleGreaterThanPi__(vPrev, v, vNext):
                    vertexType.append(splitVertex)
                else:
                    vertexType.append(startVertex)
            elif vPrev.above(v) and vNext.above(v):
                if Polygon.__interiorAngleGreaterThanPi__(vPrev, v, vNext):
                    vertexType.append(mergeVertex)
                else:
                    vertexType.append(endVertex)
            else:
                vertexType.append(regularVertex)
                
        # Construct doubly-linked edge list
        edges = []
        for vertex in range(size):
            v = self.vertices[vertex]
            vPrev = self.vertices[(vertex-1)%size]
            e = Polygon.Edge(v, vPrev)
            if len(edges) > 0:
                edges[-1].connectRight(e)
            edges.append(e)
            if len(edges) == size:
                edges[-1].connectRight(edges[0])
                
        # Sort vertices top to bottom
        # Insertion sort
        sortedVertices = []        
        for vertex in range(size):
            v = self.vertices[vertex]
            sizeSorted = len(sortedVertices)
            found = False
            for index in range(sizeSorted):
                if v.above(self.vertices[sortedVertices[index]]):
                    sortedVertices.insert(index, vertex)
                    found = True
                    break
            if not found:
                sortedVertices.append(vertex)
        
        # Initialize the partition with the edges of the (possibly) not-monotone polygon
        diagonals = []
        tree = Polygon.Tree()        
        
        # Handle each vertex, from top to bottom the polygon
        # Handling each vertex depends on the type of the vertex
        for vertex in sortedVertices:
            print vertex
            if vertexType[vertex] == startVertex:
                e = edges[vertex] 
                e.setHelper(vertex)
                # Add e to searchtree
                tree.add(e)
            elif vertexType[vertex] == endVertex:
                e = edges[(vertex+1)%size]
                helper = e.helper
                # Add a diagonal if helper(e_i+1) is a merge vertex
                if vertexType[helper] == mergeVertex:
                    diagonals.append(Polygon.Diagonal(vertex, helper))
                # Delete e from searchtree
                tree.remove(e)
            elif vertexType[vertex] == splitVertex:
                print "todo"
                    
                
                
        # Monotone decomposition        
        for vertex in range(size):
            v = self.vertices[vertex]
            if vertexType[vertex] == startVertex:
                glPointSize(12)
                glColor3f(1., 2., 2.)
            elif vertexType[vertex] == endVertex:
                glPointSize(3)
                glColor3f(1., 1., 2.)
            elif vertexType[vertex] == splitVertex:
                glPointSize(6)
                glColor3f(2., 1., 1.)
            elif vertexType[vertex] == mergeVertex:
                glPointSize(9)
                glColor3f(2., 2., 1.)
            else:
                glPointSize(1)
                glColor3f(1., 1., 1.)
            
            glColor3f(1., 2., 2.)
            glBegin(GL_POINTS)
            glVertex2f(v.x, v.y)
            glEnd()
        

                
        # draw sorted vertices
        # glColor3f(1., 1., 0.)
        # glBegin(GL_POLYGON)
        # for vertex in range(size):
            # v = self.vertices[sortedVertices[vertex]]
            # glVertex2f(v.x, v.y)
        # glEnd()
    
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
            
       