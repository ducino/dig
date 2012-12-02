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
        
    def draw(self, r, g, b):
        # glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        # glColor3d (.9, .9, .9)
        # glColor3d (.1, .1, .1)
        # self.__privateDraw__()
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
        glEnable(GL_POLYGON_OFFSET_LINE);
        glColor3d (r, g, b);
        glPolygonOffset(-1.,-1.);
        self.__privateDraw__()
        
    def __privateDraw__(self):
        glBegin(GL_POLYGON)
        size = len(self.vertices)
        for vertex in range(size):
            glVertex2f(self.vertices[vertex].x, self.vertices[vertex].y)
            glVertex2f(self.vertices[(vertex+1)%size].x, self.vertices[(vertex+1)%size].y)
        glEnd()
    
    # Line segment 1: p + t*r
    # Line segment 2: q + u*s
    @staticmethod
    def __LineSegmentIntersection__(p, r, q, s):
        rCrossS = r.cross(s)
        if rCrossS == 0:
            return -1, -1
        qMinusP = q.subtract(p)
        rhsT = qMinusP.cross(s)
        t = rhsT/rCrossS
        rhsR = qMinusP.cross(r)
        u = rhsR/rCrossS
        
        return t, u
        
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
            
            t, u = Polygon.__LineSegmentIntersection__(p, r, q, s)
            
            if t >= 0 and t < 1 and u >= 0 and u < 1:
                return True, p.add(r.multiply(t)).add(r.multiply(0.5))
        return False, Vector(0,0)
        
    def solveSelfIntersections(self):
        return self # TODO remove
        # Try solving the self intersections by swapping vertices
        # at the locations where self intersection occurs
        p1 = self
        p2 = self.__internalSolveSelfIntersections1__()
        count = 0
        while not p1 == p2 and count < 16:
            p1 = p2
            p2 = p1.__internalSolveSelfIntersections1__()
            count = count +1
        
        # If the first attempt failed
        # Start dropping parts (the smalles) of the self intersecting polygon
        if not p1 == p2 and count == 16:
            p1, p2 = self.__internalSolveSelfIntersections2__()
            if len(p1.vertices) > len(p2.vertices):
                return p1.solveSelfIntersections()
            else:
                return p2.solveSelfIntersections()
        else:
            p2.updateBoundingBox()
            return p2

    def __selfIntersects__(self):
        size = len(self.vertices)
        for vertex1 in range(size):
            p = self.vertices[vertex1]
            pPlusR = self.vertices[(vertex1+1)%size]
            r = pPlusR.subtract(p)
            for vertex2 in range(vertex1, size):
                if vertex1 == vertex2:
                    continue
                q = self.vertices[vertex2]
                qPlusS = self.vertices[(vertex2+1)%size]
                s = qPlusS.subtract(q)
                
                t, u = Polygon.__LineSegmentIntersection__(p, r, q, s)
            
                if t >= 0 and t < 1 and u >= 0 and u < 1:
                    return True
        return False
    
    def __internalSolveSelfIntersections1__(self):
        size = len(self.vertices)
        for vertex1 in range(size):
            p = self.vertices[vertex1]
            pPlusR = self.vertices[(vertex1+1)%size]
            r = pPlusR.subtract(p)
            for vertex2 in range(vertex1, size):
                if vertex1 == vertex2:
                    continue
                q = self.vertices[vertex2]
                qPlusS = self.vertices[(vertex2+1)%size]
                s = qPlusS.subtract(q)
                
                t, u = Polygon.__LineSegmentIntersection__(p, r, q, s)
            
                if t >= 0 and t < 1 and u >= 0 and u < 1:
                    print "Intersect"
                    # Swap vertices     
                    newP = Polygon()                    
                    newP.vertices = self.vertices[vertex1+1:vertex2]
                    newP.vertices.reverse()
                    for vertex in range(vertex2, vertex1+size+1):
                        newP.vertices.append(self.vertices[vertex%size])
                    return newP
                    
        return self
        
    def __internalSolveSelfIntersections2__(self):
        size = len(self.vertices)
        for vertex1 in range(size):
            p = self.vertices[vertex1]
            pPlusR = self.vertices[(vertex1+1)%size]
            r = pPlusR.subtract(p)
            for vertex2 in range(vertex1, size):
                if vertex1 == vertex2:
                    continue
                q = self.vertices[vertex2]
                qPlusS = self.vertices[(vertex2+1)%size]
                s = qPlusS.subtract(q)
                
                t, u = Polygon.__LineSegmentIntersection__(p, r, q, s)
            
                if t >= 0 and t < 1 and u >= 0 and u < 1:
                    # One solution: split polygon into 2
                    newV = p.add(r.multiply(t))
                    polygon1 = Polygon()
                    polygon2 = Polygon()
                    start = vertex1
                    stop = vertex2
                    if vertex1 > vertex2:
                        start = vertex2
                        stop = vertex1
                    polygon1.addVertex(newV)
                    for vertex in range(start+1, stop):
                        polygon1.addVertex(self.vertices[vertex])
                    
                    polygon2.addVertex(newV)
                    for vertex in range(stop, start+size):
                        polygon2.addVertex(self.vertices[vertex%size])
                        
                    return polygon1, polygon2
        return self, None
                        
                        
               
    
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
   
    # Vertex data structure, used for triangulation of the polygon
    # class Vertex:
        # def __init__(self, v):
            # self.v = v
            # self.nextList = []
            # self.nextOrder = []
            # self.prev = None
        
        # def addNext(self, next):
            # cross = self.prev.v.subtract(self.v).cross(next.v.subtract(self.v))
            # insert = 0
            # for n in range(len(nextList)):
                # if self.nextOrder[n] >= cross:
                    # insert = n
                    # break
                
            # self.nextList.insert(insert, next)
            # self.nextOrder.insert(insert, cross)
            
    # Doubly Connected edge list: a data structure which can be used in monotone decomposition and triangulation of a polygon
    class DCELHalfEdge:
        def __init__(self, origin):
            self.origin = origin
            self.twin = None
            self.next = None
            self.prev = None
            self.helper = None
            
        def start(self):
            return self.origin.v
        
        def end(self):
            return self.next.origin.v
            
        def xDistTo(self, v):
            # y - y1 = m(x - x1)
            # x = x1 + (y - y1)/m
            m = (self.start().y - self.end().y) / (self.start().x - self.end().x)
            # glPointSize(10)
            # glBegin(GL_POINTS)
            # glVertex2f(v.x, v.y)
            # glEnd()
            # glPointSize(5)
            # glBegin(GL_POINTS)
            # glVertex2f(self.v1.x + (v.y - self.v1.y)/m, v.y)
            # glEnd()
            # self.mark()
            return abs( v.x - (self.start().x + (v.y - self.start().y)/m) )
            
        def mark(self):
            glLineWidth(10)
            glBegin(GL_LINES)
            glVertex2f(self.start().x, self.start().y)
            glVertex2f(self.end().x, self.end().y)
            glEnd()
            glLineWidth(1)
            
    class DCELVertex:
        def __init__(self, v):
            self.v = v
            self.incidentEdge = None

    # Search tree, implemented initially as a flat array (Laziness)
    class Tree:
        def __init__(self):
            self.edges = []
        
        def add(self, edge):
            self.edges.append(edge)
         
        def remove(self, edge):
            self.edges.remove(edge)
            
        def edgeLeftOf(self, v):
            d = sys.float_info.max
            e = None
            for edge in self.edges:
                if (edge.start().y > v.y and edge.end().y <= v.y) or \
                    (edge.start().y <= v.y and edge.end().y > v.y):
                        de = edge.xDistTo(v)
                        if d > de:
                            d = de
                            e = edge
            return e
       
    def __addDiagonal__(self, vi1, vi2, dcelVertices):
        print "todo"
        # v1 = self.vertices[vi1]
        # v2 = self.vertices[vi2]
        # print "--"
        # print str(vi1) + " : " + str(v1.x) + ", " + str(v1.y)
        # print str(vi2) + " : " + str(v2.x) + ", " + str(v2.y)
        # size = len(edges)
        # e11 = edges[vi1]
        # e12 = edges[(vi1+1)%size]
        # e21 = edges[vi2]
        # e22 = edges[(vi2+1)%size]
        
        # glColor3f(1., 1., .2)
        # glBegin(GL_LINES)
        # glVertex2f(v1.x, v1.y)
        # glVertex2f(v2.x, v2.y)
        # glEnd()
        
        # d1 = Polygon.Edge(v1, v2)
        # d2 = Polygon.Edge(v2, v1)
                
        # e11.connectRight(d1)
        # d1.connectRight(e22)
        
        # e21.connectRight(d2)
        # d2.connectRight(e12)
    
    # Construct the Doubly connected edge list
    # \param dcelVertices A list of DCELVertex objects, must be empty
    def __constructDCEL__(self, dcelVertices):
        size = len(self.vertices)
        lastVertex = None
        previousVertex = None
        currentVertex = None
        # Create all DCEL Vertices with an incident edge
        for i in range(size):
            # Create the current vertex or use the previously stored last one, if this is the last vertex
            v = self.vertices[i]
            if i == size-1:
                currentVertex = lastVertex
            else:
                currentVertex = Polygon.DCELVertex(v)
            # For the first vertex, the previous one is not known yet, create it and remember it as the last vertex
            if i == 0:                
                vPrev = self.vertices[(i-1)%size]
                lastVertex = Polygon.DCELVertex(vPrev)
                previousVertex = lastVertex
            # Create the DCEL Half Edge, origin: previous vertex
            currentEdge = Polygon.DCELHalfEdge(previousVertex)
            previousVertex = currentVertex
                
            # The incident edge of the current vertex is the previously create half edge with origin: previous vertex
            currentVertex.incidentEdge = currentEdge
            dcelVertices.append(currentVertex)
        
        # Create all twin half edges
        for i in range(size):
            currentVertex = dcelVertices[i]
            currentEdge = currentVertex.incidentEdge
            twinEdge = Polygon.DCELHalfEdge(currentVertex)
            currentEdge.twin = twinEdge
            twinEdge.twin = currentEdge
            
        # Connect all edges
        for i in range(size):
            prevVertex = dcelVertices[i-1]
            currentVertex = dcelVertices[i]
            nextVertex = dcelVertices[(i+1)%size]
            
            currentEdge = currentVertex.incidentEdge
            currentEdge.prev = prevVertex.incidentEdge
            currentEdge.next = nextVertex.incidentEdge
            
            twinEdge = currentEdge.twin
            twinEdge.prev = nextVertex.incidentEdge
            twinEdge.next = prevVertex.incidentEdge             
                
    def __ySortVertices__(self, sortedVertices):
        size = len(self.vertices)
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
    
    @staticmethod
    def __constructPolygonsFromDoublyLinkedEdges__(edges, polygons):
        print "todo"
        # for edge1 in edges:
            # if not edge1.visited:
                # edge2 = edge1
                # p = Polygon()
                # p.addVertex(edge2.v1)
                # while not edge2.visited:
                    # p.addVertex(edge2.v2)
                    # edge2.visited = True
                    # edge2 = edge2.right
                # polygons.append(p)
            
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
                
        # Construct doubly-connected edge list
        dcelVertices = []
        self.__constructDCEL__(dcelVertices)    
                
        # Sort vertices top to bottom
        # Insertion sort
        sortedVertices = []        
        self.__ySortVertices__(sortedVertices)
                        
        # draw sorted vertices
        # glColor3f(1., 1., 0.)
        # glBegin(GL_POLYGON)
        # for vertex in range(size):
            # v = self.vertices[sortedVertices[vertex]]
            # glVertex2f(v.x, v.y)
        # glEnd()
        
        # Initialize the partition with the edges of the (possibly) not-monotone polygon
        tree = Polygon.Tree()        
        print "-------------"
        # Handle each vertex, from top to bottom the polygon
        # Handling each vertex depends on the type of the vertex
        for vertex in sortedVertices:
            v = self.vertices[vertex]
            if vertexType[vertex] == startVertex:
                ei = dcelVertices[vertex].incidentEdge
                ei.helper = vertex
                # Add e to searchtree
                tree.add(ei)
            elif vertexType[vertex] == endVertex or vertexType[vertex] == mergeVertex:
                # Perform actions which are the same of end or merge vertices
                eiPlus1 = dcelVertices[(vertex+1)%size].incidentEdge
                # Add a diagonal if helper(e_i+1) is a merge vertex
                if vertexType[eiPlus1.helper] == mergeVertex:
                    self.__addDiagonal__(vertex, eiPlus1.helper, dcelVertices)
                # Delete e_i+1 from searchtree
                tree.remove(eiPlus1)
                # Perform actions which are unique to a merge vertex
                if vertexType[vertex] == mergeVertex:
                    # Find edge directly to the left of vertex
                    ej = tree.edgeLeftOf(v)
                    if vertexType[ej.helper] == mergeVertex:
                        self.__addDiagonal__(vertex, ej.helper, dcelVertices)
                        # Set vertex as new helper
                        ej.helper = vertex
            elif vertexType[vertex] == splitVertex:
                # Find edge directly to the left of vertex
                ej = tree.edgeLeftOf(v)
                self.__addDiagonal__(vertex, ej.helper, dcelVertices)
                # Set vertex as new helper
                ej.helper = vertex
                ei = dcelVertices[vertex].incidentEdge
                # Add e_i to tree and set vertex as helper
                ei.helper = vertex
                tree.add(ei)
            elif vertexType[vertex] == regularVertex:
                #If the interior of the polygon lies right of this vertex
                if self.vertices[vertex-1].y <= v.y and \
                   self.vertices[(vertex+1)%size].y > v.y:
                    # glPointSize(20)
                    # glBegin(GL_POINTS)
                    # glVertex2f(v.x, v.y)
                    # glEnd()
                    # glPointSize(1)
                    eiPlus1 = dcelVertices[(vertex+1)%size].incidentEdge
                    helper = eiPlus1.helper
                    # Add a diagonal if helper(e_i+1) is a merge vertex
                    if vertexType[helper] == mergeVertex:
                        print "Reg 1"
                        self.__addDiagonal__(vertex, helper, dcelVertices)
                    # Delete e_i+1 from searchtree
                    tree.remove(eiPlus1)
                    # Add e_i to tree and set vertex as helper
                    ei = dcelVertices[vertex].incidentEdge
                    ei.helper = vertex
                    tree.add(ei)
                else:
                    # glPointSize(10)
                    # glBegin(GL_POINTS)
                    # glVertex2f(v.x, v.y)
                    # glEnd()
                    # glPointSize(1)
                    # Find edge directly to the left of vertex
                    ej = tree.edgeLeftOf(v)
                    if vertexType[ej.helper] == mergeVertex:
                        print "Reg 2"
                        self.__addDiagonal__(vertex, ej.helper, dcelVertices)
                        # Set vertex as new helper
                        ej.helper = vertex
        
        # for diag in diagonals:
            # v1 = self.vertices[diag.v1]
            # v2 = self.vertices[diag.v2]
            # glColor3f(1., 1., .2)
            # glBegin(GL_LINES)
            # glVertex2f(v1.x, v1.y)
            # glVertex2f(v2.x, v2.y)
            # glEnd()
        
        # Construct the monotones
        self.monotones = []
        #Polygon.__constructPolygonsFromDoublyLinkedEdges__(edges, self.monotones)
                        
        # for m in self.monotones:
            # if len(m.vertices) == 3:
                # continue
            
            # monotoneEdges = []
            # m.__constructDoublyLinkedEdgeList__(monotoneEdges)
            
            # monotoneSortedVertices = []
            # m.__ySortVertices__(monotoneSortedVertices)
            
            # stack = []
            # stack.append(monotoneSortedVertices[0])
            # stack.append(monotoneSortedVertices[1])
            # for j in range(3, len(monotoneSortedVertices)):
                # if m.__onSameEdge__(stack[-1], monotoneSortedVertices[j]):
                    # poppedLast = stack.pop()
                    # vStack = stack[-1]
                    # while not Polygon.__interiorAngleGreaterThanPi__(m.vertices[vStack], m.vertices[poppedLast], m.vertices[monotoneSortedVertices[j]]):
                        # m.__addDiagonal__(vStack, monotoneSortedVertices[j], monotoneEdges)
                        # poppedLast = stack.pop()
                        # if len(stack) == 0:
                            # break;
                        # vStack = stack[-1]
                    
                    # stack.append(poppedLast)
                    # stack.append(monotoneSortedVertices[j])
                # else:
                    # while len(stack) > 0:
                        # vi = stack.pop()
                        # m.__addDiagonal__(vi, monotoneSortedVertices[j], monotoneEdges)
                    # stack.append(monotoneSortedVertices[j-1])
                    # stack.append(monotoneSortedVertices[j])

            # m.triangles = []
            # Polygon.__constructPolygonsFromDoublyLinkedEdges__(monotoneEdges, m.triangles)
            # for t in m.triangles:
                # t.draw(1., 1., .2)
            
        #    for m in self.monotones:         
            #    m.draw(1.0, 0.2, 0.2)
            
        # self.monotones[0].draw()
        
        
        # Monotone decomposition        
        # for vertex in range(size):
            # v = self.vertices[vertex]
            # if vertexType[vertex] == startVertex:
                # glPointSize(12)
            # elif vertexType[vertex] == endVertex:
                # glPointSize(3)
            # elif vertexType[vertex] == splitVertex:
                # glPointSize(6)
            # elif vertexType[vertex] == mergeVertex:
                # glPointSize(9)
            # else:
                # glPointSize(1)
            
            # glColor3f(1., 2., 2.)
            # glBegin(GL_POINTS)
            # glVertex2f(v.x, v.y)
            # glEnd()

    # check if two vertices are on the same edge, given their indices
    def __onSameEdge__(self, i1, i2):
        size = len(self.vertices)
        return abs(i1-i2)%size == 1

    # Apply midpoint displacement to this polygon
    # The midpoint of each line segment will be displaced randomly
    # perpendicular to the line segment, distance between -factor and +factor
    def midpointDisplacement(self, factor):
        #seed = random.randint(0, 10000)
        seed = 6160
        print seed
        random.seed(seed)
        size = len(self.vertices)
        oldVertices = list(self.vertices)
        vertices = []
        for vertex in range(size):
            p1 = self.vertices[vertex]
            p2 = self.vertices[(vertex+1)%size]
            vertices.append(p1)
            p = p1.subtract(p2).perpendicularVector()
            r = factor - random.random()*factor*2
            x = (p1.x + p2.x)/2 + r*p.x
            y = (p1.y + p2.y)/2 + r*p.y
            vertices.append(Vector(x, y))
        self.vertices = vertices
        if self.__selfIntersects__():
            print "uh oh"
            self.vertices = oldVertices
            self.midpointDisplacement(factor)
        self.updateBoundingBox()

    def updateBoundingBox(self):
        size = len(self.vertices)
        self.bbox = BoundingBox(0,0,0,0)
        for vertex in range(size):
            self.bbox.add(self.vertices[vertex].x, self.vertices[vertex].y)
            
       