from pyglet.gl import *
from Vector import Vector
from BoundingBox import BoundingBox
import random
from math import acos, floor
import sys
import time

class Polygon:
    def __init__(self):
        self.vertices = []
        self.monotones = None
        self.triangles = None
        self.debug = 0
    
    def addVertex(self, p):
        self.vertices.append(p)
        self.updateBoundingBox()
        
    def draw(self, r, g, b):
        if self.debug == 2:
            return
        if self.debug == 1:
            glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
            glEnable(GL_POLYGON_OFFSET_LINE);
            glColor3d (r, g, b);
            glPolygonOffset(-1.,-1.);
            self.__privateDraw__()
            return
        
        size = len(self.vertices)

        if size <= 3:
            glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
            glEnable(GL_POLYGON_OFFSET_LINE);
            glColor3d (r, g, b);
            glPolygonOffset(-1.,-1.);
            self.__privateDraw__()
            return
    
        if self.monotones:
            for m in self.monotones:
                m.draw(r,g,b)
            return
        
        if self.triangles:
            for t in self.triangles:
                # print "Triangle " + str(len(t.vertices))
                t.draw(r,g,b)
            return
        print "wtf"
        # glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        # glColor3d (.9, .9, .9)
        # glColor3d (.1, .1, .1)
        # self.__privateDraw__()

        
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
                    
    # Check if the given point is inside the polygon
    def inside(self, p):
        size = len(self.vertices)
        count = 0
        for i in range(size):
            v1 = self.vertices[i]
            v2 = self.vertices[(i+1)%size]
            
            if (v1.y > p.y and v2.y <= p.y) or \
                (v1.y <= p.y and v2.y > p.y):
                            
                div = (v1.x - v2.x)
                if div == 0:
                    if p.x - v1.x > 0:
                        count += 1
                m = (v1.y - v2.y) / div
                if p.x - (v1.x + (p.y - v1.y)/m) > 0:
                    count += 1
        return count%2 == 1
        
    @staticmethod
    def __interiorAngleGreaterThanPi__(v1, v2, v3):
        cross = v1.subtract(v2).cross(v3.subtract(v2))
        if cross < 0:
            return True
        return False
      
    # Doubly Connected edge list: a data structure which can be used in monotone decomposition and triangulation of a polygon
    class DCELHalfEdge:
        def __init__(self, origin):
            self.origin = origin
            self.twin = None
            self.next = None
            self.prev = None
            self.face = None
            self.helper = None
            
        def start(self):
            return self.origin.v
        
        def end(self):
            return self.next.origin.v
            
        def xDistTo(self, v):
            # y - y1 = m(x - x1)
            # x = x1 + (y - y1)/m
            div = (self.start().x - self.end().x)
            if div == 0:
                return v.x - self.start().x
            m = (self.start().y - self.end().y) / div
            # glPointSize(10)
            # glBegin(GL_POINTS)
            # glVertex2f(v.x, v.y)
            # glEnd()
            # glPointSize(5)
            # glBegin(GL_POINTS)
            # glVertex2f(self.v1.x + (v.y - self.v1.y)/m, v.y)
            # glEnd()
            # self.mark()
            return v.x - (self.start().x + (v.y - self.start().y)/m)
            
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
            self.leavingEdge = None
            
    class DCELFace:
        def __init__(self, edge):
            self.edge = edge
            self.visited = False
            
    class DCELNextEdgeIterator:
        def __init__(self, edge):
            self.start = edge
            self.current = edge
            
        def next(self):
            self.current = self.current.next
            if self.current == self.start:
                return None
            return self.current
            
    class DCELLeavingEdgeIterator:
        def __init__(self, edge):
            self.start = edge
            self.current = edge
            
        def next(self):
            self.current = self.current.prev.twin
            if self.current == self.start:
                return None
            return self.current
            

    # Search tree, implemented initially as a flat array (Laziness)
    class Tree:
        def __init__(self):
            self.edges = []
            # self.count = 0
        
        def add(self, edge):
            self.edges.append(edge)
         
        def remove(self, edge):
            self.edges.remove(edge)
            
        def edgeLeftOf(self, v):
            d = sys.float_info.max
            e = None
            for edge in self.edges:
                if (edge.start().y >= v.y and edge.end().y < v.y) or \
                    (edge.start().y < v.y and edge.end().y >= v.y):
                        distToEdge = edge.xDistTo(v)                        
                        if distToEdge > 0 and d > distToEdge:
                            d = distToEdge
                            e = edge
            
            # if self.count == int(time.clock()*2):
                # glPointSize(20)
                # glBegin(GL_POINTS)
                # glVertex2f(v.x, v.y)
                # glEnd()
                # glPointSize(1)
                # if e:
                    # e.mark()
            # self.count = (self.count+1)%25
            return e
        def draw(self):
            print "hmpf"
            for edge in self.edges:
                glLineWidth(6)
                glBegin(GL_LINES)
                glVertex2f(edge.start().x, edge.start().y)
                glVertex2f(edge.end().x, edge.end().y)
                glEnd()
                glLineWidth(1)
                
    # Is it possible to add a diagonal between two vertices
    def __canAddDiagonal__(self, vi1, vi2):
        size = len(self.vertices)
        p = self.vertices[vi1]
        pPlusR = self.vertices[vi2]
        r = pPlusR.subtract(p)
        for v in range(size):
            # if (vi1==v and vi2==(v+1)%size) or (vi2==v and vi1==(v+1)%size):
                # return False
            q = self.vertices[v]
            qPlusS = self.vertices[(v+1)%size]
            s = qPlusS.subtract(q)
            
            t, u = Polygon.__LineSegmentIntersection__(p, r, q, s)
            
            if t > 0 and t < 1 and u > 0 and u < 1:
                return False
                
        midDiagonal = Vector((p.x+pPlusR.x)/2, (p.y+pPlusR.y)/2)
        return self.inside(midDiagonal)
            
       
    # Add a diagonal to the DCEL of this polygon
    def __addDiagonal__(self, vi1, vi2, dcelVertices):
        v1 = dcelVertices[vi1]
        v2 = dcelVertices[vi2]
        
        # Find the common face for v1 and v2
        face = None
        e1 = v1.leavingEdge 
        it1 = Polygon.DCELLeavingEdgeIterator(e1)
        while e1:
            face1 = e1.face
            e2 = v2.leavingEdge
            it2 = Polygon.DCELLeavingEdgeIterator(e2)
            while e2:
                if face1 == e2.face and face1 != None:
                    face = face1
                    break
                e2 = it2.next()
            e1 = it1.next()
        
        # Find the next and previous edges for both diagonals
        prevEdge1 = None
        nextEdge1 = None
        prevEdge2 = None
        nextEdge2 = None

        e = face.edge
        it = Polygon.DCELNextEdgeIterator(e)
        while e:
            if e.origin == v1:
                prevEdge1 = e.prev
                nextEdge1 = e
            if e.origin == v2:
                prevEdge2 = e.prev
                nextEdge2 = e
            e = it.next()
        
        # Connect 2 new half edges
        diagonal1 = Polygon.DCELHalfEdge(v1)
        diagonal1.prev = prevEdge1
        prevEdge1.next = diagonal1
        diagonal1.next = nextEdge2
        nextEdge2.prev = diagonal1
        
        diagonal2 = Polygon.DCELHalfEdge(v2)
        diagonal2.prev = prevEdge2
        prevEdge2.next = diagonal2
        diagonal2.next = nextEdge1
        nextEdge1.prev = diagonal2

        diagonal1.twin = diagonal2
        diagonal2.twin = diagonal1
        
        # Connect new faces
        face1 = Polygon.DCELFace(diagonal1)
        face2 = Polygon.DCELFace(diagonal2)
        e1 = diagonal1
        it1 = Polygon.DCELNextEdgeIterator(e1)
        while e1:
            e1.face = face1
            e1 = it1.next()
        
        e2 = diagonal2
        it2 = Polygon.DCELNextEdgeIterator(e2)
        while e2:
            e2.face = face2
            e2 = it2.next()

    
    # Construct the Doubly connected edge list
    # \param dcelVertices A list of DCELVertex objects, must be empty
    def __constructDCEL__(self, dcelVertices):
        size = len(self.vertices)
        currentVertex = None
        # Create all DCEL Vertices with a leaving edge
        for i in range(size):
            # Create the current vertex or use the previously stored last one, if this is the last vertex
            v = self.vertices[i]
            currentVertex = Polygon.DCELVertex(v)
            # Create the DCEL Half Edge
            currentEdge = Polygon.DCELHalfEdge(currentVertex)
            # Connect
            currentVertex.leavingEdge = currentEdge
            dcelVertices.append(currentVertex)
        
        # Create all twin half edges
        for i in range(size):
            currentVertex = dcelVertices[i]
            nextVertex = dcelVertices[(i+1)%size]
            currentEdge = currentVertex.leavingEdge
            twinEdge = Polygon.DCELHalfEdge(nextVertex)
            currentEdge.twin = twinEdge
            twinEdge.twin = currentEdge
            
        # Connect all edges
        for i in range(size):
            prevVertex = dcelVertices[i-1]
            currentVertex = dcelVertices[i]
            nextVertex = dcelVertices[(i+1)%size]
            
            currentEdge = currentVertex.leavingEdge
            currentEdge.prev = prevVertex.leavingEdge
            currentEdge.next = nextVertex.leavingEdge
            
            twinEdge = currentEdge.twin
            twinEdge.prev = nextVertex.leavingEdge.twin
            twinEdge.next = prevVertex.leavingEdge.twin

        # Connect the single face
        startVertex = dcelVertices[0]
        startEdge = startVertex.leavingEdge
        face = Polygon.DCELFace(startEdge)
        
        e = startEdge
        it = Polygon.DCELNextEdgeIterator(e)
        while e:
            e.face = face
            e = it.next()
        
                
    # Sort the vertices of this polygon according to y coordinate
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
    def __constructPolygonsFromDCEL__(dcelVertices, polygons):
        # Iterator all leaving edges on all vertices to find all faces
        for vertex in dcelVertices:
            e1 = vertex.leavingEdge
            it1 = Polygon.DCELLeavingEdgeIterator(e1)
            polygon = Polygon()
            while e1:
                if e1.face and not e1.face.visited:
                    e1.face.visited = True
                    polygon = Polygon()
                    e2 = e1.face.edge
                    it2 = Polygon.DCELNextEdgeIterator(e2)
                    while e2:
                        polygon.addVertex(e2.origin.v)
                        e2 = it2.next()
                    polygons.append(polygon)
                e1 = it1.next()
            
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
            vPrev = self.vertices[vertex-1]
            vNext = self.vertices[(vertex+1)%size]
            if v.above(vPrev) and v.above(vNext):
                if Polygon.__interiorAngleGreaterThanPi__(vNext, v, vPrev):
                    vertexType.append(splitVertex)
                else:
                    vertexType.append(startVertex)
            elif vPrev.above(v) and vNext.above(v):
                if Polygon.__interiorAngleGreaterThanPi__(vNext, v, vPrev):
                    vertexType.append(mergeVertex)
                else:
                    vertexType.append(endVertex)
            else:
                vertexType.append(regularVertex)

        glColor3f(1., 1., 1.)
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)
        if False:
            self.__privateDraw__()
            for vertex in range(size):
                v = self.vertices[vertex]
                if vertexType[vertex] == startVertex:
                    glPointSize(12)
                elif vertexType[vertex] == endVertex:
                    glPointSize(3)
                elif vertexType[vertex] == splitVertex:
                    glPointSize(6)
                elif vertexType[vertex] == mergeVertex:
                    glPointSize(9)
                else:
                    glPointSize(1)
                
                glColor3f(1., 2., 2.)
                glBegin(GL_POINTS)
                glVertex2f(v.x, v.y)
                glEnd()
        
        
        # Construct doubly-connected edge list
        dcelVertices = []
        self.__constructDCEL__(dcelVertices)
        
       
        #
        # TEST CODE : Check DCEL
        #
        # polys = []
        # Polygon.__constructPolygonsFromDCEL__(dcelVertices, polys)
        
        # c = int(time.clock()*2)%(len(polys)+1)
        # if c == len(polys):
            # glColor3f(1., 1., .2)
            # self.__privateDraw__()
            # return
        # glColor3f(.2, .2, 1.)
        # polys[c].__privateDraw__()
        
        #
        # TEST CODE : Check prev, next, twin edges
        #
        # c = int(time.clock()*2)%(len(dcelVertices))
        # v = dcelVertices[c]
        # glPointSize(30)
        # glBegin(GL_POINTS)
        # glVertex2f(v.v.x, v.v.y)
        # glEnd()
        # glPointSize(1)
        # v.leavingEdge.twin.mark()
        # glColor3f(1., 1., .2)
        # v.leavingEdge.twin.next.mark()
        # return
        
        #
        # TEST CODE : Check edgeLeftOf
        #
        # tree = Polygon.Tree()
        # e1 = Polygon.DCELHalfEdge(Polygon.DCELVertex(Vector(-1, 1)))
        # e2 = Polygon.DCELHalfEdge(Polygon.DCELVertex(Vector(-1, -1)))
        # e1.next = e2
        # e2.prev = e1
        # tree.add(e1)
        # tree.edgeLeftOf(Vector(0, 0)).mark()
        # return
        

        # glEnable(GL_POLYGON_OFFSET_LINE)
        # self.__privateDraw__()
        # return
        
        # start.mark()
        # e = dcelVertices[0].incidentEdge
        # it = Polygon.DCELEdgeIterator(e)
        # while e:
            # e.mark()                     
            # e = it.next()
        # return
            
        # e = dcelVertices[0].incidentEdge
        # it = Polygon.DCELIncidentEdgeIterator(e)
        # while e:
            # e.mark()                     
            # e = it.next()            
        # return
        
        
        # Debug code for construct DCEL
        # c = int(time.clock())
        # vtest = dcelVertices[c%len(self.vertices)]
        # glPointSize(20)
        # glBegin(GL_POINTS)
        # glVertex2f(vtest.v.x, vtest.v.y)
        # glEnd()
        # glPointSize(1)
        # glColor3f(1., 1., 1.)
        # vtest.incidentEdge.twin.mark()
        # glColor3f(1., 1., .2)
        # vtest.incidentEdge.twin.prev.mark()
        # glColor3f(1., .2, .2)
        # vtest.incidentEdge.twin.next.mark()
                
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

        # Handle each vertex, from top to bottom the polygon
        # Handling each vertex depends on the type of the vertex
        for vertex in sortedVertices:
            v = self.vertices[vertex]
            if vertexType[vertex] == startVertex:
                ei = dcelVertices[vertex].leavingEdge
                ei.helper = vertex
                # Add e_i to searchtree
                tree.add(ei)
            elif vertexType[vertex] == endVertex or vertexType[vertex] == mergeVertex:
                # Perform actions which are the same of end or merge vertices
                eiMinus1 = dcelVertices[vertex-1].leavingEdge
                # Add a diagonal if helper(e_i+1) is a merge vertex
                if vertexType[eiMinus1.helper] == mergeVertex:
                    self.__addDiagonal__(vertex, eiMinus1.helper, dcelVertices)
                # Delete e_i from searchtree
                tree.remove(eiMinus1)
                # Perform actions which are unique to a merge vertex
                if vertexType[vertex] == mergeVertex:
                    # Find edge directly to the right of vertex
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
                ei = dcelVertices[vertex].leavingEdge
                # Add e_i to tree and set vertex as helper
                ei.helper = vertex
                tree.add(ei)
            elif vertexType[vertex] == regularVertex:
                #If the interior of the polygon lies right of this vertex
                if self.vertices[(vertex+1)%size].y <= v.y and \
                   self.vertices[vertex-1].y > v.y:
                    eiMinus1 = dcelVertices[vertex-1].leavingEdge
                    helper = eiMinus1.helper
                    # Add a diagonal if helper(e_i-1) is a merge vertex
                    if vertexType[helper] == mergeVertex:
                        self.__addDiagonal__(vertex, helper, dcelVertices)
                    # Delete e_i-1 from searchtree
                    tree.remove(eiMinus1)
                    # Add e_i to tree and set vertex as helper
                    ei = dcelVertices[vertex].leavingEdge
                    ei.helper = vertex
                    tree.add(ei)
                else:
                    # Find edge directly to the left of vertex
                    ej = tree.edgeLeftOf(v)
                    if not ej:
                        glPointSize(5)
                        v.draw()
                        glPointSize(1)
                        return
                    if vertexType[ej.helper] == mergeVertex:
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
        Polygon.__constructPolygonsFromDCEL__(dcelVertices, self.monotones)
        
        # for m in self.monotones:
            # if len(m.vertices) == len(self.vertices):
                # self.monotones.remove(m)
        
        # print "Monotones: " + str(len(self.monotones))
        if self.debug == 2:
            c = int(time.clock()*2)%(len(self.monotones)+1)
            if c == len(self.monotones):
                glColor3f(1., 1., .2)
                self.__privateDraw__()
                return
            glColor3f(.2, .2, 1.)
            self.monotones[c].__privateDraw__()
            # for polygon in self.monotones:
                # polygon.__privateDraw__()
            return
        
        for m in self.monotones:
            glColor3f(1., 1., 1.)
            m.__privateDraw__()
            
        # random.seed(int(time.clock()))
        # p = Vector(random.randrange(-10, 10), random.randrange(-10, 10))
        # glPointSize(30)
        # glBegin(GL_POINTS)
        # glVertex2f(p.x, p.y)
        # glEnd()
        # glPointSize(1)
        # print self.inside(p)
        # return
        
        testCount = 0
        for m in self.monotones:
        # if True:
            # m = self.monotones[1]
            if len(m.vertices) == 3:
                if self.debug == 1 or self.debug == 3:
                    m.draw(1., .6, .0)
                continue
                       
            monoDcelVertices = []
            m.__constructDCEL__(monoDcelVertices)
                        
            # polys = []
            # Polygon.__constructPolygonsFromDCEL__(monoDcelVertices, polys)
            # for p in polys:
                # glColor3f(.2, 1., .2);
                # p.__privateDraw__()
            # return
                        
            monotoneSortedVertices = []
            m.__ySortVertices__(monotoneSortedVertices)
          
            stack = []
            stack.append(monotoneSortedVertices[0])
            stack.append(monotoneSortedVertices[1])
            

            
            for j in range(2, len(monotoneSortedVertices)-1):
            
                if m.__onSameEdge__(stack[-1], monotoneSortedVertices[j]):
                    # if testCount == int(time.clock()%12):
                        # print "======"
                        # glColor3f(1., .2, 1.)
                        # glBegin(GL_LINES)
                        # glVertex2f(m.vertices[monotoneSortedVertices[j]].x, m.vertices[monotoneSortedVertices[j]].y)
                        # glVertex2f(m.vertices[stack[-1]].x, m.vertices[stack[-1]].y)
                        # glEnd()
                    # testCount = testCount+1
                    # if testCount == int(time.clock()%12):
                        # glPointSize(20)
                        # for v in stack:
                            # m.vertices[v].draw()
                        # return
                        # glPointSize(1)
                    # testCount = testCount+1
                
                    poppedLast = stack.pop()
                    vStack = stack[-1]
                    
                    # if testCount == int(time.clock()%12):
                        # glColor3f(1., .2, 1.)
                        # glBegin(GL_LINES)
                        # glVertex2f(m.vertices[monotoneSortedVertices[j]].x, m.vertices[monotoneSortedVertices[j]].y)
                        # glVertex2f(m.vertices[vStack].x, m.vertices[vStack].y)
                        # glEnd()
                    # testCount = testCount+1
                    while m.__canAddDiagonal__(vStack, monotoneSortedVertices[j]):
                        # polys = []
                        # Polygon.__constructPolygonsFromDCEL__(monoDcelVertices, polys)
                        # for p in polys:
                            # glColor3f(.2, 1., .2);
                            # p.__privateDraw__()
                        # return
                        
                        # if testCount == int(time.clock()%12):
                            # glColor3f(1., .2, 1.)
                            # glBegin(GL_LINES)
                            # glVertex2f(m.vertices[monotoneSortedVertices[j]].x, m.vertices[monotoneSortedVertices[j]].y)
                            # glVertex2f(m.vertices[vStack].x, m.vertices[vStack].y)
                            # glEnd()
                        
                        # if testCount == 1:
                            # glPointSize(20)
                            # glBegin(GL_POINTS)
                            # glVertex2f(m.vertices[poppedLast].x, m.vertices[poppedLast].y)
                            # glVertex2f(m.vertices[monotoneSortedVertices[j]].x, m.vertices[monotoneSortedVertices[j]].y)
                            # glVertex2f(m.vertices[vStack].x, m.vertices[vStack].y)
                            # glEnd()
                            # glPointSize(1)
                            # print "roar"
                            # return
                        # testCount = testCount+1
                        
                        m.__addDiagonal__(vStack, monotoneSortedVertices[j], monoDcelVertices)
                        poppedLast = stack.pop()
                        if len(stack) == 0:
                            break;
                        vStack = stack[-1]
                    
                    stack.append(poppedLast)
                    stack.append(monotoneSortedVertices[j])
                else:
                    # if testCount == int(time.clock()%12):
                        # print "======"
                        # glColor3f(1., .2, 1.)
                        # glBegin(GL_LINES)
                        # glVertex2f(m.vertices[monotoneSortedVertices[j]].x, m.vertices[monotoneSortedVertices[j]].y)
                        # glVertex2f(m.vertices[stack[-1]].x, m.vertices[stack[-1]].y)
                        # glEnd()
                    # testCount = testCount+1
                    while len(stack) > 0:
                        vi = stack.pop()
                        # glPointSize(20)
                        # glBegin(GL_POINTS)
                        # glVertex2f(m.vertices[vi].x, m.vertices[vi].y)
                        # glVertex2f(m.vertices[monotoneSortedVertices[j]].x, m.vertices[monotoneSortedVertices[j]].y)
                        # glEnd()
                        # glPointSize(1)
                        # return

                        # if testCount == int(time.clock()%12):
                            # glColor3f(1., .2, 1.)
                            # glBegin(GL_LINES)
                            # glVertex2f(m.vertices[monotoneSortedVertices[j]].x, m.vertices[monotoneSortedVertices[j]].y)
                            # glVertex2f(m.vertices[vi].x, m.vertices[vi].y)
                            # glEnd()
                        # testCount = testCount+1
                        if len(stack) > 0:
                            m.__addDiagonal__(vi, monotoneSortedVertices[j], monoDcelVertices)
                    stack.append(monotoneSortedVertices[j-1])
                    stack.append(monotoneSortedVertices[j])

            stack.pop()
            while len(stack) > 1:
                m.__addDiagonal__(stack[-1], monotoneSortedVertices[-1], monoDcelVertices)
                stack.pop()
                
            # if len(stack) == 3:
                # tri = Polygon()
                # tri.addVertex(m.vertices[stack.pop()])
                # tri.addVertex(m.vertices[stack.pop()])
                # tri.addVertex(m.vertices[stack.pop()])
                # m.triangles.append(tri)

            m.triangles = []
            Polygon.__constructPolygonsFromDCEL__(monoDcelVertices, m.triangles)
            if self.debug == 1:
                m.triangles[int(time.clock()*2)%len(m.triangles)].draw(1., .2, .2)
            if self.debug == 3:
                for t in m.triangles:
                    t.draw(1., .2, .2)
            
        # for m in self.monotones:   
            # if len(m.vertices) == 3:
                # m.draw(1., .2, .2)
        
        # c = int(time.clock()/2)%(len(self.monotones))
        # m = self.monotones[c]
        # if len(m.vertices) == 3:
            # m.__privateDraw__()
        # else:   
            # c2 = int(time.clock()*15)%(len(m.triangles))
            # m.triangles[c2].__privateDraw__()
    
    
    # check if two vertices are on the same edge, given their indices
    def __onSameEdge__(self, i1, i2):
        size = len(self.vertices)
        return abs(i1-i2) == 1 or abs(i1-i2) == size-1

    # Apply midpoint displacement to this polygon
    # The midpoint of each line segment will be displaced randomly
    # perpendicular to the line segment, distance between -factor and +factor
    def midpointDisplacement(self, factor):
        seed = random.randint(0, 10000)
        # if factor == 8:
            # seed = 8977
            # seed = 9539
            # seed = 1909
            # seed = 5021
            # seed = 6131 # Triangulation issue
            # seed = 6198 # Triangulation issue
            # seed = 3403 # Triangulation issue
        print factor
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
            self.midpointDisplacement(factor/2)
        self.updateBoundingBox()

    def updateBoundingBox(self):
        size = len(self.vertices)
        self.bbox = BoundingBox(0,0,0,0)
        for vertex in range(size):
            self.bbox.add(self.vertices[vertex].x, self.vertices[vertex].y)
            
       