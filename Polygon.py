import sys
import time
import random
from pyglet.gl import *
from math import sqrt
from Vector import Vector
from BoundingBox import BoundingBox

class CollisionInfo:
    def __init__(self, bbox):
        # The projection onto the vertical and horizontal axis
        self.horizontalMin = sys.float_info.max
        self.horizontalMax = -(sys.float_info.max-1)
        self.verticalMin = sys.float_info.max
        self.verticalMax = -(sys.float_info.max-1)
        
        self.horizontalAxis = Vector(1, 0)
        self.verticalAxis = Vector(0, 1)
        
        # The bounding box of the actor
        self.bbox = bbox
    
    #
    # Add a horizontal projection and check for min and max values
    #    
    def addHorizontalProjection(self, projection):
        if projection < self.horizontalMin:
            self.horizontalMin = projection
            
        if projection > self.horizontalMax:
            self.horizontalMax = projection
    
    #
    # Add a vertical projection and check for min and max values
    #
    def addVerticalProjection(self, projection):
        if projection < self.verticalMin:
            self.verticalMin = projection
            
        if projection > self.verticalMax:
            self.verticalMax = projection
            
    #
    # Check if there was a collision given the projections
    #
    def collisionDetected(self):
        dist = CollisionInfo.__getIntervalDistance__(self.horizontalMin, self.horizontalMax, self.bbox.minY, self.bbox.maxY)
        if dist > 0:
            return False
        
        dist = CollisionInfo.__getIntervalDistance__(self.verticalMin, self.verticalMax, self.bbox.minX, self.bbox.maxX)
        if dist > 0:
            return False
            
        return True
        
    #
    # Merge two collision info objects
    #
    def add(self, other):
        if self.horizonalMin > other.horizontalMin:
            self.horizontalMin = other.horizontalMin
         
        if self.horizonalMax < other.horizontalMax:
            self.horizontalMax = other.horizontalMax
            
        if self.verticalMin > other.verticalMin:
            self.verticalMin = other.verticalMin
         
        if self.verticalMax < other.verticalMax:
            self.verticalMax = other.verticalMax
            

class Polygon:
    def __init__(self):
        self.vertices = []
        self.monotones = None
        self.triangles = None
        self.bbox = None
        
        # If this is False, this does not mean it is no convex, but if it is True, it is!
        self.convex = False
    
    def addVertex(self, p):
        self.vertices.append(p)
        self.updateBoundingBox()
        
        if len(self.vertices) == 3:
            self.convex = True
        else:
            self.convex = False
    
    #
    # Draw this polygon
    #
    def draw(self):
        self.__draw__(0)
    
    def __draw__(self, level):
        if len(self.vertices) < 3:
            raise ValueError("This ain't no polygon! A duogon, at best!")
            
        if len(self.vertices) == 3:
            glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
            glColor3f (.1, .1, .1)
            glBegin(GL_TRIANGLES)
            glVertex2f(self.vertices[0].x, self.vertices[0].y)
            glVertex2f(self.vertices[1].x, self.vertices[1].y)
            glVertex2f(self.vertices[2].x, self.vertices[2].y)
            glEnd()
            
        if self.monotones and self.triangles:
            raise ValueError("A polygon containing both monotones and triangles? Something went wrong")
            
        if self.monotones:
            for m in self.monotones:
                m.__draw__(level + 1)
                
        if self.triangles:
            for t in self.triangles:
                t.__draw__(level + 1)
         
        if level == 0:
            glColor3f(.9, .9, .9)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glEnable(GL_POLYGON_OFFSET_LINE);
            glPolygonOffset(-1.,-1.);
            self.__drawPolygon__()

    def __drawPolygon__(self):
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
    
    #
    # Check for collisions
    # \return True|False, CollisionVector
    #
    def collisionActor(self, actor):
    
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
                # return True, p.add(r.multiply(t)).add(r.multiply(0.5))
                return True, r.multiply(t)#.add(Vector(0, -.001))
        return False, Vector(0,0)
              
    #
    # Check if this polygon collides with another polygon
    # \return True|False, CollisionVector
    #
    def collision(self, other):        
        collides, axis, dist = self.__privateCollision__(other)
        
        if collides:
            return True, axis.multiply(dist)
        
        return False, None
            
    def __privateCollision__(self, other):
        if not self.bbox.overlaps(other.bbox):
            return False, None, None
            
        if self.convex:
            minAxis = None
            minDist = sys.float_info.max
            collides, minAxis, minDist = self.__privateCollisionConvex__(other, minAxis, minDist)
            if not collides:
                return False, minAxis, minDist
                
            return other.__privateCollisionConvex__(self, minAxis, minDist)
            
        if self.monotones:
            collides = False
            newCollides = False
            newAxis = None
            newDist = None
            axis = None
            dist = -1
            for m in self.monotones:
                newCollides, newAxis, newDist = m.__privateCollision__(other)
                if newCollides:
                    collides = True
                    if newDist > dist:
                        dist = newDist
                        axis = newAxis
                        
            return collides, axis, dist        
            
        if self.triangles:
            collides = False
            newCollides = False
            newAxis = None
            newDist = None
            axis = None
            dist = -1
            for t in self.triangles:
                newCollides, newAxis, newDist = t.__privateCollision__(other)
                if newCollides:
                    collides = True
                    if newDist > dist:
                        dist = newDist
                        axis = newAxis
                        
            return collides, axis, dist    

            
    # Internal collision method
    def __privateCollisionConvex__(self, other, minAxis, minDist):
        size = len(self.vertices)
        for i in range(size):
            v1 = self.vertices[i]
            v2 = self.vertices[(i+1)%size]
            
            perp = Polygon.__perpendicularVector__(self, v1, v2)
            
            minSelf = maxSelf = minOther = maxOther = None
            
            minSelf, maxSelf = self.__projectOnAxis__(perp)
            minOther, maxOther = other.__projectOnAxis__(perp)
            
            dist = Polygon.__getIntervalDistance__(minSelf, maxSelf, minOther, maxOther)
            
            if dist > 0.:
                return False, minAxis, minDist
            elif abs(dist) < minDist:
                minDist = abs(dist)
                minAxis = perp
        return True, minAxis, minDist
            
            
    #
    # Project all vertices of this polygon onto an axis
    # Get the min and max values
    #
    def __projectOnAxis__(self, axis):
        min = max = axis.dot(self.vertices[0])
        
        for i in range(1, len(self.vertices)):
            product = axis.dot(self.vertices[i])
            
            if product < min:
                min = product
            
            if product > max:
                max = product
                
        return min, max
    
    #
    # Get the distance beween two 1-dimensional intervals
    #
    @staticmethod
    def __getIntervalDistance__(min1, max1, min2, max2):
        if min1 < min2:
            return min2 - max1
        else:
            return min1 - max2
            
    #
    # Get the perpendicular vector on a line segment defined by 2 poins
    #
    @staticmethod
    def __perpendicularVector__(self, v1, v2):
        dy = (v1.y - v2.y)
        dx = (v1.x - v2.x)
        d = sqrt(dy*dy + dx*dx)
        if d == 0:
            return Vector(1, 0)
        return Vector(-dy / d, dx / d)
            
    
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

    # TODO
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
                    
    #
    # Check if the given point is inside the polygon
    #
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

        # glColor3f(1., 1., 1.)
        # glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)
        # if False:
            # self.__privateDraw__()
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
                
        # Sort vertices top to bottom
        # Insertion sort
        sortedVertices = []        
        self.__ySortVertices__(sortedVertices)
        
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

        # Construct the monotones
        self.monotones = []
        Polygon.__constructPolygonsFromDCEL__(dcelVertices, self.monotones)
        
        # print "Monotones: " + str(len(self.monotones))
        # if self.debug == 2:
            # c = int(time.clock()*2)%(len(self.monotones)+1)
            # if c == len(self.monotones):
                # glColor3f(1., 1., .2)
                # self.__privateDraw__()
                # return
            # glColor3f(.2, .2, 1.)
            # self.monotones[c].__privateDraw__()
            # for polygon in self.monotones:
                # polygon.__privateDraw__()
            # return
        
        # for m in self.monotones:
            # glColor3f(1., 1., 1.)
            # m.__privateDraw__()

        for m in self.monotones:
        
            if len(m.vertices) == 3:
                # if self.debug == 1 or self.debug == 3:
                    # m.draw(1., .6, .0)
                continue
                       
            monoDcelVertices = []
            m.__constructDCEL__(monoDcelVertices)

            monotoneSortedVertices = []
            m.__ySortVertices__(monotoneSortedVertices)
          
            stack = []
            stack.append(monotoneSortedVertices[0])
            stack.append(monotoneSortedVertices[1])

            for j in range(2, len(monotoneSortedVertices)-1):
            
                if m.__onSameEdge__(stack[-1], monotoneSortedVertices[j]):
                
                    poppedLast = stack.pop()
                    vStack = stack[-1]
                    
                    while m.__canAddDiagonal__(vStack, monotoneSortedVertices[j]):
                        m.__addDiagonal__(vStack, monotoneSortedVertices[j], monoDcelVertices)
                        poppedLast = stack.pop()
                        if len(stack) == 0:
                            break;
                        vStack = stack[-1]
                    
                    stack.append(poppedLast)
                    stack.append(monotoneSortedVertices[j])
                else:
                    while len(stack) > 0:
                        vi = stack.pop()
                        if len(stack) > 0:
                            m.__addDiagonal__(vi, monotoneSortedVertices[j], monoDcelVertices)
                    stack.append(monotoneSortedVertices[j-1])
                    stack.append(monotoneSortedVertices[j])

            stack.pop()
            while len(stack) > 1:
                m.__addDiagonal__(stack[-1], monotoneSortedVertices[-1], monoDcelVertices)
                stack.pop()

            m.triangles = []
            Polygon.__constructPolygonsFromDCEL__(monoDcelVertices, m.triangles)
            
            # if self.debug == 1:
                # m.triangles[int(time.clock()*2)%len(m.triangles)].draw(1., .2, .2)
            # if self.debug == 3:
                # for t in m.triangles:
                    # t.draw(1., .2, .2)

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
        self.bbox = BoundingBox(self.vertices[0].x,self.vertices[0].y,self.vertices[0].x,self.vertices[0].y)
        for vertex in range(1, size):
            self.bbox.add(self.vertices[vertex].x, self.vertices[vertex].y)
    
    @staticmethod
    def createBoundingBoxPolygon(pMin, pMax):
        p = Polygon()
        p.addVertex(Vector(pMin.x, pMax.y))
        p.addVertex(Vector(pMin.x, pMin.y))
        p.addVertex(Vector(pMax.x, pMin.y))
        p.addVertex(Vector(pMax.x, pMax.y))
        return p
            
       