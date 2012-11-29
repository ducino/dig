import pyglet
import random
from math import sqrt
from pyglet.gl import *
from pyglet.window import key

# TODO intersection of polygons
# TODO triangulation
# TODO merging of polygons?
# TODO solve self intersecting polygons
# TODO controls proper acceleration
# TODO proper clipping

window = pyglet.window.Window(width=800, height=600)
#window.push_handlers(pyglet.window.event.WindowEventLogger())

translateZ = -100

label = pyglet.text.Label('Midpoint displacement test',
    font_name='Times New Roman',
    font_size=36,
    x=window.width//2, y=window.height//2,
    anchor_x='center', anchor_y='center')
    
def drawLineSegment(p1, p2):
    glVertex2f(p1.x, p1.y)
    glVertex2f(p2.x, p2.y)
    
    
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
    
class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def cross(self, other):
        return self.x*other.y - self.y*other.x
        
    def add(self, other):
        return Vector(self.x+other.x, self.y+other.y)
        
    def subtract(self, other):
        return Vector(self.x-other.x, self.y-other.y)
        
    def multiply(self, value):
        return Vector(self.x * value, self.y * value)
        
    def log(self):
        print str(self.x) + ", " + str(self.y)
        
    def draw(self):
        glBegin(GL_POINTS)
        glVertex2f(self.x, self.y)
        glEnd()
        
class Actor:
    def __init__(self):
        self.speed = Vector(0, 0)
        self.width = 5
        self.height = 15
        self.canJump = True
        self.location = Vector(0, 11)
        self.updateLocation(Vector(0, 10))
    
    def draw(self):
        glBegin(GL_POLYGON)
        glVertex2f(self.location.x-self.width/2, self.location.y-self.height/2)
        glVertex2f(self.location.x+self.width/2, self.location.y-self.height/2)
        glVertex2f(self.location.x+self.width/2, self.location.y+self.height/2)
        glVertex2f(self.location.x-self.width/2, self.location.y+self.height/2)
        glEnd()
        
    def accelerate(self, vector, dt):
        maxSpeedX = 5
        maxSpeedY = 20
        self.speed.x = self.speed.x + vector.x*dt
        if self.speed.x > maxSpeedX:
            self.speed.x = maxSpeedX
            
        self.speed.y = self.speed.y + vector.y*dt
        if self.speed.y > maxSpeedY:
            self.speed.y = maxSpeedY
        
    def gravity(self, dt):
        self.accelerate(Vector(0, -10), dt)
        
    def applySpeed(self, dt):
        self.location.x = self.location.x + self.speed.x
        self.location.y = self.location.y + self.speed.y
        
    def updateLocation(self, newLocation):
        self.previousLocation = self.location
        self.location = newLocation
        self.bbox = BoundingBox(self.location.x-self.width/2, self.location.y-self.height/2,
                                self.location.x+self.width/2, self.location.y+self.height/2)
    
    
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
            drawLineSegment(self.vertices[vertex], self.vertices[(vertex+1)%size])
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
                    
        
    def magic(self):
        size = len(self.vertices)
        vertices = []
        for vertex in range(size):
            p1 = self.vertices[vertex]
            p2 = self.vertices[(vertex+1)%size]
            line = Line(p1.x, p1.y, p2.x, p2.y)
            vertices.append(p1)
            factor = 10
            p = line.perpendicularVector()
            r = factor - random.random()*factor*2
            x = (line.x1 + line.x2)/2 + r*p.x
            y = (line.y1 + line.y2)/2 + r*p.y
            vertices.append(Vector(x, y))
        self.vertices = vertices
        self.updateBoundingBox()
        
    def updateBoundingBox(self):
        size = len(self.vertices)
        self.bbox = BoundingBox(0,0,0,0)
        for vertex in range(size):
            self.bbox.add(self.vertices[vertex].x, self.vertices[vertex].y)
            
            
        
def createBoundingBoxPolygon(pMin, pMax):
    p = Polygon()
    p.addVertex(Vector(pMin.x, pMin.y))
    p.addVertex(Vector(pMax.x, pMin.y))
    p.addVertex(Vector(pMax.x, pMax.y))
    p.addVertex(Vector(pMin.x, pMax.y))
    return p


class Line:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
    
    # Returns a vector perpendicular to this line
    def perpendicularVector(self):
        dy = (self.y1 - self.y2)
        dx = (self.x1 - self.x2)
        d = sqrt(dy*dy + dx*dx)
        if d == 0:
            return Vector(1, 0)
        return Vector(-dy / d, dx / d)
        
    # Returns a line perpendicular to this line of unit length
    def perpendicularLine(self):
        dy = (self.y1 - self.y2)
        dx = (self.x1 - self.x2)
        hx = (self.x1 + self.x2)/2
        hy = (self.y1 + self.y2)/2
        return Line(hx, hy, hx - dy, hy + dx)        

class Node:
    def __init__(self):
        self.partition = Vector(0, 1)
        self.location = Vector(0, 0)
        self.lines = []
        #self.nodeLeft = Node()
        #self.nodeRight = Node()
        
    def add(self, line):
        self.lines.append(line)
        
    def draw(self):
        glBegin(GL_LINES)
        for line in self.lines:
            drawLine(line)
        glEnd()
        d = 10
        glBegin(GL_LINES)
        glVertex2f(self.location.x, self.location.y)
        glVertex2f(self.location.x+self.partition.x*d, self.location.y+self.partition.y*d)
        glEnd()
        glPointSize( 6.0 )
        glBegin(GL_POINTS)
        glVertex2f(self.location.x, self.location.y)
        glEnd()
        
def midpointDisplacement(line, level, node):
    if level == 5:
        #drawLine(line)
        node.add(line)
    else:
        factor = 20 - level*2
        p = line.perpendicularVector()
        r = factor - random.random()*factor*2
        x = (line.x1 + line.x2)/2 + r*p.x
        y = (line.y1 + line.y2)/2 + r*p.y
        midpointDisplacement(Line(line.x1, line.y1, x, y), level + 1, node)
        midpointDisplacement(Line(x, y, line.x2, line.y2), level + 1, node)
        
def createWorld(node):
    line = Line(-window.width/2, 0, window.width/2, 0)
    midpointDisplacement(line, 0, node)

actor = Actor()
world = Node()
polygon = createBoundingBoxPolygon(Vector(-100, -100), Vector(100, 0))
polygon.magic()
polygon.magic()
polygon.magic()
polygon.magic()
polygon2 = createBoundingBoxPolygon(Vector(-20, -20), Vector(40, 0))
polygon2.magic()
polygon2.magic()
createWorld(world)    

@window.event
def on_draw():
    window.clear()
    glClear(GL_COLOR_BUFFER_BIT)
    glViewport(0, 0, window.width, window.height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(65, window.width / float(window.height), .1, 1000000)

    global actor
    glTranslatef(-actor.location.x, -actor.location.y, translateZ)
    line = Line(-window.width/2, 0, window.width/2, 0)
    #glEnable(GL_POLYGON_SMOOTH)
    #glBegin(GL_POLYGON)
    #drawLine(line)
    #midpointDisplacement(line, 0)
    #glEnd()
    
    glColor3f(1., 1., 1.)
    if actor.bbox.overlaps(polygon.bbox):
        glColor3f(1., 0.4, 0.4)
    actor.draw()
    #actor.bbox.draw()
    #world.draw()
    polygon.draw()
    label.draw()
    polygon.bbox.draw()
    polygon2.draw()
    polygon.cut(polygon2)
    
def drawLine(line):
    glVertex2f(line.x1, line.y1)
    glVertex2f(line.x2, line.y2)
      
#@window.event
#def on_mouse_motion(x, y, dx, dy):
    #global translateZ
    #translateZ = -y*10
    
pressedLeft = False
pressedRight = False
pressedJump = False


@window.event
def on_key_press(symbol, modifiers):
    global pressedLeft
    global pressedRight
    global pressedJump
    if symbol == key.LEFT:
        pressedLeft = True
    if symbol == key.RIGHT:
        pressedRight = True
    if symbol == key.SPACE:
        pressedJump = True
        
@window.event
def on_key_release(symbol, modifiers):
    global pressedLeft
    global pressedRight
    global pressedJump
    if symbol == key.LEFT:
        pressedLeft = False
    if symbol == key.RIGHT:
        pressedRight = False
    if symbol == key.SPACE:
        pressedJump = False
    
def update(dt):
    # move the actor
    a = 10
    a_jump = 5
    
    global actor
    global polygon
    global pressedLeft
    global pressedRight
    global pressedJump
    
    if pressedLeft:
        actor.accelerate(Vector(-a, 0), dt)
    elif pressedRight:
        actor.accelerate(Vector(a, 0), dt)
    else:
        actor.speed.x = 0
                
    if pressedJump and actor.canJump:
        actor.canJump = False
        actor.accelerate(Vector(0, a_jump), 1)
    
    actor.gravity(dt)
    
    if actor.location.y < -500:
        actor.speed.y = 0
        actor.location = Vector(0, 0)
     
    actor.applySpeed(dt)
    
    collides, newLocation = polygon.collision(actor)

    if collides:
        actor.canJump = True
        actor.speed.y = 0
    else:
        newLocation = actor.location
        
    actor.updateLocation(newLocation)
    
    
pyglet.clock.schedule_interval(update, 0.025)
    
pyglet.app.run()