import pyglet
import random
from pyglet.gl import *
from pyglet.window import key
from math import sqrt

from Polygon import Polygon
from Vector import Vector
from Actor import Actor
from BoundingBox import BoundingBox
from QuadTree import QuadNode

# TODO intersection of polygons
# TODO triangulation
# TODO merging of polygons?
# TODO solve self intersecting polygons
# TODO controls proper acceleration
# TODO proper clipping

window = pyglet.window.Window(width=1280, height=1024)
#window.push_handlers(pyglet.window.event.WindowEventLogger())

translateZ = -100

label = pyglet.text.Label('Midpoint displacement test',
    font_name='Times New Roman',
    font_size=36,
    x=window.width//2, y=window.height//2,
    anchor_x='center', anchor_y='center')

def createBoundingBoxPolygon(pMin, pMax):
    p = Polygon()
    p.addVertex(Vector(pMin.x, pMax.y))
    p.addVertex(Vector(pMin.x, pMin.y))
    p.addVertex(Vector(pMax.x, pMin.y))
    p.addVertex(Vector(pMax.x, pMax.y))
    return p
    
class LineSegment:
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

actor = Actor()
world = QuadNode()

polygon = createBoundingBoxPolygon(Vector(-100, -100), Vector(100, 0))
polygon.midpointDisplacement(10)
polygon.midpointDisplacement(10)
polygon.midpointDisplacement(8)
polygon.midpointDisplacement(6)
polygon.triangulate()
world.add(polygon)

polygon2 = createBoundingBoxPolygon(Vector(-20, -20), Vector(40, 0))
polygon2.midpointDisplacement(8)
polygon2.midpointDisplacement(6)  
polygon2.triangulate()

lastpoly = None
for i in range(-5, 5):
    poly = createBoundingBoxPolygon(Vector(-200*i, -200), Vector(-200*i+100+random.randrange(50, 150), -100))
    poly.midpointDisplacement(10)
    poly.midpointDisplacement(10)
    poly.midpointDisplacement(8)
    poly.midpointDisplacement(6)
    poly.triangulate()
    lastpoly = poly
    world.add(poly)



@window.event
def on_draw():
    window.clear()
    glClear(GL_COLOR_BUFFER_BIT)
    glViewport(0, 0, window.width, window.height)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(65, window.width / float(window.height), .1, 10000)

    global actor
    global world
    glTranslatef(-actor.location.x, -actor.location.y, translateZ)
    line = LineSegment(-window.width/2, 0, window.width/2, 0)
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
    # label.draw()
    # polygon.bbox.draw()
    # polygon2.draw()
    BoundingBox(-10, -10, 10, 10).draw()
    world.add(actor)
    world.drawCollision(actor)
    world.remove(actor)
    world.draw(actor.view(window))
    
    lastpoly.bbox.draw()
    # global lastpoly
    # world.drawCollision(lastpoly)
    
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
    global polygon2
    if symbol == key.LEFT:
        pressedLeft = True
    if symbol == key.RIGHT:
        pressedRight = True
    if symbol == key.SPACE:
        pressedJump = True
    if symbol == key.A:
        polygon2 = createBoundingBoxPolygon(Vector(-20, -20), Vector(40, 0))
        polygon2.midpointDisplacement(8)
        polygon2.midpointDisplacement(6)
        polygon2.triangulate()
    if symbol == key.Z:
        polygon2.debug = (polygon2.debug + 1)%4
        
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
    a = 200
    aStop = 500
    a_jump = 100
    
    print dt
    
    global actor
    global polygon
    global pressedLeft
    global pressedRight
    global pressedJump
    
    actor.update(dt, pressedRight, pressedLeft, pressedJump)
    
    collides, offset = world.collision(actor)
    newLocation = actor.location
    if collides:
        newLocation = newLocation.add(offset)
        # newLocation = offset
        actor.canJump = True
        actor.speed.y = 0
        
    actor.updateLocation(newLocation)
    
    
pyglet.clock.schedule_interval(update, 0.025)
    
pyglet.app.run()