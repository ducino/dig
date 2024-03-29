import pyglet
import random
import time
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

label = pyglet.text.Label('esdf to move, space to jump',
    font_name='Times New Roman',
    font_size=8,
    x=0, y=50,
    anchor_x='center', anchor_y='center')
    
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

debug = 0
actor = Actor()
world = QuadNode()

# polygon = Polygon()
# polygon.addVertex(Vector(-100., -100.))
# polygon.addVertex(Vector(100., -100.+random.randrange(-10., 10.)))
# polygon.addVertex(Vector(50., 0.))
# world.add(polygon)

polygon = Polygon.createBoundingBoxPolygon(Vector(-100, -100), Vector(100, 0))
polygon.midpointDisplacement(10)
polygon.midpointDisplacement(10)
polygon.midpointDisplacement(8)
polygon.midpointDisplacement(6)
polygon.triangulate()
world.add(polygon)

polygon2 = Polygon.createBoundingBoxPolygon(Vector(-20, -20), Vector(40, 0))
polygon2.midpointDisplacement(8)
polygon2.midpointDisplacement(6)  
polygon2.triangulate()

for i in range(-5, 5):
    poly = Polygon.createBoundingBoxPolygon(Vector(-200*i, -200), Vector(-200*i+100+random.randrange(50, 150), -100))
    poly.midpointDisplacement(10)
    poly.midpointDisplacement(10)
    poly.midpointDisplacement(8)
    poly.midpointDisplacement(6)
    poly.triangulate()
    world.add(poly)



@window.event
def on_draw():
    window.clear()
    
    glClear(GL_COLOR_BUFFER_BIT)
    glViewport(0, 0, window.width, window.height)
    
    label.draw()

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
    # polygon.bbox.draw()
    # polygon2.draw()
    
    # Quad Tree TEST CODE
    world.add(actor)
    world.drawCollision(actor)
    world.remove(actor)
    
    world.draw(actor.view(window))
    
    # TEST CODE
    # random.seed(int(time.clock()))
    # p1 = Polygon()
    # p1.addVertex(Vector(-10+random.randrange(-5, 5), 10+random.randrange(-5, 5)))
    # p1.addVertex(Vector(random.randrange(-5, 5), 20+random.randrange(-5, 5)))
    # p1.addVertex(Vector(10+random.randrange(-5, 5), 10+random.randrange(-5, 5)))
    # p1.draw()
    
    # p2 = Polygon()
    # p2.addVertex(Vector(-8+random.randrange(-5, 5), 8+random.randrange(-5, 5)))
    # p2.addVertex(Vector(random.randrange(-5, 5), 16+random.randrange(-5, 5)))
    # p2.addVertex(Vector(8+random.randrange(-5, 5), 8+random.randrange(-5, 5)))
    # p2.draw()
    
    # TEST CODE: collision vector
    # collides, offset = world.collision(actor.polygon)
    
    # glLineWidth(5)
    # glColor3f(.2, .2, 1.)
    # factor=0.2
    # glBegin(GL_LINES)
    # glVertex2f(actor.location.x, actor.location.y)
    # glVertex2f(actor.location.x+actor.speed.x*factor, actor.location.y+actor.speed.y*factor)
    # glEnd()
    # glLineWidth(1)
    
    
    # collides, vector = p1.collision(p2)
    # print collides
    
    # collides, offset = world.collision(actor.polygon)
    # ##############
    
def drawLine(line):
    glVertex2f(line.x1, line.y1)
    glVertex2f(line.x2, line.y2)
      
#@window.event
#def on_mouse_motion(x, y, dx, dy):
    #global translateZ
    #translateZ = -y*10
    
@window.event
def on_key_press(symbol, modifiers):
    global actor
    
    if symbol == key.LEFT:
        actor.controls.pressedLeft = True
    if symbol == key.RIGHT:
        actor.controls.pressedRight = True
    if symbol == key.UP:
        actor.controls.pressedUp = True
    if symbol == key.DOWN:
        actor.controls.pressedDown = True
    if symbol == key.SPACE:
        actor.controls.pressedJump = True
        
    global polygon2
    global debug

    if symbol == key.A:
        polygon2 = Polygon.createBoundingBoxPolygon(Vector(-20, -20), Vector(40, 0))
        polygon2.midpointDisplacement(8)
        polygon2.midpointDisplacement(6)
        polygon2.triangulate()
    if symbol == key.Z:
        debug = (debug + 1)%4
        
@window.event
def on_key_release(symbol, modifiers):
    global actor
    if symbol == key.LEFT:
        actor.controls.pressedLeft = False
    if symbol == key.RIGHT:
        actor.controls.pressedRight = False
    if symbol == key.UP:
        actor.controls.pressedUp = False
    if symbol == key.DOWN:
        actor.controls.pressedDown = False
    if symbol == key.SPACE:
        actor.controls.pressedJump = False
    
def update(dt):
    # print dt
    
    global actor    
    actor.update(dt)
    
    collides, offset = world.collision(actor.polygon)
    
    newLocation = actor.location
    if collides:
        dot = actor.speed.dot(offset)
        if dot > 0:
            offset = offset.multiply(-1)
        newLocation = newLocation.add(offset)
        actor.canJump = True
        actor.speed.y = 0
              
    actor.updateLocation(newLocation)
    
    
pyglet.clock.schedule_interval(update, 0.025)
    
pyglet.app.run()