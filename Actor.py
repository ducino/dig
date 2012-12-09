from pyglet.gl import *
from math import copysign
from Vector import Vector
from BoundingBox import BoundingBox
from Polygon import Polygon
from Controls import Controls

class Actor:
    def __init__(self):
        self.speed = Vector(0, 0)
        self.width = 1.
        self.height = 15.
        self.canJump = True
        self.location = Vector(0, 11)
        self.updateLocation(Vector(0, 10))
        
        self.maxSpeedX = 90
        self.maxSpeedY = 600
        self.accX = 100
        self.accStop = 500
        self.accJump = 50
        
        self.controls = Controls()
        
    def update(self, dt):
        if self.controls.pressedLeft:
            if self.speed.x > 0:
                self.accelerate(Vector(-self.accStop, 0), dt)
            else:
                self.accelerate(Vector(-self.accX, 0), dt)
        elif self.controls.pressedRight:
            if self.speed.x < 0:
                self.accelerate(Vector(self.accStop, 0), dt)
            else:
                self.accelerate(Vector(self.accX , 0), dt)
        elif abs(self.speed.x) < 10:
            self.speed.x = 0
        elif self.speed.x < 0:
            self.accelerate(Vector(self.accStop, 0), dt)
        else: #self.speed.x > 0
            self.accelerate(Vector(-self.accStop, 0), dt)
                    
        if self.controls.pressedJump and self.canJump:
            self.canJump = False
            self.accelerate(Vector(0, self.accJump), 1)
        
        self.gravity(dt)
        
        if self.location.y < -700:
            self.speed.y = 0.
            self.location = Vector(0., 0.)
         
        self.applySpeed(dt)
        
        # TEST CODE
        # if self.controls.pressedUp:
            # self.updateLocation(self.location.add(Vector(0, 1)))
        # if self.controls.pressedDown:
            # self.updateLocation(self.location.add(Vector(0, -1)))
        
        self.updateLocation(self.location)
    
    def draw(self):
        glBegin(GL_POLYGON)
        glVertex2f(self.location.x-self.width/2, self.location.y-self.height/2)
        glVertex2f(self.location.x+self.width/2, self.location.y-self.height/2)
        glVertex2f(self.location.x+self.width/2, self.location.y+self.height/2)
        glVertex2f(self.location.x-self.width/2, self.location.y+self.height/2)
        glEnd()
        
    def accelerate(self, vector, dt):
        self.speed.x = self.speed.x + vector.x*dt
        if abs(self.speed.x) > self.maxSpeedX:
            self.speed.x = copysign(self.maxSpeedX, self.speed.x)
            
        self.speed.y = self.speed.y + vector.y*dt
        if abs(self.speed.y) > self.maxSpeedY:
            self.speed.y = copysign(self.maxSpeedY, self.speed.y)
        
    def gravity(self, dt):
        self.accelerate(Vector(0, -120.), dt)
        
    def applySpeed(self, dt):
        self.location.x = self.location.x + self.speed.x*dt
        self.location.y = self.location.y + self.speed.y*dt
        
    def updateLocation(self, newLocation):
        self.previousLocation = self.location
        self.location = newLocation
        self.bbox = BoundingBox(self.location.x-self.width/2, self.location.y-self.height/2,
                                self.location.x+self.width/2, self.location.y+self.height/2)
        self.polygon = Polygon.createBoundingBoxPolygon(Vector(self.location.x-self.width/2, self.location.y-self.height/2),
                                                Vector(self.location.x+self.width/2, self.location.y+self.height/2))
        self.polygon.convex = True        
    
    #
    # Get the view bounding box                         
    #
    def view(self, window):
        fov = 0.14 # Hack: Roughly the factor needed to account for the FOV
        w = window.width*fov
        h = window.height*fov
        bbox = BoundingBox(self.location.x-w/2, self.location.y-h/2, self.location.x+w/2, self.location.y+h/2)
        # bbox.draw()
        return bbox