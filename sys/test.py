#!/usr/bin/env python
import pygame
from pygame.locals import *
import threading
import sys
import math

Color_line = (0, 0, 0)
Color_line_middle = (180, 180, 180)
Color_line_path = (240, 0, 0)
Color_Red = (255, 0, 0)
Color_BLUE = (0, 0, 255)
Color_GREEN = (0, 255, 0)
Color_PURPLE = (255, 0, 255)
Color_CYAN = (0, 255, 255)


class PLOT:
    def __init__(self, static_obs,xlim,ylim):
        self._running = True
        self.display = None
        self._image_surf = None
        self.screenH = 800
        self.screenW = 600
        self.maxX = xlim
        self.maxY = ylim
        self.lines = 9
        self.originX = 0
        self.originY = 0
        self.moveX = 0
        self.moveY = 0
        self.future_x = []
        self.future_y = []
        self.future_heading =[]
        self.triangleX = (0, -5, 5)
        self.triangleY = (-10, 10, 10)
        self.static_obs = static_obs
        pygame.font.init()
        self.myfont = pygame.font.Font("r.ttf", 15)

    def on_init(self):
        pygame.init()
        self.display = pygame.display.set_mode(
            (self.screenH, self.screenW), HWSURFACE | RESIZABLE)
        self.w, self.h = pygame.display.get_surface().get_size()
        self.scalew = self.w*0.8
        self.scaleh = self.h*0.8
        self.startw = self.w * 0.1
        self.starth = self.h * 0.1
        self._running = True
        self.background = pygame.Surface(self.display.get_size())
        self.background.fill((240, 240, 240))
        self.background = self.background.convert()
        # self._image_surf = pygame.transform.scale(pygame.image.load(
        #     "boat.png"), (int(self.scalew/20), int(self.scaleh/20))).convert_alpha()
        # self.image_obstacle = pygame.transform.scale(pygame.image.load(
        #     "obstacle.png"), (int(self.scalew/20), int(self.scaleh/20))).convert_alpha()
        # self.boatw = int(self.scalew/20)/2.0
        # self.boath = int(self.scaleh/20)
        self.curr_x = 0
        self.curr_y = 0
        self.start_heading = 0
        self.nobs = 0
        self.xobs = []
        self.yobs = []
        self.hobs = []

    def on_event(self, event):
        if event.type == QUIT:
            sys.exit(1)
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                self.originX -= self.scalew/(self.lines+1)
                self.moveX += self.maxX / (self.lines+1)
            elif event.key == pygame.K_RIGHT:
                self.originX += self.scalew/(self.lines+1)
                self.moveX -= self.maxX / (self.lines+1)
            elif event.key == pygame.K_DOWN:
                self.originY += self.scaleh/(self.lines+1)
                self.moveY += self.maxY / (self.lines+1)
            elif event.key == pygame.K_UP:
                self.originY -= self.scaleh/(self.lines+1)
                self.moveY -= self.maxY / (self.lines+1)
            elif event.key == pygame.K_MINUS:
                self.moveY /= self.maxY / (self.lines+1)
                self.moveX /= self.maxX / (self.lines+1)
                self.maxX += 100
                self.maxY += 100
                self.moveY *= self.maxY / (self.lines+1)
                self.moveX *= self.maxX / (self.lines+1)
            elif event.key == pygame.K_EQUALS:
                if self.maxX >= 200:
                    self.moveY /= self.maxY / (self.lines+1)
                    self.moveX /= self.maxX / (self.lines+1)
                    self.maxX -= 100
                    self.maxY -= 100
                    self.moveY *= self.maxY / (self.lines+1)
                    self.moveX *= self.maxX / (self.lines+1)

    def on_loop(self):
        pass

    def on_render(self):
        self.display.blit(self.background, (0, 0))
        self.draw_line()
        self.draw_path()
        self.draw_obs()
        self.draw_text()
        self.draw_future()
        self.draw_current()
        pygame.display.flip()

    def on_cleanup(self):
        pygame.quit()

    def on_execute(self, curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs):
        if self.on_init() == False:
            self._running = False
        # while self._running:
        #     self.update()
        # self.stop()
        self.pathList = [(curr_x, curr_y)]
        self.updateInformation(
            curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs,[],[],[])
        self.update()

    def draw_line(self):
        pygame.draw.line(self.display, Color_line, (self.startw,
                                                    self.starth), (self.startw+self.scalew, self.starth))
        pygame.draw.line(self.display, Color_line, (self.startw,
                                                    self.starth), (self.startw, self.starth+self.scaleh))
        pygame.draw.line(self.display, Color_line, (self.startw, self.starth +
                                                    self.scaleh), (self.startw+self.scalew, self.starth+self.scaleh))
        pygame.draw.line(self.display, Color_line, (self.startw+self.scalew,
                                                    self.starth), (self.startw+self.scalew, self.starth+self.scaleh))
        scaleh = self.scaleh/(self.lines+1)
        scalew = self.scalew/(self.lines+1)
        for i in xrange(1, self.lines+1):
            pygame.draw.line(self.display, Color_line_middle, self.scale_xy(
                0, i * scaleh), self.scale_xy(self.scalew, i * scaleh))
            pygame.draw.line(self.display, Color_line_middle, self.scale_xy(
                i * scalew, 0), self.scale_xy(i*scalew, self.scaleh))
            # self.draw_text(self.scalew,i * scaleh, i * self.maxX/(self.lines + 1) )

    def updateInformation(self, curr_x, curr_y, start_heading, nobs, xobs, yobs, hobs,future_x,future_y,future_heading):
        if self.pathList[-1] != (curr_x, curr_y):
            self.pathList.append((curr_x, curr_y))
        self.curr_x = curr_x
        self.curr_y = curr_y
        self.start_heading = start_heading
        self.nobs = nobs
        self.xobs = xobs
        self.yobs = yobs
        self.hobs = hobs
        self.future_heading = future_heading
        self.future_x = future_x
        self.future_y = future_y

    def draw_future(self):
        size = len(self.future_heading) 
        if size > 0 :
            self.draw_vehicle(self.future_heading[0],Color_CYAN,*self.scale_item(self.future_y[0], self.future_x[0]))
            if size > 1:
                decay = (255 - 100) / (size - 1)
                for i in xrange(1,size):
                    c = (255 - decay * i, 0, 255 - decay * i)
                    self.draw_vehicle(self.future_heading[i],c,*self.scale_item(self.future_y[i], self.future_x[i]))

    def draw_current(self):
        self.draw_vehicle(self.start_heading,Color_BLUE,*self.scale_item(self.curr_x, self.curr_y))
        # self.display.blit(pygame.transform.rotate(self._image_surf, -1*math.degrees(self.start_heading)),
        #                   (self.scale_item(self.curr_x, self.curr_y)))

    def draw_path(self):
        for index in xrange(1, len(self.pathList)):
            pygame.draw.line(self.display, Color_line_path, self.scale_path(
                *self.pathList[index-1]), self.scale_path(*self.pathList[index]))

    def draw_obs(self):
        for index in range(self.nobs):
            self.draw_vehicle(
                self.hobs[index], Color_Red, *self.scale_item(self.xobs[index], self.yobs[index]))

        for obs in self.static_obs:
            self.draw_static_obs(*obs)

    def draw_static_obs(self,x,y):
        pygame.draw.polygon(self.display, (0,0,0), (self.scale_item(x,y),self.scale_item(x+1,y),self.scale_item(x+1,y+1),self.scale_item(x,y+1)))

    def draw_vehicle(self, angle, color, x, y):
        tX = []
        tY = []
        c = math.cos(angle)
        s = math.sin(angle)
        for i in xrange(3):
            tX.append(self.triangleX[i]*c - self.triangleY[i]*s + x)
            tY.append(self.triangleX[i]*s + self.triangleY[i]*c + y)
        pygame.draw.polygon(self.display, color, ((
            tX[0], tY[0]), (tX[1], tY[1]), (tX[2], tY[2])))

    def draw_text(self):
        scaleh = self.scaleh/(self.lines+1)
        scalew = self.scalew/(self.lines+1)
        for i in xrange(0, self.lines+2):
            text = str(int(self.moveY+(self.lines - i + 1)
                           * self.maxX/(self.lines + 1)))
            textsurface = self.myfont.render(text, True, (0, 0, 0))
            x, y = self.scale_xy(0, (i) * scaleh)
            x -= textsurface.get_width() + 10
            y -= textsurface.get_height()/2
            self.display.blit(textsurface, (x, y))
            text = str(int(self.moveX+(self.lines - i + 1)
                           * self.maxX/(self.lines + 1)))
            textsurface = self.myfont.render(text, True, (0, 0, 0))
            x, y = self.scale_xy((self.lines - i + 1)*scalew, self.scaleh)
            x -= textsurface.get_width() / 2
            y += 10
            self.display.blit(textsurface, (x, y))
        # textsurface = self.myfont.render(str(int(t)), True, (0, 0, 0))
        # self.display.blit(textsurface,(x,y))

    def update(self):
        for event in pygame.event.get():
            self.on_event(event)
        self.on_loop()
        self.on_render()

    def stop(self):
        self.on_cleanup()

    def scale_xy(self, x, y):
        return self.startw + x, self.starth + y

    def scale_item(self, x, y):
        return self.originX + self.startw + self.scalew*(x/float(self.maxX)), self.originY + self.starth + self.scaleh - self.scaleh*(y/float(self.maxY))

    def scale_path(self, x, y):
        return self.originX + self.startw + self.scalew*(x/float(self.maxX)), self.originY + self.starth + self.scaleh - self.scaleh*(y/float(self.maxY))


# if __name__ == "__main__" :
#     theApp = App()
#     theApp.on_execute()