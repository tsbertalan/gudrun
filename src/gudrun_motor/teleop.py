#!/bin/env python
from __future__ import print_function
from os import system, getpid
import time

import sys, termios, tty

DEBUG_DUMMY = False


def _set_servo(num, milliseconds):
    cmd = 'UscCmd --servo %d,%d' % (num, 4.0 * milliseconds)
    system(cmd + ' | grep -v runtime | grep -v target')


class Axis(object):

    def __init__(self, pin, zero_point=1496, low_point=992, high_point=2000, dummy=False):
        self._pin = pin
        self._ms_points = low_point, zero_point, high_point
        self._dummy = dummy
        self.fraction = 0

    @property
    def fraction(self):
        return self._fraction
    
    @fraction.setter
    def fraction(self, fraction):
        self._fraction = fraction
        l, m, h = self._ms_points
        if fraction >= 0:
            ms = fraction * (h - m) + m
        else:
            ms = fraction * (m - l) + m
        if not self._dummy:
            _set_servo(self._pin, ms)
        else:
            print('(Set servo %d to %s ms)' % (self._pin, ms))


class Car(object):

    def __init__(self, steering_pin=0, throttle_pin=1, dummy=DEBUG_DUMMY):
        self.MAX_THROTTLE_ABS = .3
        self._steering_axis = Axis(steering_pin, dummy=dummy)
        self._throttle_axis = Axis(throttle_pin, dummy=dummy)
        self._reversing = False
        self.stop()
        self.center()

    def stop(self):
        self.throttle = 0

    def center(self):
        self.steering = 0

    def switch_to_reverse(self):
        if not self._reversing:
            self._throttle_axis.fraction = -.1
            time.sleep(.05)
            self._throttle_axis.fraction = 0
            time.sleep(.05)
            self._reversing = True

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, fraction):

        fraction = min(max(fraction, -self.MAX_THROTTLE_ABS), self.MAX_THROTTLE_ABS)

        self._throttle = fraction
        if fraction < 0:
            self.switch_to_reverse()
        else:
            self._reversing = False
        self._throttle_axis.fraction = fraction

    @property
    def steering(self):
        return self._steering_axis.fraction

    @steering.setter
    def steering(self, fraction):
        self._steering_axis.fraction = fraction

    def __del__(self):
        print('Resetting steering and throttle on deletion of %s.' % self)
        self.stop()
        self.center()
    

def keyboard_teleop():
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
     
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    BUTTON_DELAY = 0.2
    THROTTLE_INCREMENT = .05
    STEERING_INCREMENT = .25

    car = Car()

    def inc(steering=True, direction=+1):
        if steering:
            car.steering += STEERING_INCREMENT * direction
            print('to', car.steering)
        else:
            car.throttle += THROTTLE_INCREMENT * direction
            print('to', car.throttle)

    keys = dict(
        A='UP',
        B='DOWN',
        C='RIGHT',
        D='LEFT',
    )
    keys['\t'] = 'TAB'
    keys[' '] = 'SPACE'
    def display_help():
        for key, (label, unused_action) in sorted(actions.items()):
            key = keys.get(key, key)
            print('"%s": %s' % (key, label))
        print('============================')

    actions = dict(
        q=('Quit.', None),
        w=('Accelerate', lambda : inc(0)),
        A=('Accelerate', lambda : inc(0)),
        s=('Decelerate', lambda : inc(0, -1)),
        B=('Decelerate', lambda : inc(0, -1)),
        d=('Steer right', lambda : inc(1, 1)),
        C=('Steer right', lambda : inc(1, 1)),
        a=('Steer left', lambda : inc(1, -1)),
        D=('Steer left', lambda : inc(1, -1)),
        c=('Center steering.', car.center),
        h=('Display key listing.', display_help),
        )    
    actions[' '] = ('Cut throttle.', car.stop)
    actions['\t'] = ('Cut throttle.', car.stop)

    display_help()

    while True:

        char = getch()

        if char in actions:
            label, action = actions[char]
            print(label, end='\n' if action in [None, display_help, car.center, car.stop] else ' ')
            if action is None:
                break
            action()
        # else:
        #     print('Got unknown char "%s".' % char)


import multiprocessing
def _monitor_ping(server, parent_pid, ping_timeout=1, kill_parent=False):
    sys.stdout = open('child.out', 'w', buffering=0)
    sys.stderr = open('child.err', 'w', buffering=0)
    print('Heartbeat PID is %s.' % getpid())
    system('mkdir -p tmp/')
    car = Car()
    while True:
        timeout = '' if ping_timeout is None else '-w %d -W %d' % (ping_timeout, ping_timeout)
        cmd = 'ping %s -c 1 %s >/dev/null 2>&1' % (server, timeout)
        # cmd = 'touch tmp/thing 2>/dev/null'
        if system(cmd):
            print('Heartbeat skipped; stopping vehicle.')
            # Send a keyboard intterupt so that motors can stop.
            # Send an escalating sequence:
            car.stop()
            car.center()
            if kill_parent:
                for signal, count in [(2, 3), (15, 1), (9, 1)]:
                    for _ in range(count):
                        cmd = 'kill -s %d %s 2>/dev/null' % (signal, parent_pid)
                        print('$ %s' % cmd)
                        system(cmd)
                        time.sleep(2.0)
                car.stop()
                car.center()
                break


class Heartbeat(object):

    def __init__(self, server='sindri.vpn.tomsb.net'):

        self.server = server
        pid = getpid()
        print('PID is %s.' % pid)
        self.p = multiprocessing.Process(target=_monitor_ping, args=(server, pid))
        self.p.start()

    def __del__(self):
        from os import kill
        from signal import SIGKILL
        kill(self.p.pid, SIGKILL)
        self.p.terminate()


def mouse_teleop():
    import pygame
    import pygame.camera
    from glob import glob

    sys.stdout = open('teleop.out', 'w', buffering=0)
    sys.stderr = open('teleop.err', 'w', buffering=0)

    pygame.init()
    RES_X = 320 / 2
    RES_Y = 240 / 2
    print(RES_X, RES_Y)
    display = pygame.display.set_mode((RES_X, RES_Y))
    pygame.camera.init()
    camera = pygame.camera.Camera(glob('/dev/video*')[-1], (RES_X, RES_Y))
    camera.start()
    screen = pygame.surface.Surface((RES_X, RES_Y), 0, display)

    pygame.event.set_grab(True)

    car = Car()

    last_speed_update_time = time.time()
    speed_update_period = .1
    input_heartbeat_period = 10
    heartbeat = Heartbeat('sindri.vpn.tomsb.net')

    with open('teleop.log', 'w') as logfile:

        def log(args, end='\n'):

            logfile.write(
                (
                    str(args) if not isinstance(args, tuple) 
                    else ' '.join([str(x) for x in args])
                ) + end
            )
            logfile.flush()
            if not isinstance(args, tuple):
                args = (args,)
            print(*args, end=end)

        class ToggleDrivable:
            def __init__(self):
                self.drivable = True
                self()

            def __call__(self, stop=None):
                if stop:
                    self.drivable = True
                if self.drivable:
                    log('Stopping car...', end=' ')
                    car.stop()
                    car.center()
                    log('Middle click to enable driving.')
                else:
                    pygame.mouse.set_pos(RES_X/2, RES_Y/2)
                    pygame.event.set_grab(True)
                    last_speed_update_time = time.time()
                    log('Move mouse to drive car.')
                    log('Middle click or right click to disable.')
                self.drivable = not self.drivable
        toggle_drivable = ToggleDrivable()

        exit = False
        log('q to exit')
  
        while True:
            screen = camera.get_image(screen)
            display.blit(screen, (0, 0))
            pygame.display.flip()

            t = time.time()

            if not pygame.mouse.get_focused() or not pygame.event.get_grab():
                if car.throttle != 0:
                    car.stop()
                if car.steering != 0:
                    car.center()
                if toggle_drivable.drivable:
                    log('Lost focus; stopping car.')
                    toggle_drivable(True)

            if t - last_speed_update_time > input_heartbeat_period:
                if toggle_drivable.drivable:
                    log('Inputs timed out; stopping car.')
                    toggle_drivable(True)
                else:
                    last_speed_update_time = t

            try:
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == ord(" "):
                            toggle_drivable(True)
                        elif event.key == pygame.K_ESCAPE:
                            toggle_drivable(True)
                            pygame.event.set_grab(False)

                        elif event.key == ord('q') or event.key == pygame.K_ESCAPE:
                            exit = True
                        else:
                            try:
                                log('Got key %s (%s).' % (event.key, chr(event.key)))
                            except ValueError:
                                log('Got non-ASCII keycode %s.' % event.key)

                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1: # left button
                            log('Locked mouse to window (Press esc to halt car and unlock).')
                            pygame.event.set_grab(True)
                        elif event.button == 2: # middle button
                            toggle_drivable()
                        elif event.button == 3: # right button
                            toggle_drivable(True)
                        elif event.button == 8: # web-back
                            car.MAX_THROTTLE_ABS *= .9
                            log('Max-abs throttle set to %s.' % car.MAX_THROTTLE_ABS)
                        elif event.button == 9: # web-fwd
                            car.MAX_THROTTLE_ABS *= 1.1
                            log('Max-abs throttle set to %s.' % car.MAX_THROTTLE_ABS)
                        # button 4 is scroll up; 5 is scroll down

                    elif event.type == pygame.MOUSEMOTION:
                        c, r = event.pos
                        if toggle_drivable.drivable:
                            t = time.time()
                            elapsed = t - last_speed_update_time 
                            if elapsed > speed_update_period:
                                last_speed_update_time = t
                                car.steering = float(c) / RES_X * 2 - 1
                                car.throttle = -(float(r) / RES_Y * 2 - 1)

                    elif event.type == pygame.QUIT:
                        exit = True

            except KeyboardInterrupt:
                print('Caught KeyboardInterrupt; stopping.')
                car.stop()
                car.center()
                exit = True
            
            if exit:
                break

if __name__ == '__main__':
    mouse_teleop()
