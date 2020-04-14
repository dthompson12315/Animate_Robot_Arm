import vpython as vp
from vpython import canvas,vector, color, box, sphere, rate, norm, cylinder
import random
import math
import time
import numpy as np


def main():
    side = 50.0
    thk = 0.3

    rad = 5
    leng = rad*5
    ax = vector(0,1,0)
    s3 = 2 * side + thk

    start = vector(-side, 1, -side)
    end = vector(side, 1, side)
    zero = vector(0,20, 0)

    scene2 = canvas(title='Collision Avoidance', width=1800, height=900, center=zero,
                    background=color.gray(.2), forward=vector(0.5,-2,0), fov=70)

    floor = box(pos=vector(0, 0, 0), size=vector(s3, thk, s3), color=color.gray(1))
    base = cylinder(axis=vector(0,1,0), pos=vector(0, thk, 0), color=color.red, radius=3,
                    length=5)
    joint1 = sphere(pos=vector(0,base.length,0), radius=2.9, color=color.white)
    arm1 = cylinder(axis=vector(0,1,0), pos=vector(0,joint1.pos.y + joint1.radius - .4,0), color=color.red, length=10)
    joint2 = sphere(pos=vector(0,arm1.pos.y + arm1.length,0), radius=1.5, color=color.white)
    arm2 = cylinder(axis=vector(0,1,0), pos=vector(0,joint2.pos.y,0), color=color.red, length=5)
    # arm2.rotate(angle=vp.radians(1), axis=vector(1,0,0))
    dt = 0.001

    degree = 0
    reachedEnd = False

    for theta in range(0,91):
        radian = vp.radians(1)
        print(radian)
        rate(10)
        arm2.rotate(angle=radian, axis=vector(1, 0, 0))
    #
    # while not reachedEnd:
    #     rate(1)
    #     #
    #     #
    #     # radian = vp.radians(degree)
    #     # print(radian)
    #     #
    #     # arm2.rotate(angle=radian, axis=vector(1,0,0))
    #     #
    #     # degree = degree + 1
    #     # time.sleep(1)
if __name__ == '__main__':
    main()
