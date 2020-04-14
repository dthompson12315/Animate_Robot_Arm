import vpython as vp
from vpython import canvas,vector, color, box, sphere, rate, norm, cylinder
import random
import math
import time


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
    arm2 = cylinder(axis=vector(0,1,0), pos=vector(0,joint2.pos.y,0), color=color.red, length=5 + joint2.radius)

    joint3 = sphere(pos=vector(0,arm2.pos.y + arm2.length,0), radius=1.5, color=color.white)
    arm3 = cylinder(axis=vector(0,1,0), pos=vector(0,joint3.pos.y,0), color=color.red, length=5 + joint3.radius)

    effector = sphere(pos=vector(0,arm3.pos.y + arm3.length,0), radius=0.5,
                      color=color.white, make_trail=True, retain=400)


    dt = 0.001

    degree = 0
    reachedEnd = False

    time.sleep(5)

    for theta in range(0,91):
        rate(10)

        radian1 = vp.radians(1)
        radian2 = vp.radians(1.5)
        # print(radian)


        arm2.rotate(angle=radian1, axis=vector(1, 0, 0))
        joint3.pos = arm2.axis + arm2.pos

        arm3.pos = joint3.pos

        arm3.rotate(angle=radian2, axis=vector(1, 0, 0))
        effector.pos = arm3.axis + arm3.pos
        print(arm3.axis)
        # effector.pos

    #
    # while not reachedEnd:
    #     rate(1)
        #
        #
        # radian = vp.radians(degree)
        # print(radian)
        #
        # arm2.rotate(angle=radian, axis=vector(1,0,0))
        #
        # degree = degree + 1
        # time.sleep(1)
if __name__ == '__main__':
    main()
