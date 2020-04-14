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

    #transformation matric between frames 0 and 1.
    #hardcoded beacuse it is the only transformation with an x translation
    P0_P1 = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

    scene2 = canvas(title='Collision Avoidance',
                    width=1800,
                    height=900,
                    center=zero,
                    background=color.gray(.2),
                    forward=vector(0.5,-2,0),
                    fov=70)

    floor = box(pos=vector(0, 0, 0), \
               size=vector(s3, thk, s3),
               color=color.gray(1))

    base = cylinder(axis=vector(0,1,0),
                    pos=vector(0, thk, 0),
                    color=color.red,
                    radius=3,
                    length=5)

    joint1 = sphere(pos=vector(0, base.pos.y  + base.length + 1,0), radius=2.9, color=color.white)
    arm1 = cylinder(axis=vector(0,1,0), pos=vector(0,joint1.pos.y, 0), color=color.red,
                    length=10 + joint1.radius)

    joint2 = sphere(pos=vector(0,arm1.pos.y + arm1.length,0),
                    radius=1.5,
                    color=color.white)

    arm2 = cylinder(axis=vector(0,1,0),
                    pos=vector(0,joint2.pos.y,0),
                    color=color.red,
                    length=5 + joint2.radius)

    joint3 = sphere(pos=vector(0,arm2.pos.y + arm2.length,0),
                    radius=1.5,
                    color=color.white)
    
    arm3 = cylinder(axis=vector(0,1,0),
                    pos=vector(0,joint3.pos.y,0),
                    color=color.red,
                    length=5 + joint3.radius)

    effector = sphere(pos=vector(0,arm3.pos.y + arm3.length,0),
                      radius=0.5,
                      color=color.white,
                      make_trail=True,
                      retain=400)

    degree = 0
    reachedEnd = False

    Ts = []
    P1_P2 = createAdjacentTx_Ty(30, base.length)
    P2_P3 = createAdjacentTx_Ty(30, arm1.length)
    P3_P4 = createAdjacentTx_Ty(30, arm2.length)
    P4_P5 = createAdjacentTx_Ty(30, arm3.length)

    Ts.append(P0_P1)
    Ts.append(P1_P2)
    Ts.append(P2_P3)
    Ts.append(P3_P4)
    Ts.append(P4_P5)

    P0_5 = end_effector(Ts)
    
    print(P0_5)


    for theta in range(0,90):
        rate(5)
        degree1 = 0.33
        degree2 = 0.33
        degree3 = 0.33

        theta1 = degree1
        theta2 = theta1 + degree2
        theta3 = theta2 + degree3


        radian1 = vp.radians(theta1)
        radian2 = vp.radians(theta2)
        radian3 = vp.radians(theta3)


        arm1.rotate(angle=radian1, axis=vector(1, 0, 0))
        joint2.pos = arm1.axis + arm1.pos

        arm2.pos = joint2.pos
        arm2.rotate(angle=radian2, axis=vector(1, 0, 0))

        joint3.pos = arm2.axis + arm2.pos
        arm3.pos = joint3.pos

        arm3.rotate(angle=radian3, axis=vector(1, 0, 0))
        effector.pos = arm3.axis + arm3.pos
        print(effector.pos)
    
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

    #pass in theta between two frames in degrees, and length of arm between the frames
def createAdjacentTx_Ty(theta, length):
    Tx_Ty = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta), np.cos(theta), length],
                      [0, 0, 1]])
    return Tx_Ty

def end_effector(Ts):
    T0_5 = Ts[0].dot(Ts[1]).dot(Ts[2]).dot(Ts[3]).dot(Ts[4])
    return T0_5

if __name__ == '__main__':
    main()


