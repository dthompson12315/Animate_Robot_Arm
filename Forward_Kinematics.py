"""
    Brandon Sawyer and Dylan Thompson
    CSE 4280
    Forward Kinematics
    04/30/2020
"""

import vpython as vp
from vpython import canvas, vector, color, box, sphere, rate, norm, cylinder
import time
import numpy as np
from scipy.spatial.transform import Rotation as R


# pass in theta between two frames in degrees, and length of arm between the frames
def createAdjacentTx_Ty(theta, length):
    # used roatation matrix for clockwise rotations
    Tx_Ty = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                      [np.sin(theta), np.cos(theta), 0, length],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
    return Tx_Ty


def end_effector(P0_P1, P1_P2, P2_P3, P3_P4, P4_P5):
    T0_2 = np.matmul(P0_P1, P1_P2)
    T0_3 = np.matmul(T0_2, P2_P3)
    T0_4 = np.matmul(T0_3, P3_P4)
    T0_5 = np.matmul(T0_4, P4_P5)

    return P0_P1, T0_2, T0_3, T0_4, T0_5


def posCal(start, offset):
    return start + vector(offset[0],offset[1],offset[2])


def main():
    side = 50.0
    thk = 0.3

    s3 = 2 * side + thk
    zero = vector(0, 20, 0)

    # transformation matric between frames 0 and 1.
    # hardcoded beacuse it is the only transformation with an x translation
    P0_P1 = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0.3],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    scene2 = canvas(title='Collision Avoidance',
                    width=1800,
                    height=900,
                    center=zero,
                    background=color.gray(.2),
                    forward=vector(0, -2, -5),
                    fov=70)

    floor = box(pos=vector(0, 0, 0),
                size=vector(s3, thk, s3),
                color=color.gray(1))

    base = cylinder(axis=vector(0, 1, 0),
                    pos=vector(0, thk, 0),
                    color=color.red,
                    radius=3,
                    length=5)

    joint1 = sphere(pos=posCal(base.pos,[0,base.length + 1,0]),
                    radius=2.9,
                    color=color.white)
    arm1 = cylinder(axis=vector(0, 1, 0),
                    pos=joint1.pos,
                    color=color.red,
                    length=10 + joint1.radius)

    joint2 = sphere(pos=posCal(arm1.pos,[0,arm1.length,0]),
                    radius=1.5,
                    color=color.white)

    arm2 = cylinder(axis=vector(0, 1, 0),
                    pos=joint2.pos,
                    color=color.red,
                    length=5 + joint2.radius)

    joint3 = sphere(pos=posCal(arm2.pos,[0,arm2.length,0]),
                    radius=1.5,
                    color=color.white)

    arm3 = cylinder(axis=vector(0, 1, 0),
                    pos=joint3.pos,
                    color=color.red,
                    length=5 + joint3.radius)

    effector = sphere(pos=posCal(arm3.pos,[0,arm3.length,0]),
                      radius=0.5,
                      color=color.white # ,
                      # make_trail=True,
                      # retain=400
                      )


    P1_P2 = createAdjacentTx_Ty(vp.radians(-45), base.length + 1)
    P2_P3 = createAdjacentTx_Ty(vp.radians(90), arm1.length)
    P3_P4 = createAdjacentTx_Ty(vp.radians(-45), arm2.length)
    P4_P5 = createAdjacentTx_Ty(vp.radians(0), arm3.length)

    P0_1, P0_2, P0_3, P0_4, P0_5 = end_effector(P0_P1, P1_P2, P2_P3, P3_P4, P4_P5)

    r1 = R.from_matrix(P0_1[:3,:3])
    r2 = R.from_matrix(P0_2[:3,:3])
    r3 = R.from_matrix(P0_3[:3,:3])
    r4 = R.from_matrix(P0_4[:3,:3])
    r5 = R.from_matrix(P0_5[:3,:3])

    # print(P1_P2[:3,:3])
    # print(r1.as_euler('zyx', degrees=True))
    print(r2.as_euler('zyx', degrees=True))
    print(r3.as_euler('zyx', degrees=True))
    print(r4.as_euler('zyx', degrees=True))
    # print(r5.as_euler('zyx', degrees=True))

    bR = r1.as_euler('zyx', degrees=True)
    a1R = r2.as_euler('zyx', degrees=True)
    a2R = r3.as_euler('zyx', degrees=True)
    a3R = r4.as_euler('zyx', degrees=True)
    eR = r5.as_euler('zyx', degrees=True)

    time.sleep(5)
    dt = 0.01
    for theta in range(0, 100):
        rate(20)

        a1RX = vp.radians(a1R[0]*dt)
        a2RX = vp.radians(a2R[0]*dt)
        a3RX = vp.radians(a3R[0]*dt)

        arm1.rotate(angle=a1RX, axis=vector(0, 0, 1))

        joint2.pos = arm1.pos + arm1.axis
        arm2.rotate(angle=a2RX, axis=vector(0, 0, 1))
        arm2.pos = joint2.pos


        joint3.pos = arm2.pos + arm2.axis
        arm3.rotate(angle=a3RX, axis=vector(0, 0, 1))
        arm3.pos =  joint3.pos

        effector.pos = arm3.pos + arm3.axis


if __name__ == '__main__':
    main()

