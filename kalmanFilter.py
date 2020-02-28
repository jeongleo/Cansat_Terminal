import numpy as np
import numpy.linalg as lin
from math import *
import eulerAngle as eu


class EulerKalman:
    def __init__(self):
        self.H = np.eye(4)
        self.Q = np.eye(4) * 0.0001
        self.R = np.eye(4) * 10

        self.x = np.array([[1, 0, 0, 0]]).T  # x는 1차원 벡터이지만 파이썬에서 전치를 구현하기 위해 2차원 배열로 선언했다.
        self.P = np.eye(4) * 1

    def eulerKalmanFilter(self, A, z):
        xp = np.dot(A, self.x)
        Pp = np.dot(np.dot(A, self.P), A.T) + self.Q

        K = np.dot(np.dot(Pp, self.H.T), lin.inv(np.dot(np.dot(self.H, Pp), self.H.T) + self.R))

        self.x = xp + np.dot(K, z - np.dot(self.H, xp))
        self.P = Pp - np.dot(np.dot(K, self.H), Pp)

        x = self.x.reshape(4)  # 4칸짜리 1차원 배열로 변경한다.
        phi = atan2(2*(x[2]*x[3] + x[0]*x[2]), 1 - 2*(x[1]**2 + x[2]**2))
        theta = - asin(2*(x[1]*x[3] - x[0]*x[2]))
        psi = atan2(2*(x[1]*x[2] + x[0]*x[3]), 1 - 2*(x[2]**2 + x[3]**2))

        return phi, theta, psi

    def calcFilter(self, p, q, r, ax, ay, az, dt):
        A = [[0, -p, -q, -r],
             [p, 0, r, -q],
             [q, -r, 0, p],
             [r, q, -p, 0]]
        A = np.eye(4) + dt * 0.5 * np.array(A)

        phi, theta = eu.Euler.eulerAccel(ax, ay, az)
        z = self.euler2quaternion(phi, theta, 0)

        return self.eulerKalmanFilter(A, z)

    @staticmethod
    def euler2quaternion(phi, theta, psi):
        sinPhi = sin(phi/2)
        cosPhi = cos(phi/2)
        sinTheta = sin(theta/2)
        cosTheta = cos(theta/2)
        sinPsi = sin(psi/2)
        cosPsi = cos(psi/2)

        z = [[cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi],
             [sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi],
             [cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi],
             [cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi]]
        return np.array(z)


class EulerEKF:
    def __init__(self):
        self.H = np.array([[1, 0, 0],
                           [0, 1, 0]])
        self.Q = np.eye(3) * 0.0001
        self.R = np.eye(2) * 10

        self.x = np.array([[0, 0, 0]]).T
        self.P = np.eye(3) * 10

    def calcFilter(self, p, q, r, ax, ay, az, dt):
        phi_a, theta_a = eu.Euler.eulerAccel(ax, ay, az)

        z = np.array([[phi_a, theta_a]]).T
        rates = np.array([[p, q, r]]).T

        return self.eulerEKFFilter(z, rates, dt)

    def eulerEKFFilter(self, z, rates, dt):
        A = self.Ajacob(self.x, rates, dt)  # 야코비안 계산

        xp = self.fx(self.x, rates, dt)  # 상태 변수 예츨값 계산 함수 호출
        Pp = np.dot(np.dot(A, self.P), A.T) + self.Q

        K = np.dot(np.dot(Pp, self.H.T), lin.inv(np.dot(np.dot(self.H, Pp), self.H.T) + self.R))

        self.x = xp + np.dot(K, z - np.dot(self.H, xp))
        self.P = Pp - np.dot(np.dot(K, self.H), Pp)

        phi = self.x[0]
        theta = self.x[1]
        psi = self.x[2]

        return phi, theta, psi

    def fx(self, xhat, rates, dt):
        phi = xhat[0]
        theta = xhat[1]

        p = rates[0]
        q = rates[1]
        r = rates[2]

        xdot = np.zeros((3, 1))
        xdot[0] = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
        xdot[1] = q*cos(phi) - r*sin(phi)
        xdot[2] = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)

        return xhat + xdot*dt

    def Ajacob(self, xhat, rates, dt):
        A = np.zeros((3, 3))
        phi = xhat[0]
        theta = xhat[1]

        p = rates[0]
        q = rates[1]
        r = rates[2]

        A[0, 0] = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta)
        A[0, 1] = q*sin(phi)/(cos(theta)**2) + r*cos(phi)/(cos(theta)**2)
        A[0, 2] = 0

        A[1, 0] = -q*sin(phi) - r*cos(phi)
        A[1, 1] = 0
        A[1, 2] = 0

        A[2, 0] = q*cos(phi)/cos(theta) - r*sin(phi)/cos(theta)
        A[2, 1] = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)
        A[2, 2] = 0

        A = np.eye(3) + A*dt
        return A


class GPSKalman:
    def __init__(self):
        self.H = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0]])
        self.Q = np.eye(4) * 1.0
        self.R = np.eye(2) * 50
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 100

    def trackKalman(self, xm, ym, dt):
        A = np.array([[1, dt, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, dt],
                      [0, 0, 0, 1]])

        xp = np.dot(A, self.x)
        Pp = np.dot(np.dot(A, self.P), A.T) + self.Q

        K = np.dot(np.dot(Pp, self.H.T), lin.inv(np.dot(np.dot(self.H, Pp), self.H.T) + self.R))

        z = np.array([[xm, ym]]).T
        self.x = xp + np.dot(K, z - np.dot(self.H, xp))
        self.P = Pp - np.dot(np.dot(K, self.H), Pp)

        return self.x[0], self.x[2], self.x[1], self.x[3]  # x위치, y위치, x속력, y속력
