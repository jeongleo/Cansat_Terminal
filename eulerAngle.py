# 가속도 자이로 값으로 오일러 각을 계산하는 클래스

from math import *


class Euler:
    def __init__(self):
        # 자이로 적분에서 직전 각도를 저장할 변수 초기화
        self.prevPhi = 0
        self.prevTheta = 0
        self.prevPsi = 0

    def eulerGyro(self, p, q, r, dt):
        sinPhi = sin(self.prevPhi)
        cosPhi = cos(self.prevPhi)
        cosTheta = cos(self.prevTheta)
        tanTheta = tan(self.prevTheta)

        self.prevPhi += dt*(p + q*sinPhi*tanTheta + r*cosPhi*tanTheta)
        self.prevTheta += dt*(q*cosPhi - r*sinPhi)
        self.prevPsi += dt*(q*sinPhi/cosTheta + r*cosPhi/cosTheta)

        # self.prevPhi += dt*p
        # self.prevTheta += dt*q
        # self.prevPsi += dt*r

        return self.prevPhi, self.prevTheta, self.prevPsi

    @staticmethod
    def eulerAccel(ax, ay, az):
        phi = atan2(ay, az)
        theta = atan2(-ax, sqrt(ay**2 + az**2))

        return phi, theta
