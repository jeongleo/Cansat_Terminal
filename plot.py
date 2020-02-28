import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from math import *

class PlotGPS:
    def __init__(self, map_img):
        self.img = mpimg.imread(map_img)

        # --------------- 기준점 위도 경도 --------------
        # 건국대학교 지도
        # self.lat1 = 37.537910  # 지도 좌측 좌표
        # self.lon1 = 127.069595
        # self.lat2 = 37.545226  # 지도 우측 좌표
        # self.lon2 = 127.085186
        # 겨울 발사장 지도
        # self.lat1 = 37.2487309  # 지도 좌측 좌표
        # self.lon1 = 126.6459083
        # self.lat2 = 37.2525239  # 지도 우측 좌표
        # self.lon2 = 126.651932
        # 하빈 지도
        self.lat1 = 35.910032  # 지도 좌측 좌표
        self.lon1 = 128.441030
        self.lat2 = 35.914741  # 지도 우측 좌표
        self.lon2 = 128.452081

        # -------------- 사진 좌표 data 값 ---------------
        # 건국대학교 지도
        # self.x1 = 0  # 지도 좌측 좌표
        # self.y1 = 565
        # self.x2 = 726  # 지도 우측 좌표
        # self.y2 = 0
        # 겨울발사장 지도
        # self.x1 = 0  # 지도 좌측 좌표
        # self.y1 = 364
        # self.x2 = 628  # 지도 우측 좌표
        # self.y2 = 0
        # 하빈 지도
        self.x1 = 0  # 지도 좌측 좌표
        self.y1 = 406
        self.x2 = 770  # 지도 우측 좌표
        self.y2 = 0

        # figure과 axis 만들고 범위 제한
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.img)  # 지도 그리기
        self.ax.set_xlim(self.x1, self.x2)
        self.ax.set_ylim(self.y1, self.y2)
        plt.show(block=False)

    def gps_tracking(self, lon, lat, alt):
        if lon < self.lon1 or lat < self.lat1 or lon > self.lon2 or lat > self.lat2:
            return False

        x_cor = (self.x2 - self.x1) / (self.lon2 - self.lon1) * (lon - self.lon1) + self.x1
        y_cor = (self.y1 - self.y2) / (self.lat1 - self.lat2) * (lat - self.lat1) + self.y1

        self.ax.plot(x_cor, y_cor, 'r.', ms=2)
        # self.fig.canvas.draw()
        plt.pause(0.000001)


class PlotAngle:
    def __init__(self, data_len):
        self.fig, self.ax = plt.subplots()
        plt.show(block=False)

        self.timestamp = [i for i in range(data_len)]
        self.phi_data = [0 for i in range(data_len)]
        self.theta_data = [0 for i in range(data_len)]
        self.psi_data = [0 for i in range(data_len)]

    def eulerPlot(self, phi, theta, psi):
        self.append_data(phi, theta, psi)

        self.ax.clear()
        self.ax.set_ylim(-180, 180)
        self.ax.set_yticks(ticks=range(-180, 180, 30))

        self.ax.plot(self.timestamp, self.phi_data, 'r')
        self.ax.plot(self.timestamp, self.theta_data, 'g')
        self.ax.plot(self.timestamp, self.psi_data, 'b')

        # self.fig.canvas.draw()
        plt.pause(0.000001)

    def append_data(self, a, b, c):
        self.phi_data.pop(0)
        self.theta_data.pop(0)
        self.psi_data.pop(0)

        self.phi_data.append(a)
        self.theta_data.append(b)
        self.psi_data.append(c)


class PlotAngle3D:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.view_init(30, -120)

        plt.show(block=False)

        self.point_body = np.array([[1, 2, 0],
                                    [-1, 2, 0],
                                    [-1, -2, 0],
                                    [1, -2, 0],
                                    [1, 2, 0]])
        self.point_axis = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])

    def euler3DPlot(self, phi, theta, psi):
        sinPhi, sinTheta, sinPsi = sin(phi), sin(theta), sin(psi)
        cosPhi, cosTheta, cosPsi = cos(phi), cos(theta), cos(psi)

        R_x = np.array([[1, 0, 0],
                        [0, cosPhi, -sinPhi],
                        [0, sinPhi, cosPhi]])
        R_y = np.array([[cosTheta, 0, sinTheta],
                        [0, 1, 0],
                        [-sinTheta, 0, cosTheta]])
        R_z = np.array([[cosPsi, -sinPsi, 0],
                        [sinPsi, cosPsi, 0],
                        [0, 0, 1]])
        R = np.dot(np.dot(R_z, R_y), R_x)

        r_point_body = np.dot(R, self.point_body.T).T
        r_point_axis = np.dot(R, self.point_axis.T).T

        self.ax.clear()
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_zlim(-3, 3)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        # self.ax.plot(self.point_body[:, 0], self.point_body[:, 1], self.point_body[:, 2], 'black', alpha=0.3)
        self.ax.plot(r_point_body[:, 0], r_point_body[:, 1], r_point_body[:, 2], 'r')
        for i in range(3):
            self.ax.plot([0, self.point_axis[i, 0]], [0, self.point_axis[i, 1]], [0, self.point_axis[i, 2]],
                         'black', alpha=0.3)
            self.ax.plot([0, r_point_axis[i, 0]], [0, r_point_axis[i, 1]], [0, r_point_axis[i, 2]])

        plt.pause(0.00000001)
