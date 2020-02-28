# 출력 결과 파일에서 데이터를 읽어서 플로팅하는 코드

import csv
import numpy as np
import matplotlib.pyplot as plt
import kalmanFilter as ft
import plot

f = open("./출력파일/flight_data_py.csv", 'r')
reader = csv.reader(f)

timestamp = []
EulerSaved = []

# map_plot = plot.PlotGPS("./지도/하빈.png")
# angle_plot = plot.PlotAngle(50)
angle_3d = plot.PlotAngle3D()
myKalman = ft.EulerKalman()
myEKF = ft.EulerEKF()

for line in reader:
    # 가속도 자이로 데이터
    timestamp.append(line[0])
    (p, q, r, ax, ay, az) = [float(i) for i in line[1:7]]
    p *= 0.0174532
    q *= 0.0174532
    r *= 0.0174532
    dt = 0.02

    # 칼만 필터
    # (phi, theta, psi) = myKalman.calcFilter(p, q, r, ax, ay, az, dt)
    (phi, theta, psi) = myEKF.calcFilter(p, q, r, ax, ay, az, dt)
    # angle_plot.eulerPlot(phi, theta, psi)
    EulerSaved.append([phi, theta, psi])

    # GPS 데이터
    # (gps_time, lat, lon, alt) = [float(i) for i in line[7:]]
    # map_plot.gps_tracking(lon, lat, alt)

f.close()

EulerSaved_degree = np.array(EulerSaved) * 57.29578

fig, ax = plt.subplots()
ax.set_xticks(ticks=[])
ax.set_yticks(ticks=range(-90, 90, 10))
ax.plot(timestamp, EulerSaved_degree[:, 0], 'r', linewidth=1)
ax.plot(timestamp, EulerSaved_degree[:, 1], 'g', linewidth=1)
ax.plot(timestamp, EulerSaved_degree[:, 2], 'b', linewidth=1)

# for i in EulerSaved:
#     angle_3d.euler3DPlot(i[0], i[1], i[2])
    # plt.pause(0.01)

plt.show()
