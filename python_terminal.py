import serial
import decoder
import plot
import kalmanFilter as ft

com_port = "COM21"
baud_rate = 38400

ser = serial.Serial(com_port, baud_rate)                # 시리얼 객체
myDecoder = decoder.Decoder("./출력파일/flight_data_py.csv")     # 디코딩 객체
# myKalman = ft.EulerKalman()                             # 칼만필터 객체
myEKF = ft.EulerEKF()
myGpsKalman = ft.GPSKalman()
map_plot = plot.PlotGPS("./지도/하빈.png")              # 지도 플롯 객체
angle_plot = plot.PlotAngle(10)                         # 자세각 플롯 객체
angle3d_plot = plot.PlotAngle3D()

old_id = -1
while ser.readable():
    done = myDecoder.check(int.from_bytes(ser.read(), 'little'))  # 패킷 디코딩 시작

    # w = 0
    # now_id = myDecoder.packet_id
    # if now_id < old_id:
    #     w = 65535
    # dt = 0.02 * (now_id - old_id + w)
    # if old_id == -1:
    #     dt = 0.02
    # old_id = now_id

    dt = 0.02

    gyro = [i*0.0174532 for i in myDecoder.gyro_f]
    accel = [i for i in myDecoder.accel_f]
    gps_time, lat, lon, alt = myDecoder.gps[0], myDecoder.gps[1], myDecoder.gps[2], myDecoder.gps[3]

    # phi, theta, psi = myKalman.calcFilter(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt)
    euler = myEKF.calcFilter(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt)

    lon_k, lat_k, lon_v, lat_v = 0, 0, 0, 0
    if gps_time and lat and lon:
        lon_k, lat_k, lon_v, lat_v = myGpsKalman.trackKalman(lon, lat, 1)
        print(lon_k, lat_k, lon_v, lat_v)

    if done:  # 디코딩이 완료될 시
        # map_plot.gps_tracking(lon, lat, alt)
        # map_plot.gps_tracking(lon_k, lat_k, alt)
        # print(lon_v, lat_v)
        if myDecoder.packet_id % 20 == 0:
            angle3d_plot.euler3DPlot(euler[0], euler[1], euler[2])
            euler = [i*57.29578 for i in euler]
            angle_plot.eulerPlot(euler[0], euler[1], euler[2])
# plt.show()
