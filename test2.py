import serial
import decoder
import plot
import eulerAngle as eu

com_port = "COM21"
baud_rate = 38400

ser = serial.Serial(com_port, baud_rate)                # 시리얼 객체
myDecoder = decoder.Decoder("./출력파일/flight_data_py.csv")     # 디코딩 객체
myEuler = eu.Euler()
# map_plot = plot.PlotGPS("./지도/하빈.png")              # 지도 플롯 객체
# euler_g_plot = plot.PlotAngle(10)                         # 자세각 플롯 객체
# euler_a_plot = plot.PlotAngle(10)                         # 자세각 플롯 객체
angle3d_plot = plot.PlotAngle3D()

while ser.readable():
    done = myDecoder.check(int.from_bytes(ser.read(), 'little'))  # 패킷 디코딩 시작

    dt = 0.02

    gyro = [i * 0.01745329 for i in myDecoder.gyro_f]
    accel = [i for i in myDecoder.accel_f]
    # gps_time, lat, lon, alt = myDecoder.gps[0], myDecoder.gps[1], myDecoder.gps[2], myDecoder.gps[3]

    euler_g = myEuler.eulerGyro(gyro[0], gyro[1], gyro[2], dt)
    euler_a = myEuler.eulerAccel(accel[0], accel[1], accel[2])

    if done:  # 디코딩이 완료될 시
        # map_plot.gps_tracking(lon, lat, alt)
        if myDecoder.packet_id % 5 == 0:
            # angle3d_plot.euler3DPlot(euler_g[0], euler_g[1], euler_g[2])
            angle3d_plot.euler3DPlot(euler_a[0], euler_a[1], 0)
            # euler_g = [i * 57.29578 for i in euler_g]
            # euler_a = [i * 57.29578 for i in euler_a]
            # euler_g_plot.eulerPlot(euler_g[0], euler_g[1], euler_g[2])
            # euler_a_plot.eulerPlot(euler_a[0], euler_a[1], 0)

        # # euler_g = [i * 57.29578 for i in euler_g]
        # euler_a = [i * 57.29578 for i in euler_a]
        # # euler_g_plot.eulerPlot(euler_g[0], euler_g[1], euler_g[2])
        # euler_a_plot.eulerPlot(euler_a[0], euler_a[1], 0)
# plt.show()
