import decoder
import plot
import numpy as np
import matplotlib.pyplot as plt

input_file_name = "./입력파일/output_static_home.txt"
with open(input_file_name, 'r') as f:  # 입력 파일 읽기
    input_stream = f.read()

a = []
for i in input_stream:
    a.append(int(i, 16))  # 16진수 표기를 10진수 정수로 바꾸기

length = len(a)
input_stream = np.zeros(length//2, dtype=np.uint8)
for i in range(length//2):
    input_stream[i] = a[2*i]*16 + a[2*i + 1]  # 정수를 한 바이트 단위로 합침


myDecoder = decoder.Decoder("./출력파일/flight_data_py.csv")
map_plot = plot.PlotGPS("./지도/하빈.png")
for i in input_stream:
    done = myDecoder.check(i)  # 패킷 디코딩 시작

    if done:
        map_plot.gps_tracking(myDecoder.gps[2], myDecoder.gps[1], myDecoder.gps[3])
plt.show()
