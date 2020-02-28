import numpy as np
import csv


class Decoder:
    def __init__(self, save_file_name="./출력파일/flight_data.csv", file_mode='w'):
        # 패킷 헤더 확인용 변수
        self.data = np.zeros(100, dtype=np.uint8)
        self.index = 0
        self.headerCHK = 0
        self.data_size = 0

        # 패킷 데이터 저장용 변수
        self.packet_id = 0
        self.gyro = np.zeros(3, dtype=np.int16)
        self.accel = np.zeros(3, dtype=np.int16)
        self.gps = np.zeros(4, dtype=np.float32)
        self.gyro_f = np.zeros(3, dtype=np.float32)
        self.accel_f = np.zeros(3, dtype=np.float32)

        # 출력 파일 초기화
        self.save_file_name = save_file_name
        file_output = open(save_file_name, file_mode)  
        file_output.close()
    
    # 패킷 헤더 및 체크섬 확인
    def check(self, input):
        self.data[self.index] = input
        self.index += 1

        if self.headerCHK == 0:  # header1 : FF
            if input == 255:
                self.headerCHK += 1
            else:
                self.headerCHK = 1
                self.index = 1

        elif self.headerCHK == 1:  # header2 : FE
            if input == 254:
                self.headerCHK += 1
            else:
                self.headerCHK = 1
                self.index = 1

        elif self.headerCHK == 2:  # header3 : FD
            if input == 253:
                self.headerCHK += 1
            else:
                self.headerCHK = 1
                self.index = 1

        elif self.headerCHK == 3 or self.headerCHK == 4:  # packet_id(2byte)
            self.headerCHK += 1

        elif self.headerCHK == 5:  # data_size
            self.data_size = input
            self.headerCHK += 1

        if self.index >= 35:  # data_size+7
            self.index = 0  # 패킷 헤더 확인 변수 초기화
            self.headerCHK = 0
            self.data_size = 0

            checksum = self.data[0]  # 체크섬 초기화, 패킷의 첫번째 값
            for i in range(1, self.index - 3):  # 체크섬 확인
                checksum ^= self.data[i]  # xor연산

            if checksum ^ self.data[self.index - 2] == 0:  # 체크섬 일치하면 패켓 헤더 확인 완료
                self.decode()
                return True  # 데이터 디코딩 완료 시 true

        return False  # 디코딩 미완료 시 false

    # 패킷 데이터 파싱 및 저장
    def decode(self):
        self.packet_id = self.data[3:5].view(np.uint16)[0]
        self.gyro[0] = self.data[6:8].view(np.int16)
        self.gyro[1] = self.data[8:10].view(np.int16)
        self.gyro[2] = self.data[10:12].view(np.int16)
        self.accel[0] = self.data[12:14].view(np.int16)
        self.accel[1] = self.data[14:16].view(np.int16)
        self.accel[2] = self.data[16:18].view(np.int16)
        self.gps[0] = self.data[18:22].view(np.float32)
        self.gps[1] = self.data[22:26].view(np.float32)
        self.gps[2] = self.data[26:30].view(np.float32)
        self.gps[3] = self.data[30:34].view(np.float32)

        for i in range(3):
            self.gyro_f[i] = self.gyro[i] / 131.0  # 32.8, 1048
            self.accel_f[i] = self.accel[i] / 16384.0

        self.gps[1] = minute2degree(self.gps[1])
        self.gps[2] = minute2degree(self.gps[2])

        # if self.packet_id % 1000 == 0:
        # print("%d\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.1f\t%.2f\t%.2f\t%.2f"
        #       % (self.packet_id, self.gyro_f[0], self.gyro_f[1], self.gyro_f[2], self.accel_f[0], self.accel_f[1],
        #          self.accel_f[2], self.gps[0], self.gps[1], self.gps[2], self.gps[3]))
        
        # 데이터 저장
        self.save_data()
    
    # csv 파일로 저장하는 함수
    def save_data(self):
        with open(self.save_file_name, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.packet_id, self.gyro_f[0], self.gyro_f[1], self.gyro_f[2], self.accel_f[0],
                             self.accel_f[1], self.accel_f[2], self.gps[0], self.gps[1], self.gps[2], self.gps[3]])


def minute2degree(minute):
    try:
        n = int(minute/100)     # 정수부(도 단위)
        f = minute - n*100      # 소수부(분 단위)
        nf = f/60.0             # 소수부(도 단위)
    except:
        return 0.0

    return n + nf
