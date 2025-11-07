import serial
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from findport import find_arduino_port

class RadarVisualizer:
    def __init__(self):
        # 시리얼 포트 자동 찾기
        port = find_arduino_port()
        if not port:
            raise Exception("Arduino not found")
        self.ser = serial.Serial(port, 9600, timeout=1)
        
        # 플롯 설정
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = plt.subplot(111, projection='polar')
        self.setup_plot()
        
        # 데이터 버퍼
        self.buffer_size = 50  # 50개의 이전 데이터 포인트 저장
        self.angles = []
        self.distances = []
        
        # 애니메이션 설정 (10ms 간격으로 업데이트)
        self.anim = FuncAnimation(self.fig, self.update, interval=10)
        
    def setup_plot(self):
        self.ax.set_theta_zero_location('N')  # 0도를 북쪽으로 설정
        self.ax.set_theta_direction(-1)      # 시계 방향으로 회전
        self.ax.set_rlabel_position(0)
        self.ax.set_rticks([200, 400, 600, 800])  # 거리 눈금
        self.ax.set_rlim(0, 1000)           # 최대 1000mm
        self.ax.grid(True)
        
        # 고정 센서 포인트 초기화
        self.sensor_points = {
            'front': self.ax.plot([], [], 'ro', label='Front')[0],
            'back': self.ax.plot([], [], 'bo', label='Back')[0],
            'left': self.ax.plot([], [], 'go', label='Left')[0],
            'right': self.ax.plot([], [], 'yo', label='Right')[0]
        }
        
        # 스캔 라인과 히스토리 포인트
        self.scan_line, = self.ax.plot([], [], 'r-', alpha=0.5)
        self.scan_points, = self.ax.plot([], [], 'c.', alpha=0.3)
        
        self.ax.legend()

    def update(self, frame):
        if self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            if line:
                try:
                    # 데이터 파싱: right,left,front,back,servo,forcex,forcey,forcez
                    data = list(map(int, line.split(',')))
                    if len(data) == 8:
                        right, left, front, back, servo, fx, fy, fz = data
                        
                        # 고정 센서 데이터 업데이트 (각도에 맞게 위치 조정)
                        sensor_data = {
                            'front': (0, front),      # 0도
                            'right': (np.pi/2, right),  # 90도
                            'back': (np.pi, back),    # 180도
                            'left': (3*np.pi/2, left)  # 270도
                        }
                        
                        # 각 센서의 데이터 표시
                        for sensor_name, (angle, distance) in sensor_data.items():
                            if distance > 0:  # -1은 범위 초과 또는 에러
                                self.sensor_points[sensor_name].set_data([angle], [distance])
                            else:
                                self.sensor_points[sensor_name].set_data([], [])
                        
                        # 스캔 라인 업데이트
                        scan_angle = np.radians(servo)
                        self.scan_line.set_data([0, scan_angle], [0, 1000])
                        
                        # 스캔 히스토리 업데이트
                        self.angles.append(scan_angle)
                        self.distances.append(front)  # 전면 센서 사용
                        
                        # 버퍼 크기 유지
                        if len(self.angles) > self.buffer_size:
                            self.angles = self.angles[-self.buffer_size:]
                            self.distances = self.distances[-self.buffer_size:]
                        
                        # 유효한 거리 데이터만 표시
                        valid_points = [(a, d) for a, d in zip(self.angles, self.distances) if d > 0]
                        if valid_points:
                            angles, distances = zip(*valid_points)
                            self.scan_points.set_data(angles, distances)
                        else:
                            self.scan_points.set_data([], [])
                        
                except ValueError as e:
                    print(f"Data parsing error: {e}")
                    
        return self.sensor_points.values()
    
    def run(self):
        plt.show()
    
    def close(self):
        self.ser.close()
        plt.close()

if __name__ == "__main__":
    try:
        visualizer = RadarVisualizer()
        visualizer.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'visualizer' in locals():
            visualizer.close()
