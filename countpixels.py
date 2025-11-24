# -*- coding: utf-8 -*-
import smbus
import time
import math
import RPi.GPIO as GPIO
import cv2
import numpy as np
import os

# --- 상수 정의 ---
I2C_ADDRESS = 0x16
SPEED = 40                 # 평소 주행 속도
TURN_SPEED = 30            # 회피 기동 및 회피 후 이동 속도 (최소 구동 속도 고려)
SAFE_DISTANCE = 40         # cm, 감지 거리
TRIG_PIN = 23              # 초음파 센서 TRIG 핀 번호 (BCM 기준)
ECHO_PIN = 24              # 초음파 센서 ECHO 핀 번호 (BCM 기준)
CAPTURE_DIR = "obstacle_captures" # 사진 저장 폴더

# ---------------------------------------------------------
# 모터 제어 클래스 
# ---------------------------------------------------------
class YB_Pcb_Car(object):
    
    def get_i2c_device(self, address, i2c_bus):
        self._addr = address
        try:
            return smbus.SMBus(1 if i2c_bus is None else i2c_bus)
        except Exception as e:
            print(f"[Error] I2C 버스 초기화 실패: {e}")
            print("I2C 통신이 활성화되었는지, smbus 라이브러리가 설치되었는지 확인하세요.")
            return None

    def __init__(self):  
        self._device = self.get_i2c_device(0x16, 1)  

    def write_u8(self, reg, data):
        if not self._device: return
        try:
            self._device.write_byte_data(self._addr, reg, data)
        except Exception as e:
            print (f'[Error] write_u8 I2C error: {e}')

    def write_array(self, reg, data):
        if not self._device: return
        try:
            self._device.write_i2c_block_data(self._addr, reg, data)
        except Exception as e:
            print (f'[Error] write_array I2C error: {e}')

    def Ctrl_Car(self, l_dir, l_speed, r_dir, r_speed):
        if not self._device: return
        try:
            self.write_array(0x01, [l_dir, l_speed, r_dir, r_speed])
        except:
            pass 

    def Car_Run(self, speed1, speed2): self.Ctrl_Car(1, speed1, 1, speed2)
    def Car_Stop(self): self.write_u8(0x02, 0x00)
    def Car_Back(self, speed1, speed2): self.Ctrl_Car(0, speed1, 0, speed2)
    def Car_Left(self, speed1, speed2): self.Ctrl_Car(0, speed1, 1, speed2)
    def Car_Right(self, speed1, speed2): self.Ctrl_Car(1, speed1, 0, speed2)

# ---------------------------------------------------------
# 초음파 센서 클래스 
# ---------------------------------------------------------
class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.1)

    def get_distance(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001) 
        GPIO.output(self.trig, False)
        
        start_time, stop_time = time.time(), time.time()
        timeout = time.time() + 0.04

        while GPIO.input(self.echo) == 0 and time.time() < timeout:
            start_time = time.time()
        
        if time.time() >= timeout: return 400

        while GPIO.input(self.echo) == 1 and time.time() < timeout:
            stop_time = time.time()
        
        if time.time() >= timeout: return 400
             
        elapsed = stop_time - start_time
        dist = (elapsed * 34300) / 2
        
        return dist if 2 < dist < 400 else 400

# ---------------------------------------------------------
# 경로 분석 함수 
# ---------------------------------------------------------
def find_open_path(frame):
    """
    현재 카메라 프레임을 분석하여 왼쪽 또는 오른쪽 중 더 뚫린 길을 결정합니다.
    """
    height, width, _ = frame.shape
    roi = frame[height//2:height, 0:width]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # 밝기(V)가 높은 영역을 장애물로 간주
    lower_bound = np.array([0, 0, 150])
    upper_bound = np.array([180, 50, 255])
    obstacle_mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    center_x = obstacle_mask.shape[1] // 2
    left_mask = obstacle_mask[:, :center_x]
    right_mask = obstacle_mask[:, center_x:]
    
    left_obstacle_area = cv2.countNonZero(left_mask)
    right_obstacle_area = cv2.countNonZero(right_mask)
    
    print(f"   [분석 결과] 좌측 장애물 픽셀 수: {left_obstacle_area}")
    print(f"   [분석 결과] 우측 장애물 픽셀 수: {right_obstacle_area}")

    if left_obstacle_area < right_obstacle_area:
        return "LEFT"
    elif right_obstacle_area < left_obstacle_area:
        return "RIGHT"
    else:
        return "RIGHT" 

# ---------------------------------------------------------
# 메인 루프 (수정됨: TURN_SPEED = 30)
# ---------------------------------------------------------
def main():
    GPIO.setwarnings(False) 
    
    # 0. 초기화
    print("시스템 초기화 중...")
    car = YB_Pcb_Car()
    if not car._device: return

    sensor = UltrasonicSensor(TRIG_PIN, ECHO_PIN)
    
    # 카메라 초기화
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다. 종료합니다.")
        GPIO.cleanup()
        return

    # 캡처 폴더 생성
    if not os.path.exists(CAPTURE_DIR):
        os.makedirs(CAPTURE_DIR)
        print(f"캡처 폴더 생성: {CAPTURE_DIR}")
        
    is_avoiding = False

    try:
        print(f"자율주행 시작 (평소 속도: {SPEED}, 회피 속도: {TURN_SPEED}, 감지 거리: {SAFE_DISTANCE}cm)")
        car.Car_Run(SPEED, SPEED) 

        while True:
            # 1. 프레임 획득
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            
            # 2. 메인 로직: 회피 중일 때 vs. 일반 주행 중일 때
            if is_avoiding:
                # 회피 기동 중에는 추가적인 감지/분석/회피 로직을 건너뜁니다.
                print(f"거리: {sensor.get_distance():.1f}cm | 상태: 회피 기동 중...", end='\r')
                time.sleep(0.05)
                continue
            
            # --- 일반 주행 로직 ---
            dist = sensor.get_distance()
            print(f"거리: {dist:.1f}cm | 상태: 주행 중", end='\r')

            # 3. 장애물 감지 로직 (최우선)
            if dist < SAFE_DISTANCE:
                is_avoiding = True 
                
                car.Car_Stop()
                print("\n장애물 감지! 멈춤 및 경로 분석 시작...")
                
                # 2-1. 사진 찍기
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(CAPTURE_DIR, f"obstacle_{timestamp}.jpg")
                cv2.imwrite(filename, frame)
                print(f"사진 저장 완료: {filename}")
                
                # 2-2. 경로 분석
                open_path = find_open_path(frame)
                print(f"최적 경로: {open_path}으로 회피")
                
                # 2-3. 회피 기동 실행
                turn_time = 1.0 
                
                # 회피 속도 TURN_SPEED(30) 적용
                if open_path == "LEFT":
                    car.Car_Left(TURN_SPEED, TURN_SPEED)
                    time.sleep(turn_time)
                elif open_path == "RIGHT":
                    car.Car_Right(TURN_SPEED, TURN_SPEED)
                    time.sleep(turn_time)
                else: 
                     car.Car_Right(TURN_SPEED, TURN_SPEED) 
                     time.sleep(turn_time)
                
                car.Car_Stop()
                time.sleep(0.5)
                
                # 회피가 끝났으므로 플래그 해제
                is_avoiding = False
                
                # 회피 후 느린 속도 (30)로 전진 재개
                print(f"회피 완료, 느린 속도({TURN_SPEED})로 주행 재개.")
                car.Car_Run(TURN_SPEED, TURN_SPEED)
            
            else:
                # 장애물이 없으면 평소 속도 (40)로 계속 직진
                car.Car_Run(SPEED, SPEED)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n수동 종료 (Ctrl+C)")
    except Exception as e:
        print(f"\n[오류 발생] 치명적인 오류: {e}")
    finally:
        # 프로그램 종료 시 반드시 실행
        car.Car_Stop()
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("프로그램 종료 및 리소스 해제 완료")

if __name__ == '__main__':
    main()
