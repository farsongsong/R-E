# -*- coding: utf-8 -*-
import time
import smbus
import RPi.GPIO as GPIO
import cv2
import numpy as np

# ---------------------------------------------------------
# 모터 제어 클래스
# ---------------------------------------------------------
class YB_Pcb_Car(object):
    def get_i2c_device(self, address, i2c_bus=None):
        self._addr = address
        return smbus.SMBus(1 if i2c_bus is None else i2c_bus)

    def __init__(self):
        self._device = self.get_i2c_device(0x16, 1)

    def write_u8(self, reg, data):
        try:
            self._device.write_byte_data(self._addr, reg, data)
        except:
            print('[Error] write_u8: I2C 통신 문제')

    def write_array(self, reg, data):
        try:
            self._device.write_i2c_block_data(self._addr, reg, data)
        except:
            print('[Error] write_array: I2C 통신 문제')

    def Ctrl_Car(self, l_dir, l_speed, r_dir, r_speed):
        try:
            self.write_array(0x01, [l_dir, l_speed, r_dir, r_speed])
        except:
            print('[Error] Ctrl_Car: I2C 통신 문제')

    def Car_Run(self, s1, s2): self.Ctrl_Car(1, s1, 1, s2)
    def Car_Back(self, s1, s2): self.Ctrl_Car(0, s1, 0, s2)
    def Car_Left(self, s1, s2): self.Ctrl_Car(0, s1, 1, s2)
    def Car_Right(self, s1, s2): self.Ctrl_Car(1, s1, 0, s2)
    def Car_Stop(self): self.write_u8(0x02, 0x00)

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
        time.sleep(0.5)

    def get_distance(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)
        start, stop = time.time(), time.time()
        timeout = time.time() + 0.04

        while GPIO.input(self.echo) == 0 and time.time() < timeout:
            start = time.time()
        while GPIO.input(self.echo) == 1 and time.time() < timeout:
            stop = time.time()

        elapsed = stop - start
        dist = (elapsed * 34300) / 2
        return dist if dist < 400 else 400

# ---------------------------------------------------------
# 메인 루프 (초음파 + 신호등 감지)
# ---------------------------------------------------------
if __name__ == '__main__':
    print("I2C 카 컨트롤러 초기화 중...")
    car = YB_Pcb_Car()
    sensor = UltrasonicSensor(23, 24)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        exit()

    SPEED = 40
    SAFE_DISTANCE = 25
    stopped_due_to_red = False  # 빨간불 때문에 멈췄는가?
    stopped_due_to_obstacle = False  # 장애물 때문에 멈췄는가?

    try:
        print("자율주행 시작 (빨간불 멈춤 / 초록불 재출발 / 장애물 회피)")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("카메라 프레임 수신 실패")
                continue

            frame = cv2.resize(frame, (320, 240))
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # === 색상 범위 (조명 환경에 맞게 조정 가능) ===
            lower_red1, upper_red1 = np.array([0,100,100]), np.array([10,255,255])
            lower_red2, upper_red2 = np.array([160,100,100]), np.array([180,255,255])
            lower_green, upper_green = np.array([35,100,100]), np.array([85,255,255])

            mask_red = cv2.bitwise_or(
                cv2.inRange(hsv, lower_red1, upper_red1),
                cv2.inRange(hsv, lower_red2, upper_red2)
            )
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            red_area = cv2.countNonZero(mask_red)
            green_area = cv2.countNonZero(mask_green)
            dist = sensor.get_distance()

            print(f"거리: {dist:.1f}cm | 빨강: {red_area} | 초록: {green_area}")

            # === 1. 장애물 감지 ===
            if dist < SAFE_DISTANCE:
                print("장애물 감지 → 회피 중...")
                car.Car_Stop()
                stopped_due_to_obstacle = True
                time.sleep(0.5)

                # 후진 후 왼쪽 회피
                car.Car_Back(30, 30)
                time.sleep(1)
                car.Car_Left(30, 30)
                time.sleep(0.7)

                car.Car_Stop()
                stopped_due_to_obstacle = False
                stopped_due_to_red = False  # 신호와 무관하게 초기화

            # === 2. 빨간불 감지 ===
            elif red_area > 1500:
                if not stopped_due_to_red:
                    print("빨간불 감지 → 정지")
                    car.Car_Stop()
                    stopped_due_to_red = True

            # === 3. 초록불 감지 ===
            elif green_area > 1500:
                if stopped_due_to_red:
                    print("초록불 감지 → 다시 출발")
                    car.Car_Run(SPEED, SPEED)
                    stopped_due_to_red = False
                else:
                    car.Car_Run(SPEED, SPEED)

            # === 4. 평상시 주행 ===
            else:
                if not stopped_due_to_red:  # 빨간불 중에는 대기
                    car.Car_Run(SPEED, SPEED)
                else:
                    car.Car_Stop()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n수동 종료")
    except Exception as e:
        print("[오류 발생] {}".format(e))
    finally:
        car.Car_Stop()
        GPIO.cleanup()
        cap.release()
        cv2.destroyAllWindows()
        print("프로그램 종료 완료")
