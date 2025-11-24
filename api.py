import cv2
import time
from ultralytics import YOLO  # YOLOv5n 대신 YOLOv8n도 가능
from openai import OpenAI
from car import YB_Pcb_Car     # 직접 만든 모터 제어 클래스
# === 초기 설정 ===
client = OpenAI(api_key="YOUR_OPENAI_API_KEY")
car = YB_Pcb_Car()
model = YOLO("yolov5n.pt")   # YOLOv5n 모델 파일 (가볍고 빠름)
camera = cv2.VideoCapture(0)
# === 메인 루프 ===
while True:
    ret, frame = camera.read()
    if not ret:
        break
    # 1️객체 인식
    results = model(frame)
    labels = [r.names[int(c)] for c in results[0].boxes.cls]
   
    # 관심 객체만 추출
    detected = [l for l in labels if l in ["traffic light", "person", "car"]]
    # 2️ 상황을 설명할 문장 생성
    scene_desc = f"I see: {', '.join(detected)}."
    print("Scene:", scene_desc)
    # 3️ GPT에게 상황 판단 요청
    prompt = f"""
    상황: {scene_desc}
    차량 제어 규칙:
    - 빨간불 → 정지
    - 초록불 → 전진
    - 사람 감지 → 감속 또는 정지
    - 아무것도 없으면 → 전진
    출력은 '정지', '전진', '감속' 중 하나로만 해주세요.
    """
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[{"role": "user", "content": prompt}]
    )
    command = response.choices[0].message.content.strip()
    print("GPT 판단:", command)
    # 4️ 제어 실행
    if "정지" in command:
        car.Car_Stop()
    elif "감속" in command:
        car.Car_Slow()
    elif "전진" in command:
        car.Car_Run()
    else:
        car.Car_Stop()
    time.sleep(0.5)
camera.release()
cv2.destroyAllWindows()
