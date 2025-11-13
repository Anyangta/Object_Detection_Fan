import time
import cv2
import numpy as np
import spidev

# ------------------- SPI 초기화 -------------------
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

# ------------------- 모델 로드 -------------------
print("=" * 60)
print("YOLOv8 모델 로딩 중...")
model_path = 'yolov8n.onnx'
net = cv2.dnn.readNet(model_path)
class_names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
print("✓ 모델 로드 완료!")
print("=" * 60)

# ------------------- 카메라 설정 -------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
if not cap.isOpened():
    print("오류: 카메라를 열 수 없습니다.")
    exit()

# ------------------- 제어 설정 -------------------
confidence_threshold = 0.5
input_size = 160
MIN_ANGLE = 10
MAX_ANGLE = 170
CENTER_ANGLE = 90

# SPI 명령/상태 코드
RESET_COMMAND = 200
START_COMMAND = 255
POLL_COMMAND = 0
STATUS_RUNNING = 222
STATUS_READY = 111
STATUS_HOMING_OFF = 0

# 제어 파라미터
DEAD_ZONE_PERCENT = 0.275
MOVE_SPEED = 3
RESET_TIMEOUT = 5.0

# ------------------- 상태 변수 -------------------
current_state = 'WAITING_BUTTON'  # 초기: 버튼 대기
current_angle = CENTER_ANGLE 
last_direction = 'none' 
wait_start_time = 0

# FPS 계산
frame_count = 0
start_time = time.time()
last_frame = None

try:
    print("\n" + "=" * 60)
    print("  스마트 팬 제어 시스템 시작")
    print("=" * 60)
    print("  • PD1 버튼을 눌러 시스템을 시작하세요")
    print("  • 종료: 'q' 키")
    print("=" * 60 + "\n")
    
    while True:
        
        # ========== [상태 1] 버튼 대기 (초기 시작) ==========
        if current_state == 'WAITING_BUTTON':
            ret, frame = cap.read()
            if not ret:
                continue
            
            # 대기 화면 표시
            display = frame.copy()
            overlay = np.zeros_like(display)
            cv2.rectangle(overlay, (0, FRAME_HEIGHT//2-70), (FRAME_WIDTH, FRAME_HEIGHT//2+70), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, display, 0.3, 0, display)
            
            cv2.putText(display, "Press PD1 Button to Start", (FRAME_WIDTH//2-260, FRAME_HEIGHT//2-20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 2)
            cv2.putText(display, "Model Loaded - Ready", (FRAME_WIDTH//2-190, FRAME_HEIGHT//2+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            cv2.imshow("Smart Fan Controller", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # ATmega 상태 폴링
            try:
                response = spi.xfer2([POLL_COMMAND])
                if response and response[0] == STATUS_READY:
                    print("✓ ATmega 준비 완료 (111 수신)")
                    print("✓ 서보모터 90도 위치 확인")
                    
                    # 팬 시작 명령
                    time.sleep(0.1)
                    response_start = spi.xfer2([START_COMMAND])
                    print(f"✓ 팬 시작 명령(255) 전송, 응답: {response_start[0] if response_start else 'None'}")
                    
                    # 상태 전환
                    current_angle = CENTER_ANGLE
                    current_state = 'IDLE'
                    last_direction = 'none'
                    frame_count = 0
                    start_time = time.time()
                    
                    print("✓ 객체 탐지 시작!\n")
            except Exception as e:
                print(f"폴링 오류: {e}")
            
            time.sleep(0.3)
            continue

        # ========== [상태 2] 정지 상태 (리셋 후) ==========
        if current_state == 'STOPPED':
            if last_frame is not None:
                display = last_frame.copy()
            else:
                ret, display = cap.read()
                if not ret:
                    continue
            
            # 정지 화면 표시
            overlay = np.zeros_like(display)
            cv2.rectangle(overlay, (0, FRAME_HEIGHT//2-90), (FRAME_WIDTH, FRAME_HEIGHT//2+90), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, display, 0.3, 0, display)
            
            cv2.putText(display, "SYSTEM STOPPED", (FRAME_WIDTH//2-200, FRAME_HEIGHT//2-40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
            cv2.putText(display, "Fan OFF - Servo at 90deg", (FRAME_WIDTH//2-230, FRAME_HEIGHT//2+5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, "Press PD1 to Restart", (FRAME_WIDTH//2-200, FRAME_HEIGHT//2+45), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
            
            cv2.imshow("Smart Fan Controller", display)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break
            
            # ATmega 상태 폴링
            try:
                response = spi.xfer2([POLL_COMMAND])
                if response and response[0] == STATUS_READY:
                    print("\n✓ 사용자가 PD1 버튼을 눌렀습니다")
                    print("✓ 시스템 재시작 준비...\n")
                    current_state = 'WAITING_BUTTON'
            except Exception as e:
                print(f"폴링 오류: {e}")
            
            time.sleep(0.5)
            continue

        # ========== [상태 3] 작동 중 (객체 탐지) ==========
        ret, frame = cap.read()
        if not ret:
            print("프레임 읽기 실패")
            continue
        last_frame = frame.copy()

        # 객체 탐지
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (input_size, input_size), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(net.getUnconnectedOutLayersNames())[0].T

        detected_persons = []
        for detection in outputs:
            classes_scores = detection[4:]
            class_id = np.argmax(classes_scores)
            confidence = classes_scores[class_id]
            if confidence >= confidence_threshold and class_names[class_id] == 'person':
                x_factor = frame.shape[1] / input_size
                y_factor = frame.shape[0] / input_size
                cx, cy, w, h = detection[:4]
                left = int((cx - w / 2) * x_factor)
                width = int(w * x_factor)
                top = int((cy - h / 2) * y_factor)
                height = int(h * y_factor)
                detected_persons.append({
                    'center_x': left + width / 2, 
                    'box': [left, top, width, height], 
                    'area': width * height
                })
        
        person_detected = len(detected_persons) > 0
        target_angle = current_angle

        # 상태 전환 로직
        if person_detected and current_state != 'TRACKING':
            current_state = 'TRACKING'
            print(f"→ TRACKING (사람 감지)")
        elif not person_detected and current_state == 'TRACKING':
            current_state = 'SEARCHING'
            print(f"→ SEARCHING (사라짐, 방향: {last_direction})")

        # 상태별 동작
        if current_state == 'TRACKING':
            # 사람 추적
            target_person = max(detected_persons, key=lambda p: p['area'])
            center_x = target_person['center_x']
            dead_zone_width = FRAME_WIDTH * DEAD_ZONE_PERCENT
            dead_zone_start = (FRAME_WIDTH / 2) - (dead_zone_width / 2)
            dead_zone_end = (FRAME_WIDTH / 2) + (dead_zone_width / 2)

            if center_x < dead_zone_start:
                target_angle = current_angle - MOVE_SPEED
                last_direction = 'left'
            elif center_x > dead_zone_end:
                target_angle = current_angle + MOVE_SPEED
                last_direction = 'right'
            else:
                target_angle = current_angle
                last_direction = 'none'

        elif current_state == 'SEARCHING':
            # 사라진 방향으로 계속 이동
            if last_direction == 'left':
                target_angle = current_angle - MOVE_SPEED
            elif last_direction == 'right':
                target_angle = current_angle + MOVE_SPEED
            
            # 끝 도달 시 대기
            if target_angle <= MIN_ANGLE or target_angle >= MAX_ANGLE:
                current_state = 'WAITING'
                wait_start_time = time.time()
                print(f"→ WAITING (끝 도달: {current_angle}도, 5초 대기)")

        elif current_state == 'WAITING':
            if person_detected:
                current_state = 'TRACKING'
                print(f"→ TRACKING (사람 재발견)")
            elif time.time() - wait_start_time > RESET_TIMEOUT:
                # 타임아웃: 초기화
                print("\n" + "=" * 60)
                print("⚠  5초 타임아웃: 시스템 초기화")
                print("=" * 60)
                
                try:
                    response = spi.xfer2([RESET_COMMAND])
                    print(f"✓ 리셋 명령({RESET_COMMAND}) 전송")
                    print(f"  ATmega 응답: {response[0] if response else 'None'}")
                    print("✓ 팬 정지 및 서보 90도 복귀 시작")
                    print("=" * 60 + "\n")
                except Exception as e:
                    print(f"✗ 리셋 명령 오류: {e}\n")
                
                current_state = 'STOPPED'
                current_angle = CENTER_ANGLE
                last_direction = 'none'
                continue

        elif current_state == 'IDLE':
            target_angle = CENTER_ANGLE
            if person_detected:
                current_state = 'TRACKING'
                print(f"→ TRACKING (사람 감지)")

        # 각도 계산 및 SPI 전송
        final_angle = int(max(MIN_ANGLE, min(MAX_ANGLE, target_angle)))

        try:
            # 각도 전송 및 ATmega 상태 확인
            response = spi.xfer2([final_angle])
            atmega_status = response[0] if response else None
            
            if atmega_status == STATUS_RUNNING:
                # 정상 작동 중
                current_angle = final_angle
                
            elif atmega_status == STATUS_HOMING_OFF:
                # 수동 정지 감지!
                print("\n" + "=" * 60)
                print("⚠  ATmega 수동 정지 감지 (PD1 버튼으로 끔)")
                print("=" * 60)
                print("✓ 팬 정지됨")
                print("✓ 서보 90도 복귀 중")
                print("=" * 60 + "\n")
                current_state = 'STOPPED'
                current_angle = CENTER_ANGLE
                last_direction = 'none'
                continue
                
            elif atmega_status == STATUS_READY:
                # 비정상 상태 (작동 중인데 READY는 이상함)
                print(f"⚠ 상태 불일치: RPi={current_state}, ATmega=READY({atmega_status})")
                
            else:
                print(f"⚠ 알 수 없는 ATmega 응답: {atmega_status}")
                
        except Exception as e:
            print(f"SPI 통신 오류: {e}")

        # 화면 표시
        overlay = frame.copy()
        alpha = 0.2
        dz_start_px = int((FRAME_WIDTH / 2) - (FRAME_WIDTH * DEAD_ZONE_PERCENT / 2))
        dz_end_px = int((FRAME_WIDTH / 2) + (FRAME_WIDTH * DEAD_ZONE_PERCENT / 2))
        cv2.rectangle(overlay, (dz_start_px, 0), (dz_end_px, FRAME_HEIGHT), (255, 255, 0), -1)
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

        # FPS 계산
        fps = (frame_count := frame_count + 1) / (time.time() - start_time) if time.time() > start_time else 0
        
        # 상태별 색상
        state_colors = {
            'IDLE': (0, 255, 255),
            'TRACKING': (0, 255, 0),
            'SEARCHING': (0, 165, 255),
            'WAITING': (0, 0, 255)
        }
        state_color = state_colors.get(current_state, (255, 255, 255))
        
        # 정보 표시
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(frame, f"Angle: {current_angle}", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(frame, f"State: {current_state}", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)
        
        # WAITING 상태일 때 카운트다운 표시
        if current_state == 'WAITING':
            remaining = RESET_TIMEOUT - (time.time() - wait_start_time)
            cv2.putText(frame, f"Reset: {remaining:.1f}s", (20, 145), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 사람 감지 시 박스 표시
        if person_detected:
            person_info = max(detected_persons, key=lambda p: p['area'])
            box = person_info['box']
            center_x = int(person_info['center_x'])
            center_y = int(box[1] + box[3] / 2)
            
            cv2.rectangle(frame, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 2)
            cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 2)

        cv2.imshow("Smart Fan Controller", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("\n" + "=" * 60)
    print("프로그램 종료 중...")
    cap.release()
    cv2.destroyAllWindows()
    try:
        print("최종 리셋 명령 전송...")
        spi.xfer2([RESET_COMMAND])
        time.sleep(0.2)
        spi.close()
        print("✓ 종료 완료!")
    except Exception as e:
        print(f"✗ 종료 중 오류: {e}")
    print("=" * 60)
