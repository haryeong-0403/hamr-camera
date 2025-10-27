#!/usr/bin/env python3
import cv2

# /dev/video2 열기
cap2 = cv2.VideoCapture(2, cv2.CAP_V4L2)

# 해상도, 포맷, fps 강제 지정
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 1536)
cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'UYVY'))
cap2.set(cv2.CAP_PROP_FPS, 30)

if not cap2.isOpened():
    print("❌ 카메라 열기 실패 (/dev/video2)")
    exit()

print("✅ 카메라 열림, 스트리밍 시작")

while True:
    ret, frame = cap2.read()
    if not ret:
        print("⚠️ 프레임 읽기 실패")
        continue

    # 화면에 출력
    cv2.imshow("Camera2", frame)

    # q 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap2.release()
cv2.destroyAllWindows()
