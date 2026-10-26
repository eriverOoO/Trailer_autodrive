# 조향 진단 (2026-10-24)

- 키보드 송신 로그에서 조향 -7..+4 및 속도 0 조향 명령을 확인했다.
- 실제 Arduino UART에 속도 0으로 중앙 → 좌 -7 (1초) → 우 +7 (1초) → 중앙을 전송했다. 보드의 응답·자세 피드백은 없으며 기계적 움직임은 사용자 확인이 필요하다.
- USB 보드는 Arduino Mega, 부트로더 signature는 0x1e9801이다.
- avrdude의 flash READ만 사용해 기존 보드 프로그램을 flash_before.bin으로 백업했다. FLASH WRITE는 수행하지 않았다.
- 기존 소스(STEERING_SPEED=256)를 Mega로 컴파일한 프로그램 3956 bytes가 보드 플래시 시작 영역과 byte-for-byte 일치했다. firmware_comparison.json 참조.
- 설치된 Arduino AVR core에서 Mega pin 2/3은 TIMER3B/C이고 Timer3은 8-bit phase-correct PWM이다. analogWrite는 256을 255로 제한하지 않고 OCR3B/C에 그대로 쓴다. 256은 PWM TOP=255를 넘는 잘못된 비교값이다.
- 원본 ros2_parking의 STEERING_SPEED는 128이다. 현재 소스의 값만 128로 수정하고 컴파일 시 범위 검사를 추가했다. A5, 330/404, 주행 모터 핀 등 현장 설정은 유지했다.
- 수정본 Mega 빌드 통과: build_fixed/driving.ino.hex. 아직 업로드하지 않았으며 실제 수정 후 좌우 동작도 미검증이다.

참고 소스: 로컬 Arduino 1.8.19의 hardware/arduino/avr/cores/arduino/wiring.c, wiring_analog.c, variants/mega/pins_arduino.h.
