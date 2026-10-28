# RobStride CAN 기본 통신 명령 모음

작성일: 2026-10-05  
목적: Linux SocketCAN에서 RobStride 모터의 연결 확인, ID 검색·변경, 상태 읽기, 정지·활성화, PP 각도 제어와 속도 제어를 안전한 순서로 수행한다.

## 적용 범위

이 문서는 RobStride **사설 프로토콜(private protocol)** 기준이다.

- CAN 2.0B, 29-bit extended frame, DLC 8
- 기본 비트레이트: `1,000,000 bit/s`
- 기본 호스트 ID: `0xFD`
- 예시 모터 ID: `1`
- 위치 단위: `rad`, 속도 단위: `rad/s`, 가속도 단위: `rad/s²`, 전류 단위: `A`
- `run_mode`: `0` 운전 제어, `1` PP 위치, `2` 속도, `3` 전류, `5` CSP 위치

CANopen 또는 11-bit MIT 프로토콜로 변경된 모터에는 아래 프레임을 보내면 안 된다. 프로토콜 변경은 전원 재인가 후 적용되므로, 현재 프로토콜이 불명확하면 제조사 프로그램에서 먼저 확인한다.

## 안전 전제

1. 모터와 링크를 지지하고 작업 반경을 비운다. 물리 E-stop을 즉시 누를 수 있어야 한다.
2. CAN 전원을 끈 상태에서 CAN-H와 CAN-L 사이 종단 저항을 확인한다. 버스 양 끝에 120 Ω씩이면 전체 측정값은 보통 약 60 Ω이다.
3. CAN-H, CAN-L, CAN-GND를 올바르게 연결하고 모터 전원 전압을 모델 사양과 맞춘다.
4. 모드 변경은 반드시 모터 정지(Type 4) 상태에서 한다.
5. 위치 모드에서는 현재 위치를 목표값으로 먼저 기록한 다음 enable해야 갑작스러운 이동을 피할 수 있다.
6. 속도 모드 종료 시 목표 속도를 `0`으로 먼저 내리고 모터를 stop한다.
7. `ip link ... down`은 통신 인터페이스만 내린다. 모터를 정지시키지 않으므로 반드시 모터 stop을 먼저 보낸다.

## 1. 작업 위치와 공통 값

```bash
cd /home/minwoo/Development/LLM

export RS_CAN_IF=can0
export RS_CAN_BITRATE=1000000
export RS_HOST_ID=0xFD
```

사용할 수 있는 CAN 인터페이스를 확인한다.

```bash
ip -brief link
```

`can0`가 없다면 USB-CAN 어댑터 인식과 드라이버부터 확인한다.

## 2. CAN 통신 시작

프로젝트에 포함된 시작 스크립트를 사용하는 방법이 가장 간단하다.

```bash
PALLET_CAN_INTERFACE="$RS_CAN_IF" \
PALLET_CAN_BITRATE="$RS_CAN_BITRATE" \
./ros2_ur_ws/src/pallet/scripts/pallet_can.sh
```

같은 작업을 직접 실행하려면 다음과 같다.

```bash
sudo ip link set dev "$RS_CAN_IF" down
sudo ip link set dev "$RS_CAN_IF" type can bitrate "$RS_CAN_BITRATE"
sudo ip link set dev "$RS_CAN_IF" up
ip -details -statistics link show "$RS_CAN_IF"
```

정상 상태에서는 `state ERROR-ACTIVE`가 보인다. `BUS-OFF`, error counter 증가 또는 dropped packet이 보이면 배선, 종단 저항, 비트레이트, 전원을 확인한다.

## 3. 수신 모니터링과 진단

터미널 하나에서 계속 실행한다.

```bash
candump -tz -e "$RS_CAN_IF"
```

통계와 상세 상태를 다시 확인한다.

```bash
ip -details -statistics link show "$RS_CAN_IF"
```

BUS-OFF 원인을 해결한 뒤 수동 복구한다.

```bash
sudo ip link set dev "$RS_CAN_IF" type can restart
```

## 4. RobStride ID 찾기

이 저장소의 검색 도구는 Type 0 요청을 각 ID에 보내고, 모터 ID와 64-bit MCU UID를 확인한다. 전체 범위 검색은 기본 timeout 20 ms 기준 약 5초가 걸린다.

```bash
python3 ros2_ur_ws/src/pallet/scripts/robstride_can_id.py scan \
  --interface "$RS_CAN_IF" \
  --start-id 1 --end-id 255 \
  --timeout-ms 20
```

예상 범위만 검색하면 더 빠르다.

```bash
python3 ros2_ur_ws/src/pallet/scripts/robstride_can_id.py scan \
  --interface "$RS_CAN_IF" \
  --start-id 1 --end-id 20 \
  --timeout-ms 50
```

JSON 출력:

```bash
python3 ros2_ur_ws/src/pallet/scripts/robstride_can_id.py scan \
  --interface "$RS_CAN_IF" \
  --start-id 1 --end-id 20 \
  --timeout-ms 50 --json
```

모터 ID 1 하나만 직접 조회하는 raw frame은 다음과 같다.

```bash
cansend "$RS_CAN_IF" 0000FD01#0000000000000000
```

ID 1 모터의 정상 응답 ID는 보통 `000001FE`이며 8-byte data는 MCU UID이다. `candump`에 아무 응답도 없으면 ID뿐 아니라 프로토콜, 1 Mbit/s 비트레이트, 전원과 종단 저항도 함께 확인한다.

## 5. RobStride ID 변경

ID가 같은 모터가 여러 개면 **한 대씩만 CAN 버스에 연결하여 변경**한다. 새 ID가 이미 사용 중인지 먼저 검색해야 한다.

ID 1을 11로 변경:

```bash
python3 ros2_ur_ws/src/pallet/scripts/robstride_can_id.py set-id \
  --interface "$RS_CAN_IF" \
  --timeout-ms 50 \
  1 11
```

도구는 다음 순서로 동작한다.

1. 기존 ID의 UID 확인
2. 새 ID 충돌 확인
3. UID를 포함한 Type 7 변경 명령 송신
4. 새 ID와 동일 UID의 응답 확인

일부 구형 펌웨어에서 UID payload 방식이 동작하지 않을 때만 다음 fallback을 사용한다.

```bash
python3 ros2_ur_ws/src/pallet/scripts/robstride_can_id.py set-id \
  --interface "$RS_CAN_IF" \
  --timeout-ms 50 \
  --zero-payload \
  1 11
```

수동 Type 7 프레임의 arbitration ID 예시는 `1 → 11`일 때 `070BFD01`이다. 펌웨어에 따라 UID payload가 필요하고 충돌 검사를 생략하게 되므로 raw `cansend`보다 위 도구를 사용한다.

## 6. 현재 각도 읽기

읽기만 수행하며 모터를 enable하지 않는다.

```bash
python3 robstride_read_angles.py --channel "$RS_CAN_IF" --motor 1 --once
```

여러 모터:

```bash
python3 robstride_read_angles.py \
  --channel "$RS_CAN_IF" \
  --motor 1 2 3 4 5 6 7 \
  --once
```

10 Hz 연속 출력:

```bash
python3 robstride_read_angles.py \
  --channel "$RS_CAN_IF" \
  --motor 1 2 3 4 5 \
  --rate 10
```

모터 ID 1의 기계각 `0x7019`를 직접 읽는 raw frame:

```bash
cansend "$RS_CAN_IF" 1100FD01#1970000000000000
```

Type 17 응답 data의 Byte 4~7은 little-endian `float32` rad 값이다. 일반 점검에는 직접 디코딩하지 말고 `robstride_read_angles.py`를 사용한다.

## 7. raw frame 송신용 Bash 보조 함수

아래 함수는 이 터미널 세션에서만 유지된다. `rs_write_f32`는 parameter index와 값을 RobStride 형식인 little-endian payload로 변환한다.

```bash
rs_frame() {
  local comm_type="$1"
  local motor_id="$2"
  local payload="${3:-0000000000000000}"
  local can_id
  printf -v can_id '%02X%04X%02X' "$((comm_type))" "$((RS_HOST_ID))" "$((motor_id))"
  cansend "$RS_CAN_IF" "${can_id}#${payload}"
}

rs_write_u8() {
  local motor_id="$1"
  local index="$2"
  local value="$3"
  local can_id payload
  printf -v can_id '%02X%04X%02X' 0x12 "$((RS_HOST_ID))" "$((motor_id))"
  payload="$(python3 -c 'import struct,sys; i=int(sys.argv[1],0); v=int(sys.argv[2],0); print((struct.pack("<H",i)+b"\0\0"+bytes((v,0,0,0))).hex().upper())' "$index" "$value")"
  cansend "$RS_CAN_IF" "${can_id}#${payload}"
}

rs_write_f32() {
  local motor_id="$1"
  local index="$2"
  local value="$3"
  local can_id payload
  printf -v can_id '%02X%04X%02X' 0x12 "$((RS_HOST_ID))" "$((motor_id))"
  payload="$(python3 -c 'import struct,sys; i=int(sys.argv[1],0); v=float(sys.argv[2]); print((struct.pack("<H",i)+b"\0\0"+struct.pack("<f",v)).hex().upper())' "$index" "$value")"
  cansend "$RS_CAN_IF" "${can_id}#${payload}"
}

rs_enable() {
  rs_frame 0x03 "$1"
}

rs_stop() {
  rs_frame 0x04 "$1"
}

rs_clear_fault_and_stop() {
  rs_frame 0x04 "$1" 0100000000000000
}
```

함수 확인용 예시다. 아래 두 명령은 실제 송신하므로 모터가 움직일 수 있는 작업 단계에서는 주의한다.

```bash
rs_stop 1
rs_enable 1
```

## 8. PP 각도 제어

`run_mode=1`은 모터 내부 프로파일을 사용하는 절대 위치 제어다. 목표 `loc_ref`는 **모터 좌표계의 rad**이며 로봇 관절 각도가 아니다. 감속기, 설치 방향 및 영점 offset이 있으면 다음 변환이 필요하다.

```text
motor_target_rad = motor_zero_rad + direction × joint_target_rad × transmission_ratio
```

예시는 모터 ID 1, 전류 제한 1 A, 최대 속도 0.2 rad/s, 가속도 0.5 rad/s²이다. 먼저 `robstride_read_angles.py`로 읽은 현재 모터 각도를 `CURRENT_POS_RAD`에 넣는다.

```bash
MOTOR_ID=1
CURRENT_POS_RAD=0.0   # 반드시 실측값으로 바꾼다
TARGET_POS_RAD=0.1    # 현재 위치에서 아주 가까운 값으로 시작한다

rs_stop "$MOTOR_ID"
rs_write_u8 "$MOTOR_ID" 0x7005 1
rs_write_f32 "$MOTOR_ID" 0x7018 1.0
rs_write_f32 "$MOTOR_ID" 0x7024 0.2
rs_write_f32 "$MOTOR_ID" 0x7025 0.5
rs_write_f32 "$MOTOR_ID" 0x7016 "$CURRENT_POS_RAD"
rs_enable "$MOTOR_ID"
rs_write_f32 "$MOTOR_ID" 0x7016 "$TARGET_POS_RAD"
```

정지:

```bash
rs_stop "$MOTOR_ID"
```

`CURRENT_POS_RAD=0.0`을 그대로 복사하지 않는다. 실제 현재 위치와 다르면 enable 순간 점프할 수 있다. 프로젝트의 실기 브리지는 이 문제를 막기 위해 실측 위치를 먼저 `LOC_REF`에 기록한다.

## 9. 속도 제어

`run_mode=2`는 속도 제어다. 예시는 모터 ID 1, 전류 제한 1 A, 가속도 0.5 rad/s², 목표 속도 0.2 rad/s이다.

```bash
MOTOR_ID=1

rs_stop "$MOTOR_ID"
rs_write_u8 "$MOTOR_ID" 0x7005 2
rs_write_f32 "$MOTOR_ID" 0x7018 1.0
rs_write_f32 "$MOTOR_ID" 0x7022 0.5
rs_write_f32 "$MOTOR_ID" 0x700A 0.0
rs_enable "$MOTOR_ID"
rs_write_f32 "$MOTOR_ID" 0x700A 0.2
```

반대 방향은 음수 속도를 사용한다.

```bash
rs_write_f32 "$MOTOR_ID" 0x700A -0.2
```

속도 제어 종료는 0 rad/s 명령 후 stop 순서로 한다.

```bash
rs_write_f32 "$MOTOR_ID" 0x700A 0.0
rs_stop "$MOTOR_ID"
```

축 끝단이 있는 관절에서 속도 모드를 방치하면 기구 한계에 충돌한다. 짧은 시간과 낮은 속도로만 방향을 확인한다.

## 10. 기계 영점 설정

현재 위치를 모터의 기계 영점으로 바꾸는 명령이다. 기존 모든 위치 기준이 바뀌므로 ID 변경보다 더 신중하게 사용한다.

```bash
MOTOR_ID=1
rs_stop "$MOTOR_ID"
rs_frame 0x06 "$MOTOR_ID" 0100000000000000
python3 robstride_read_angles.py --channel "$RS_CAN_IF" --motor "$MOTOR_ID" --once
```

PP 모드에서는 영점 설정이 제한될 수 있다. 모터를 stop하고 CSP 또는 운전 제어 모드 조건에서 제조사 매뉴얼의 해당 펌웨어 절차를 따른다.

## 11. 모터 정지와 CAN 통신 종료

알고 있는 모든 모터를 먼저 stop한다.

```bash
for MOTOR_ID in 1 2 3 4 5 6 7; do
  rs_stop "$MOTOR_ID"
done
```

그 다음 CAN 인터페이스를 내린다.

```bash
sudo ip link set dev "$RS_CAN_IF" down
ip -brief link show "$RS_CAN_IF"
```

통신 프로세스를 `Ctrl+C`로 끝내거나 CAN 인터페이스만 down하는 것은 모터 stop 명령을 대신하지 않는다.

## 12. 자주 쓰는 파라미터와 통신 타입

| 기능 | 값 | 의미 |
|---|---:|---|
| Type 0 | `0x00` | 장치 ID와 MCU UID 조회 |
| Type 3 | `0x03` | 모터 enable |
| Type 4 | `0x04` | 모터 stop; Byte 0=`1`이면 fault clear 포함 |
| Type 6 | `0x06` | 현재 위치를 기계 영점으로 설정 |
| Type 7 | `0x07` | 모터 CAN ID 변경 |
| Type 17 | `0x11` | 단일 parameter 읽기 |
| Type 18 | `0x12` | 단일 parameter 쓰기 |
| `run_mode` | `0x7005` | `0` 운전 제어, `1` PP, `2` 속도, `3` 전류, `5` CSP |
| `iq_ref` | `0x7006` | 전류 모드 목표 전류 A |
| `spd_ref` | `0x700A` | 속도 모드 목표 속도 rad/s |
| `loc_ref` | `0x7016` | 위치 목표 rad |
| `limit_spd` | `0x7017` | CSP 위치 모드 속도 제한 rad/s |
| `limit_cur` | `0x7018` | 위치·속도 모드 전류 제한 A |
| `mechPos` | `0x7019` | 기계각 피드백 rad |
| `mechVel` | `0x701B` | 기계 속도 피드백 rad/s |
| `acc_rad` | `0x7022` | 속도 모드 가속도 rad/s² |
| `vel_max` | `0x7024` | PP 위치 모드 최대 속도 rad/s |
| `acc_set` | `0x7025` | PP 위치 모드 가속도 rad/s² |

일반 host→motor arbitration ID 형식:

```text
bits 28..24: communication type
bits 23..8 : data area 2 (일반 명령에서는 host ID 0x00FD)
bits 7..0  : target motor ID

CAN_ID = (communication_type << 24) | (host_id << 8) | motor_id
```

Type 18 parameter payload 형식:

```text
Byte 0..1: parameter index, little-endian
Byte 2..3: 0
Byte 4..7: 값, little-endian uint/float32
```

## 13. 문제별 점검

### 응답이 전혀 없음

- `ip -details link show can0`에서 인터페이스가 UP인지 확인
- 모터 기본값과 같은 1 Mbit/s인지 확인
- CAN-H/L 뒤바뀜, CAN-GND 누락, 전원과 종단 저항 확인
- private/CANopen/MIT 중 현재 프로토콜 확인
- 실제 ID 범위를 넓혀 Type 0 scan

### `BUS-OFF` 또는 RX/TX error 증가

- 즉시 송신을 중지하고 인터페이스를 down
- 비트레이트와 배선·종단 저항을 수정
- 원인 수정 뒤 인터페이스를 다시 up

### enable은 되지만 움직이지 않음

- 올바른 `run_mode`인지 확인
- PP: `vel_max`, `acc_set`, `limit_cur`, `loc_ref` 확인
- 속도: `limit_cur`, `acc_rad`, `spd_ref` 확인
- fault frame과 전원 전압 확인

### 반대 방향으로 움직임

- 속도 부호 또는 `direction` 설정을 반대로 지정
- 로봇 관절 좌표를 그대로 raw 모터 좌표로 보내지 말고 영점·방향·감속비를 적용

## 14. 검증 체크리스트

- [ ] CAN 어댑터와 `can0` 인식
- [ ] `ERROR-ACTIVE`, RX/TX error 증가 없음
- [ ] Type 0 scan으로 ID와 UID 확인
- [ ] ID 중복 없음
- [ ] enable하지 않은 상태에서 각도 읽기 성공
- [ ] E-stop과 하중 지지 준비
- [ ] 모터 하나씩 낮은 전류·속도로 방향 확인
- [ ] PP enable 전 실측 현재 위치를 `loc_ref`로 기록
- [ ] 속도 종료 시 `spd_ref=0` 후 stop
- [ ] 모든 모터 stop 후 CAN 인터페이스 down

## 참고 자료

- [RobStride 공식 Python SDK](https://github.com/RobStride/Python_Sample)
- [RobStride 공식 protocol 상수와 parameter index](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py)
- [RobStride 공식 제품 자료와 모델별 매뉴얼](https://github.com/RobStride/Product_Information)
- [RS06 User Manual](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS06/RS06User%20Manual251112.pdf)

