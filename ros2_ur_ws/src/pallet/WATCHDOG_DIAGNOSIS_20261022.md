# ID6 RS05 watchdog 점검

2026-10-22. 조사 중 ENABLE/비영점 전류/펌웨어 변경 명령은 전송하지 않았다. 자동 보고 켜기/끄기, 레지스터 읽기, type4 특수 버전조회(정지 계열)를 사용했다.

확인 사실:
- 버전 응답: type4 00 C4 질의 -> response 0x020006FD, payload 00 C4 56 00 05 00 09 07. 매뉴얼 Byte3..6 기준 앱 버전은 0.5.0.9.
- 0x7005=3, 0x7006=0, 0x7028=5000. 응답 ID 0x110006FD, 읽기 성공.
- 0x1000/1003/1004/1005/1006/200C/301C/3022는 응답 ID 0x110106FD, 읽기 오류(status=1). payload 0을 실제 0값으로 해석하면 안 된다. 읽기 오류 필드를 검증하도록 코드 수정.
- 정지 상태 자동 보고는 type24(0x180006FD)로 실제 수신됐다. type2만 기다리던 코드는 이미 수정했다.
- 사용자가 수행한 0A 활성화/수신전용 600ms 시험에서 마지막 상태는 Run(2). watchdog 자동 Reset은 아직 검증되지 않았다.

공식 자료:
https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py : CAN_TIMEOUT=0x7028 uint32.
https://github.com/RobStride/Product_Information/releases/tag/V26.05.19 : RS05 0.5.0.14 제공.
공식 변경 이력 default.pdf의 RS05 0.5.0.13은 CANopen watchdog 0x6099:1 추가를 명시하지만 현재 사용 중인 private-CAN 0x7028의 이 증상 수정 여부는 명시하지 않는다. 업데이트하면 해결된다고 단정하지 않는다.

판정: 현재 펌웨어/어댑터/프로토콜 조합에서 250ms timeout에 기대한 Reset은 확인되지 않음. 읽기 가능한 5000 설정값만으로 정지 보장 불가. 제조사에 위 버전/raw 응답/재현 절차로 동작 조건 확인 또는 지원 펌웨어 검증이 필요하다. 펌웨어를 임의 업데이트하지 않았으며, commissioned=false 및 motor_watchdog_verified=false 유지. 실제 전류 펄스/무지지 중력보상은 진행하지 않는다.

후속 진행: 자동 보고 on/off 비교 시험을 시도했으나 watchdog 설정 0x7028 읽기 단계에서 timeout으로 활성화 이전 중단했다. 이후 6번 위치 읽기 2회 및 ID1..7 개별 위치 읽기가 모두 미응답이었다. USB 시리얼 포트 자체는 열리지만 모터 CAN 응답이 없어 공통 전원/버스/어댑터 상태 점검이 먼저 필요하다. 비교 시험의 ENABLE 또는 비영점 전류는 실행되지 않았다. 전원 재시작을 임의로 하지 않았으며, 전원 확인은 외부 지지 후 사용자가 해야 한다.

전원 재투입 후 추가 실기 비교:
- ID1..7 모두 위치 읽기 성공.
- ID6 STOP 후 보고 상태 Reset(0), fault=0 확인.
- 0A 활성화 후 자동 보고 on, 1.2초 TX 중단: 121개 상태 보고 모두 Run(2), Reset 없음.
- 별도 STOP 후 자동 보고 켜기만으로는 Reset(0) 유지(보고 켜기가 재활성화하지 않는 대조 확인).
- 다시 0A 활성화, 자동 보고 off, 0.8초 TX 중단 후 보고만 켜서 조회: Run(2). 조회가 타이머를 갱신할 가능성은 남지만 STOP 대조에서 보고 켜기는 활성화하지 않았다.
- 마지막 STOP 및 자동 보고 off 수행. 비영점 전류/펌웨어 변경 없음.
따라서 보고 on만으로 발생하는 현상이라는 가설은 지지되지 않는다. 0.5.0.9/current mode/0x7028=5000 조합의 watchdog 정지 상태 전환은 입증되지 않음. 비영점 전류에서는 다르게 동작하는지 알 수 없으며 그 검증을 위해 안전장치 없이 출력을 가하지 않는다. 최신 RS05 0.5.0.14는 후보지만 private-CAN watchdog 수정 명시가 없어 해결 보장 불가. 펌웨어 업데이트 또는 제조사 동작 조건 확인이 다음 결정사항이다.

추가 분리 시험: 5000값 유지, 0A/보고 on에서 6초 동안 송신 중단 관측: 600개 보고 모두 Run(2). 활성화 전후 0x7028=5000. 활성화 후 같은 값 재설정/읽기 확인 후에도 0.8초 동안 81개 보고 모두 Run(2). 마지막 STOP 수행. 짧은 관측창 및 활성화 시 초기화 가설은 지지되지 않음. 비영점 전류 없음. 공식 0.5.0.14는 후보 파일을 /tmp/pallet_rs05_firmware_review/에 준비했을 뿐 설치 안 했으며 해당 증상 수정은 보장할 수 없음.

## 承認後の更新結果 / 승인 후 업데이트 결과

사용자 명시 승인으로 ID6(UID 6d5a30222431310f)만 RS05 0.5.0.9→0.5.0.14 업데이트. 공식 OTA.py의 11/12/13/14 핸드셰이크를 AT 시리얼에 적용했으며, 93768 bytes / 11721 packets 전송 후 END ACK와 버전 응답 00c4560005000e07을 확인했다. 초기 START ACK 재시도 2회 후 전송 완료. 다른 ID에 OTA 쓰기를 보내지 않았다.

공식 release SHA256 일치: e05502e115a014029c79f5ad052b6c8b1e1790b557763c8a42da0dc35b6d79be. 읽을 수 있는 설정 부분 백업과 이벤트는 /home/minwoo/Development/firmware_tools/id6_before_0_5_0_14/. 이는 전체 공장 보정 백업은 아니다.

전후 위치: 0.40457505→0.40442660 rad (약0.00014845 rad 차이). zero_sta=1, add_offset=0 유지. mode는 재시작 후 0으로 초기화, iq_ref=0.

동일 watchdog 시험 재수행: 5000 ticks, Iq=0. Before Run(2)/fault0→송신 중단 약0.27392초에 Reset(0), 0.6초 관측 종료에도 Reset/fault0. PASS. 마지막 STOP 및 보고 off. 이번 기기에서는 firmware 업데이트 전후의 동작 차이가 실측됐으며 일반적인 모든 펌웨어/기기의 보장을 의미하지 않는다. 비영점 전류 시험 및 부하에서 실제 전류 차단 지연 측정은 수행하지 않았다. 검증 범위는 ID6의 0A 상태 전환만이며 전역 commissioned/motor_watchdog_verified는 false 유지.

ID5 조사: RS05 0.5.0.9, UID ce5238001473b708. 0A watchdog 5000 시험 후에도 Run(2), 끝에 STOP 수행. 비영점 전류 미실행. 설정 일부/버전/UID 백업은 /home/minwoo/Development/firmware_tools/id5_before_0_5_0_14/before.json. ID5 한정 OTA 도구 사전 점검만 완료했으며 펌웨어 변경은 사용자 승인 대기다.

사용자 승인 후 ID5만 0.5.0.9→0.5.0.14 OTA 완료. END ACK와 버전응답00c4560005000e07 확인. 전후 위치0.32203698→0.32193804rad, zero_sta=1/add_offset=0 유지. 0A watchdog 재검증에서 약0.27855초 후 Run2→Reset0, 관측 종료에도 Reset/fault0. 마지막 STOP. 비영점 전류 시험은 미수행. ID1..4 watchdog 미검증으로 전역 활성화는 계속false. 백업/로그는 firmware_tools/id5_before_0_5_0_14/.

사용자 승인으로 ID1..4 업데이트 완료. ID1/3 RS00:0.0.3.22→0.0.3.32, ID2 RS06:0.6.0.6→0.6.0.12, ID4 RS05:0.5.0.9→0.5.0.14. 모두 공식 release SHA256 일치, UID/모델 버전 확인, 설정 일부 백업, END ACK와 업데이트 버전 확인. 0A watchdog5000 재검증 Reset전환: ID1=0.27807s, ID2=0.28156s, ID3=0.27854s, ID4=0.27773s. 읽은zero_sta/add_offset 유지, 위치편차 최대0.000128rad. 모두 STOP/보고off. ID7 미변경. 팔 6축의 0A 상태전환 검증은 완료됐으나 부하전류차단/전체중력보상/ID7은 미검증이며 commissioned=false 유지. 상세로그는 firmware_tools/firmware_updates_ids1_4_summary.json과 축별 backup_folder.

ID7 승인 후 RS05 0.5.0.9→0.5.0.14 완료. UID064230222431310f 확인, 공식 SHA256 일치, OTA END ACK와 버전확인. 위치 -0.003018379rad 전후동일, iq_ref0/zero_sta1/add_offset0 유지. 0A watchdog 약0.27835초 후 Reset0, 종료상태fault0. 마지막STOP/보고off. 개폐·비영점전류·하드웨어영점설정은 하지 않았다. 이로써 ID1..7 0A watchdog 상태전환 확인 완료. 부하차단/전체중력보상은 미검증, commissioned=false, gripper_homed=false 유지. 백업/로그 firmware_tools/id7_before_0_5_0_14/.
