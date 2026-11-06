# RobStride MoveIt 궤적 추종 지연 관련 조사

작성일: 2026-10-27. 이번 작업은 조사이며 모터 설정/구동 변경 없음.

## 직접 관련된 근거

1. RobStride 공식 Python_Sample examples/position_control.py
   https://github.com/RobStride/Python_Sample/blob/main/examples/position_control.py
   https://github.com/RobStride/Python_Sample
   실제 raw 소스도 열어 확인했다. PP run_mode1: 드라이브 내부 trapezoidal profile,
   이산 go-to-position. CSP run_mode5: host trajectory planner를 위한 고속 위치 streaming.
   예제 CSP100Hz, 속도제한은limit_spd로1회설정 후loc_ref 갱신. 예제 모델RS01이므로
   샘플 수치나 모델을 현재RS00/05/06에 그대로 적용하지 않는다.

2. pollen-robotics/jerem_arm의 README 및 sine.py
   https://github.com/pollen-robotics/jerem_arm
   https://github.com/pollen-robotics/jerem_arm/blob/main/sine.py
   RS05 두축/PEAK PCAN사용. README는PP내부 trapezoidal generator가streamed targets를
   filter한다고명시하고CSP권장. sine.py는CSP5/기본50Hz. 실제작성자코드에근거한사례이며
   제조사성능보장이나우리6축USB환경검증은아니다. 기본진폭/게인/종료동작은복사하지않는다.

3. Morten Lind, Real-time quintic Hermite interpolation for robot trajectory execution,
   PeerJ Computer Science6:e304(2020), DOI10.7717/peerj-cs.304
   https://pmc.ncbi.nlm.nih.gov/articles/PMC7924709/
   저속상위응용과고속하위제어사이에보간서비스를배치.위치/속도/가속도를활용하는quintic
   Hermite spline과추종지연분석.실험은Franka Panda이며RobStride검증논문은아니다.
   적용아이디어는상위계획과하위주기분리/부드러운시간보간이며100Hz수치를보장하지않는다.

4. s2015-turtle/robstride_ros2
   https://github.com/s2015-turtle/robstride_ros2
   비공식community ros2_control Hardware Component.다축단일CAN,비동기전송/피드백진단,
   watchdog/종료확인/command coalescing이참고할구조다. 실제README의position interface는
   position/Kp/Kd운동명령이므로CSP드라이버와동일시하지않는다. can_msgs topictransport이므로
   현재USB AT SerialTransport에그대로대체할수있는것도아니다.설치/실기검증하지않았다.

5. Zhang et al., Experimental assessment and feedforward control of backlash and stiction
   in industrial serial robots for low-speed operations, IJCIM36(3),393–410(2023),
   DOI10.1080/0951192X.2022.2090609
   https://kclpure.kcl.ac.uk/portal/en/publications/experimental-assessment-and-feedforward-control-of-backlash-and-s/
   방향반전의backlash/stiction을실험식별하고feedforward보상.우리J5에서해당원인을확인한것은
   아니므로모드/통신분리후남은반전오차에대한후속진단근거로사용한다.

## 우리 시스템에 대한 판단과 제안

현재PP1에MoveIt보간목표를10Hz로전송한다. 공식CSP용도와비교하면PP추가프로파일이
지연에기여할가능성이있다. 이는가설이며소규모fixed/stream비교는둘다통과했다.
확정원인이라고하지않는다.추종허용값증가나전류증가보다다음순서를권한다.

- J5단축에서동일한진폭/파형/갱신주기의PP와CSP를비교해모드효과분리.
- 위치제어CSP에서도watchdog/초기현재각목표/속도제한/STOP확인필요.중력보상/RL불필요.
- 6축고속명령갱신과느린전류등상태수집을분리.수신비동기화/active feedback검토.
- 가능한전송률을측정한뒤단계적으로갱신주기높임.현재3ms TX간격이면6개위치쓰기만
  최소18ms.18개read까지동기처리하면총24TX에약72ms+처리시간이필요해실측약77ms와
  부합한다.따라서현재구현에period10ms/100Hz만설정하면안된다.
- 제어기내부보간/버퍼링으로위치/속도/가속도연속성개선.ros2_control/JTC구조는중장기후보.
- 도달후최종목표유지통합은별도과제.정상도달과오류/취소/종료정책분리필요.
- 모드/주기개선후에도오차가남으면load별gain응답/마찰/백래시식별.무조건gain상향금지.

## 남은 검증

CSP에서현재RS00/05/06 firmware의watchdog/속도제한과실기추종은미검증.
공식예제100Hz는단일RS01/SocketCAN환경수치로현재6축CH340 USB에서의달성근거가아니다.
추천다음작업은J5 PP/CSP동일조건비교코드작성과제한된소규모실기검증이다.
