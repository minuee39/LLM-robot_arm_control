# pallet_gripper 관성·좌표 검증

작성일: 2026-09-18

## 결론

`urdf/pallet_gripper.urdf`의 10개 관성행렬은 모두 대칭 양의 정부호이며 주관성모멘트 삼각조건을 만족한다. 질량중심은 모두 해당 STL의 축 정렬 경계 안에 있다. 현재 COM과 관성 텐서는 `source/urdf/어셈블리1.SLDASM.urdf`의 SolidWorks Y-up 값을 `Rx(+pi/2)`로 Z-up 변환한 결과와 수치 오차 범위에서 일치한다. 따라서 관성곱을 0으로 만들거나 좌표를 임의 이동하지 않았다.

검증된 변환은 다음과 같다.

```text
c_zup = Q c_yup
I_zup = Q I_yup Q^T
Q = Rx(+pi/2)
```

URDF의 `inertial/origin rpy`는 모두 0이며, 텐서 자체가 링크의 Z-up 축으로 회전되어 있으므로 일관된다. 비대각 관성곱은 오류가 아니라 링크 축이 주관성축과 일치하지 않아 생기는 정상적인 항이다.

## 수정 사항

- 변환된 base STL의 최저점 `z=-0.0042247553356 m`에 맞춰 `base_footprint_joint`를 `+0.0042247553356 m` 올렸다. 이제 최저점은 base_footprint의 z=0과 일치한다.
- 이미 Z-up으로 bake된 URDF에 launch가 world→base_link `Rx(+pi/2)`를 다시 추가하면서 base_link에 두 부모를 만들던 중복 변환을 제거했다.
- standalone RViz Fixed Frame을 `world`에서 실제 URDF 루트인 `base_footprint`로 변경했다. MoveIt의 별도 world virtual joint는 변경하지 않았다.
- fixed joint `J7`에 의미 없이 남아 있던 zero axis와 zero limit을 제거했다.
- `tools/validate_inertials.py`를 추가해 원본 변환, 관성 물리조건, COM 경계, 관절축과 바닥 높이를 재검증할 수 있게 했다.
- J8/J9 zero-position STL 끝단의 중앙을 계산해 J7 기준 `tcp`를 `(0, 0, 0.094545039) m`에 추가했다. TCP 축은 J7과 같고 `+Z`가 접근 방향이다.

## 주의사항

이 검사는 URDF가 보관된 SolidWorks export를 올바르게 좌표 변환했는지와 기본 물리조건을 검사한다. CAD의 재질, 누락 부품, 실제 배선·볼트와 측정 질량이 정확한지는 증명하지 않는다. 새로운 `그리퍼 포함 질량수정본`의 값을 이 모델에 섞지 않았다. 그 export는 별도 좌표계이며 일부 관성값에 물리조건 위반이 있으므로 CAD에서 다시 계산하기 전에는 정확한 대체값으로 사용할 수 없다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
python3 src/pallet_gripper/tools/validate_inertials.py
check_urdf src/pallet_gripper/urdf/pallet_gripper.urdf
```
