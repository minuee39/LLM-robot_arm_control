# Codex Working Guide

## Project
Pallet VLA Robot Arm Project

## Goal
사용자의 자연어 명령을 LLM이 JSON으로 변환하고, 로봇팔이 Isaac Sim / MoveIt2 / 실제 하드웨어에서 pick-place 동작을 수행하는 VLA 기반 로봇팔 시스템을 개발한다.

## Current Pipeline
User Command
→ LLM Parser
→ JSON Command
→ Command Validator
→ Target Position Resolver
→ Motion Planner / Controller
→ Robot Arm Execution

## Current Implementation
- LLM to JSON 구조 구현
- Gemini 기반 명령 파싱
- Gemini 실패 시 local parser fallback
- Isaac Sim UR10e pick-place 기반 시뮬레이션
- RMPflow 기반 제어
- 명령 반복 입력 루프 구현
- object position과 relation offset 기반 목표 위치 계산
- YOLO 비전 인식 준비 단계
- MoveIt2 전환 검토 단계

## Important Files
- command_parser.py: 자연어 명령을 JSON으로 변환
- llm_to_json.py: LLM API 호출 및 응답 처리
- scene_config.py: 물체 이름, 위치, 관계 offset 정의
- pick_place_example.py: Isaac Sim 실행 진입점
- tasks/pick_place.py: pick-place task 구성
- controllers/pick_place_controller.py: 로봇 동작 제어
- test_llm_to_json.py: 명령 파서 테스트

## Codex Priorities
1. command_parser.py 개선
2. command memory 기능 추가
3. scene_config.py의 object state 업데이트 구조 개선
4. YOLO detection module 분리
5. Isaac Sim 실행 코드와 실제 로봇 제어 코드 분리
6. pytest 테스트 추가
7. README.md와 AGENTS.md 작성

## Change Logging
- 사용자가 명시적으로 다시 허용하기 전까지 Notion `Pallet VLA Project Hub`를 포함한 어떤 Notion 페이지에도 내용을 작성, 추가, 수정하거나 기록하지 않는다.
- 작업 완료 보고와 변경 로그도 Notion에 남기지 않으며, 이 제한을 해제하려면 사용자의 명시적인 요청이 필요하다.
- 기술 작업 기록을 작성할 때는 `UR10e + Robotiq 2F-140 MoveIt2 / Isaac Sim 작업 기록` 페이지처럼 작성일과 목적, 기준 workspace, 주요 패키지/파일, 문제와 원인, 해결 과정, 최종 구조, 실행 순서, 검증 명령과 결과, 남은 주의사항, 체크리스트 형태의 다음 작업 로드맵 순서로 정리한다.
- 관련 경로, 설정값, 실행 명령, 핵심 로그를 구체적으로 포함하되 실제로 확인하지 않은 결과나 자료는 작성하지 않는다.
