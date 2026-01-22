# Robotics Study - Python 소스코드

## 📁 파일 구조

```
Robotics_Study_python/
├── extension.py      # 확장 진입점 (Extension 클래스)
├── ui_builder.py     # 메인 UI 구성
├── scenario.py       # Chapter2 시나리오 로직
├── global_variables.py   # 확장 메타데이터 (제목, 설명 등)
└── assignments/      # Assignment별 모듈
    └── assignment1/
        ├── scenario.py   # Assignment1 로직
        └── ui.py         # Assignment1 UI
```

---

## 🔧 각 파일 역할

### `extension.py`
- Isaac Sim 툴바에 확장 등록
- 메뉴 버튼 클릭 시 UI 창 생성
- 타임라인/스테이지 이벤트 구독

### `ui_builder.py`
- **World Controls**: LOAD, RESET 버튼
- **Run Scenario**: RUN/STOP 토글
- **Robot Information**: 조인트 상태 실시간 표시
- **Chapter2 Example**: 7DOF, 2DOF 예제 버튼

### `scenario.py`
- `RoboticsStudyScenario` 클래스
- 7DOF 사인파 궤적 생성 및 End-effector 시각화
- 2DOF Prismatic 로봇 제어

### `assignments/assignment1/`
- `scenario.py`: 포즈 정의, FK 계산, 애니메이션 로직
- `ui.py`: Assignment1 전용 UI 구성

---

## 🔄 실행 흐름

```
1. extension.py → UI 창 생성
2. ui_builder.py → LOAD 버튼 클릭
3. scenario.py → 로봇 초기화 (setup_scenario)
4. RUN 버튼 → 매 프레임 update_scenario() 호출
5. RESET → teardown_scenario() 호출
```

---

## ➕ 새 Assignment 추가하기

```bash
# 1. 폴더 생성
mkdir -p assignments/assignment2

# 2. 파일 생성
touch assignments/assignment2/__init__.py
touch assignments/assignment2/scenario.py
touch assignments/assignment2/ui.py

# 3. assignments/__init__.py에 import 추가
# 4. ui_builder.py에서 Assignment2UI 사용
```
