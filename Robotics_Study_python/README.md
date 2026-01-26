# Robotics Study - Python 소스코드

## 📁 파일 구조

```
Robotics_Study_python/
├── extension.py          # 확장 진입점 (Extension 클래스)
├── ui_builder.py         # 메인 UI 구성 및 모듈 통합
├── global_variables.py   # 확장 메타데이터 (제목, 설명 등)
├── tutorial/             # Tutorial 모듈 (Chapter2 예제)
│   ├── scenario.py       # 7DOF/2DOF 시나리오 로직
│   └── ui.py             # Tutorial UI
├── forward/              # Forward Kinematics 모듈
│   ├── scenario.py       # FK 계산 (LulaKinematicsSolver)
│   └── ui.py             # 조인트 슬라이더 UI
├── inverse/              # Inverse Kinematics 모듈
│   ├── scenario.py       # IK 계산 및 Workspace 분석
│   └── ui.py             # IK 제어 UI
└── assignments/          # Assignment 모듈
    └── assignment1/
        ├── scenario.py   # Assignment1 로직
        └── ui.py         # Assignment1 UI
```

---

## 🔧 각 모듈 역할

### `ui_builder.py`
- **World Controls**: LOAD, RESET 버튼
- 모든 하위 모듈 통합 및 lifecycle 관리

### `tutorial/`
- **Robot Information**: 조인트 상태 실시간 표시
- **Chapter2 Example**: 7DOF 사인파 궤적, 2DOF Prismatic 제어

### `forward/`
- 각 조인트 슬라이더로 로봇 직접 제어
- LulaKinematicsSolver로 EE Position/RPY 계산

### `inverse/`
- Target Cube 추종 IK 제어
- Position Only / Position+Orientation 모드
- Workspace 시각화 (도달 가능 영역 분석)

### `assignments/assignment1/`
- 순기구학 실습 (Pose 1, 2, 3 적용)
- FK 애니메이션 및 궤적 시각화
