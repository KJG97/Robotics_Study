# Robotics Study - Isaac Sim Extension

NVIDIA Isaac Sim 환경에서 로보틱스 기초를 학습하기 위한 확장 프로그램입니다.

## 📋 Features

### Tutorial (Chapter 2 Examples)
- **7DOF Manipulator** - ALLEX Right Arm 로봇의 사인파 궤적 제어 및 End-effector 궤적 시각화
- **2DOF Prismatic** - Revolute + Prismatic 조인트를 가진 2자유도 로봇 제어
- **Robot Information** - 조인트 상태 실시간 모니터링

### Forward Kinematics
- 각 조인트 슬라이더로 로봇 제어
- End-Effector Position 및 RPY(Roll, Pitch, Yaw) 실시간 표시
- LulaKinematicsSolver 기반 정확한 FK 계산

### Inverse Kinematics
- Target Cube를 따라가는 IK 제어
- Position Only / Position + Orientation 모드 전환
- **Workspace Visualization** - IK 도달 가능 영역 시각화 (초록: 도달 가능, 빨강: 불가능)

### Assignments
- **Assignment 1: Forward Kinematics** - ALLEX 로봇의 순기구학 실습 (Pose 애니메이션)

## 🚀 Installation

1. Isaac Sim의 extsUser 폴더로 이동:
   ```bash
   cd {ISAAC_SIM_PATH}/extsUser/
   ```

2. 리포지토리 클론:
   ```bash
   git clone git@github.com:KJG97/Robotics_Study.git
   ```

3. Isaac Sim 재시작 후 Extensions에서 활성화

## 📖 Usage

1. Window → Extensions 열기
2. "Robotics Study" 검색 및 활성화
3. 툴바에서 Robotics Study 아이콘 클릭
4. LOAD 버튼으로 로봇 로드
5. 각 기능 버튼 클릭하여 사용

## 📁 Project Structure

```
Robotics_Study/
├── asset/                        # 로봇 USD/URDF 파일
│   ├── ALLEX.usd
│   ├── ALLEX_Right_Arm.usd
│   ├── prismatic_2dof.usd
│   └── URDF_ALLEX_RightArm/      # FK/IK용 URDF 및 설정
│       ├── urdf/
│       └── config/
├── config/
│   ├── extension.toml            # 확장 설정
│   └── joint_config.json         # 조인트 설정
├── Robotics_Study_python/        # Python 소스코드
│   ├── extension.py              # 확장 진입점
│   ├── ui_builder.py             # 메인 UI
│   ├── tutorial/                 # Tutorial 모듈
│   ├── forward/                  # Forward Kinematics 모듈
│   ├── inverse/                  # Inverse Kinematics 모듈
│   └── assignments/              # Assignment 모듈
├── data/                         # 아이콘, 프리뷰 이미지
└── docs/                         # 문서
```

## 🛠️ Requirements

- NVIDIA Isaac Sim 4.5.0+
- Python 3.11+

## 📝 License

Apache-2.0 License
