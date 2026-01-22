# Robotics Study - Isaac Sim Extension

NVIDIA Isaac Sim 환경에서 로보틱스 기초를 학습하기 위한 확장 프로그램입니다.

## 📋 Features

### Chapter 2 Examples
- **Example 3: 7DOF Manipulator** - ALLEX Right Arm 로봇의 사인파 궤적 제어 및 End-effector 궤적 시각화
- **Example 4: 2DOF Prismatic** - Revolute + Prismatic 조인트를 가진 2자유도 로봇 제어

### Assignments
- **Assignment 1: Forward Kinematics** - ALLEX 로봇의 순기구학 실습 (Pose 1, 2, 3 적용 및 애니메이션)

## 🚀 Installation

1. Isaac Sim이 설치되어 있어야 합니다.
2. 이 폴더를 Isaac Sim의 확장 폴더에 배치합니다:
   ```
   {ISAAC_SIM_PATH}/extsUser/Robotics_Study
   ```

## 📖 Usage

Isaac Sim 실행 시 확장을 활성화합니다:

```bash
./isaac-sim.sh --ext-folder /path/to/extsUser --enable Robotics_Study
```

또는 Isaac Sim UI에서:
1. Window → Extensions 열기
2. "Robotics Study" 검색
3. 활성화 (Enable)

## 📁 Project Structure

```
Robotics_Study/
├── asset/                    # 로봇 USD/URDF 파일
│   ├── ALLEX.usd
│   ├── ALLEX_Right_Arm.usd
│   └── prismatic_2dof.usd
├── config/
│   ├── extension.toml        # 확장 설정
│   └── joint_config.json     # 조인트 설정
├── Robotics_Study_python/    # Python 소스코드
│   ├── extension.py          # 확장 진입점
│   ├── ui_builder.py         # 메인 UI
│   ├── scenario.py           # 메인 시나리오 (Chapter2)
│   └── assignments/          # Assignment별 모듈
│       └── assignment1/
│           ├── scenario.py
│           └── ui.py
├── data/                     # 아이콘, 프리뷰 이미지
└── docs/                     # 문서
```

## 🛠️ Requirements

- NVIDIA Isaac Sim 5.1.0
- Python 3.11+

## 📝 License

Apache-2.0 License

