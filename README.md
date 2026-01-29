<div align="center">

# 🤖 Robotics Study

**NVIDIA Isaac Sim 기반 로보틱스 학습 확장 프로그램**

[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-4.5.0+-76B900?style=for-the-badge&logo=nvidia&logoColor=white)](https://developer.nvidia.com/isaac-sim)
[![Python](https://img.shields.io/badge/Python-3.11+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue?style=for-the-badge)](LICENSE)

<p align="center">
  <img src="data/preview.png" alt="Robotics Study Preview" width="600"/>
</p>

[Features](#-features) •
[Installation](#-installation) •
[Usage](#-usage) •
[Modules](#-modules) •
[Project Structure](#-project-structure)

</div>

---

## 📖 Overview

**Robotics Study**는 NVIDIA Isaac Sim 환경에서 로보틱스 기초 이론을 실습할 수 있는 교육용 확장 프로그램입니다.

순기구학(FK), 역기구학(IK), 궤적 생성(Trajectory Generation), 자세 표현법(Orientation Representation) 등 로보틱스 핵심 개념을 **시각적으로 학습**할 수 있습니다.

### 주요 특징

- 🎯 **인터랙티브 학습** - 실시간 파라미터 조절 및 결과 확인
- 📊 **시각화 도구** - Debug Draw를 활용한 궤적/워크스페이스 시각화
- 🔧 **모듈화 설계** - 독립적인 기능별 모듈 구조
- 📚 **교육 목적 최적화** - Peter Corke의 Robotics Toolbox 알고리즘 구현

---

## ✨ Features

### 🎓 Tutorial
| 기능 | 설명 |
|------|------|
| **7DOF Manipulator** | ALLEX Right Arm 로봇의 사인파 궤적 제어 및 End-effector 궤적 시각화 |
| **2DOF Prismatic** | Revolute + Prismatic 조인트를 가진 2자유도 로봇 제어 |
| **Robot Information** | 조인트 상태 실시간 모니터링 |

### 🦾 Forward Kinematics
- 각 조인트 슬라이더를 통한 직관적인 로봇 제어
- End-Effector Position 및 RPY(Roll, Pitch, Yaw) 실시간 표시
- LulaKinematicsSolver 기반 정확한 FK 계산

### 🎯 Inverse Kinematics
- Target Cube를 따라가는 IK 제어
- **Position Only** / **Position + Orientation** 모드 전환
- **Workspace Visualization** - IK 도달 가능 영역 시각화
  - 🟢 초록: 도달 가능 | 🔴 빨강: 도달 불가

### 📐 Trajectory Generation

#### Trapezoidal Velocity Profile
- 사다리꼴 속도 프로파일 기반 궤적 생성
- 가속-등속-감속 구간 시각화

#### Orientation Trajectory
4가지 자세 보간 방법 비교 학습:

| 방법 | 설명 | 특징 |
|------|------|------|
| **Euler (ZYZ)** | `tr2eul` → `jtraj` → `eul2r` | Gimbal Lock 발생 가능 |
| **Angle-Axis** | `tr2angvec` → `jtraj` → `angvec2r` | 최단 경로, Singularity 없음 |
| **Quaternion (SLERP)** | 구면 선형 보간 | 최단 경로, Singularity 없음 |
| **HT (ctraj)** | `trinterp(T1, T2, s)` | 위치 + 자세 동시 보간 |

**보간 파라미터 옵션:**
- `linspace` - 등속 (constant velocity)
- `jtraj` - S-curve (조절 가능한 smoothness `n`)
- `ctraj` - 사다리꼴 (LSPB)

### 📝 Assignments
- **Assignment 1: Forward Kinematics** - ALLEX 로봇의 순기구학 실습 (Pose 애니메이션)

---

## 🚀 Installation

### 사전 요구사항

- NVIDIA Isaac Sim **4.5.0** 이상
- Python **3.11** 이상
- NVIDIA GPU (RTX 권장)

### 설치 방법

```bash
# 1. Isaac Sim의 extsUser 폴더로 이동
cd {ISAAC_SIM_PATH}/extsUser/

# 2. 리포지토리 클론
git clone https://github.com/KJG97/Robotics_Study.git

# 3. Isaac Sim 재시작
```

> 💡 `{ISAAC_SIM_PATH}`는 일반적으로 `~/.local/share/ov/pkg/isaac-sim-4.5.0` 입니다.

---

## 📖 Usage

### 기본 사용법

1. **Extensions 활성화**
   - `Window` → `Extensions` 열기
   - "Robotics Study" 검색 후 활성화

2. **확장 프로그램 실행**
   - 툴바에서 Robotics Study 아이콘 클릭
   - 또는 `Window` → `Robotics Study`

3. **로봇 로드**
   - `LOAD` 버튼 클릭하여 ALLEX 로봇 로드

4. **기능 사용**
   - 원하는 기능 버튼 클릭하여 학습 시작

### 예제: Orientation Trajectory

```
1. [Orientation Trajectory] 버튼 클릭
2. Input 섹션에서 Ri, Rf 각도 설정
3. [Spawn] 버튼으로 XYZ 좌표계 생성
4. 원하는 방법 선택 (Euler/Angle-Axis/Quaternion/HT)
5. [Generate] → [Execute]로 궤적 실행
6. Debug Draw로 궤적 확인
```

---

## 📦 Modules

```
┌─────────────────────────────────────────────────────────────┐
│                     Robotics Study                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────────┐ │
│  │Tutorial │  │ Forward │  │ Inverse │  │   Trajectory    │ │
│  │         │  │   (FK)  │  │   (IK)  │  │   Generation    │ │
│  └─────────┘  └─────────┘  └─────────┘  └─────────────────┘ │
│                                          ├── Trapezoidal    │
│                                          └── Orientation    │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────┐│
│  │                    Assignments                          ││
│  │  └── Assignment 1: Forward Kinematics                   ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
Robotics_Study/
├── 📄 README.md                    # 프로젝트 문서
├── 📁 asset/                       # 로봇 USD/URDF 파일
│   ├── ALLEX.usd                   # 전체 로봇
│   ├── ALLEX_Right_Arm.usd         # 오른팔 로봇
│   ├── XYZ.usd                     # 좌표계 시각화용
│   ├── prismatic_2dof.usd          # 2DOF 로봇
│   └── URDF_ALLEX_RightArm/        # FK/IK용 URDF 및 설정
│       ├── urdf/
│       └── config/
├── 📁 config/
│   ├── extension.toml              # 확장 설정
│   └── joint_config.json           # 조인트 설정
├── 📁 Robotics_Study_python/       # Python 소스코드
│   ├── extension.py                # 확장 진입점
│   ├── ui_builder.py               # 메인 UI
│   ├── global_variables.py         # 전역 변수
│   ├── 📁 tutorial/                # Tutorial 모듈
│   │   ├── scenario.py
│   │   └── ui.py
│   ├── 📁 forward/                 # Forward Kinematics
│   │   ├── scenario.py
│   │   └── ui.py
│   ├── 📁 inverse/                 # Inverse Kinematics
│   │   ├── scenario.py
│   │   └── ui.py
│   ├── 📁 trapezoidal/             # Trapezoidal Trajectory
│   │   ├── scenario.py
│   │   └── ui.py
│   ├── 📁 orientation_trajectory/  # Orientation Trajectory
│   │   ├── scenario.py             # 궤적 생성 로직
│   │   └── ui.py                   # UI 컴포넌트
│   └── 📁 assignments/             # 과제 모듈
│       └── assignment1/
├── 📁 data/                        # 리소스
│   ├── icon.png
│   └── preview.png
└── 📁 docs/                        # 문서
    ├── CHANGELOG.md
    └── trajectory_generation_manual.md
```

---

## 🔧 Architecture

각 모듈은 **Scenario-UI 패턴**으로 구성되어 있습니다:

```
┌──────────────┐      ┌──────────────┐
│     UI       │ ───▶ │   Scenario   │
│   (ui.py)    │      │ (scenario.py)│
└──────────────┘      └──────────────┘
       │                     │
       │                     ▼
       │              ┌──────────────┐
       │              │  Isaac Sim   │
       │              │     API      │
       └──────────────┴──────────────┘
```

- **UI (ui.py)**: omni.ui 기반 사용자 인터페이스
- **Scenario (scenario.py)**: 비즈니스 로직 및 시뮬레이션 제어

---

## 📚 References

- [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Robotics, Vision and Control (Book)](https://petercorke.com/rvc/)

---

## 🤝 Contributing

기여를 환영합니다! 다음 방법으로 참여할 수 있습니다:

1. 이 저장소를 Fork 합니다
2. Feature 브랜치를 생성합니다 (`git checkout -b feature/AmazingFeature`)
3. 변경사항을 커밋합니다 (`git commit -m 'Add some AmazingFeature'`)
4. 브랜치에 Push 합니다 (`git push origin feature/AmazingFeature`)
5. Pull Request를 생성합니다

---

## 📝 License

이 프로젝트는 **Apache-2.0 License** 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

---

<div align="center">

**Made with ❤️ for Robotics Education**

</div>
