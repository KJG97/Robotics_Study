# 7. Trajectory Generation - Trapezoidal Velocity Profile

<aside>
➡️

**사다리꼴 속도 함수를 이용한 궤적계획**

단일 조인트의 Point-to-Point 모션을 위한 시간 기반 궤적 생성

</aside>

---

## 7.1 개요

### 입출력 정의

| 항목 | **입력** | **출력** |
| --- | --- | --- |
| 파라미터 | qi, qf, tf, a_max | q(t), v(t), a(t) |
| 의미 | 초기위치, 목표위치, 이동시간, 최대가속도 | 시간에 따른 위치, 속도, 가속도 |

### 프로파일 종류

| **프로파일** | **조건** | **특징** |
| --- | --- | --- |
| **Trapezoidal** | tc < tf/2 | 가속 → 등속 → 감속 (3구간) |
| **Triangular** | tc = tf/2 | 가속 → 감속 (2구간, 등속구간 없음) |

---

## 7.2 수학적 배경

### 핵심 수식

**위치 함수 q(t):**

```
         ⎧ qi + (1/2)·a·t²                    0 ≤ t ≤ tc        (가속 구간)
q(t) =   ⎨ qi + a·tc·(t - tc/2)               tc < t ≤ tf-tc    (등속 구간)
         ⎩ qf - (1/2)·a·(tf-t)²               tf-tc < t ≤ tf    (감속 구간)
```

**가속시간 tc 계산:**

```
tc = tf/2 - (1/2)·√((tf²·a - 4·(qf-qi))/a)
```

**필수 조건:**

```
|a| ≥ 4·|qf-qi|/tf²
```

> ⚠️ 이 조건을 만족하지 않으면 주어진 시간 내에 목표 위치에 도달할 수 없습니다.

### 변수 정의

| 변수 | 의미 | 단위 |
| --- | --- | --- |
| qi | 초기 위치 | rad |
| qf | 최종 위치 | rad |
| tf | 총 이동 시간 | s |
| tc | 가속/감속 구간 시간 | s |
| a | 가속도 (부호 있음) | rad/s² |
| v_max | 최대 속도 = \|a\| · tc | rad/s |

---

## 7.3 Scenario 구축

### 필수 Import 및 경로 설정

```python
import numpy as np
from typing import Tuple, Optional
```

| **모듈** | **역할** |
| --- | --- |
| numpy | 수치 계산 (sqrt, clip 등) |
| typing | 타입 힌트 (Tuple, Optional) |

---

### 클래스 ① - TrapezoidalProfile (궤적 프로파일)

```python
class TrapezoidalProfile:
    """
    사다리꼴 속도 함수를 이용한 궤적계획
    
    q(t) = | qi + (1/2)*a*t²              0 ≤ t ≤ tc
           | qi + a*tc*(t - tc/2)         tc < t ≤ tf - tc
           | qf - (1/2)*a*(tf - t)²       tf - tc < t ≤ tf
    """

    def __init__(self, qi: float, qf: float, tf: float, tc: float, a: float):
        """
        Args:
            qi: Initial position [rad]
            qf: Final position [rad]  
            tf: Final time [s]
            tc: Acceleration/deceleration time [s]
            a: Acceleration (signed) [rad/s²]
        """
        self.qi = qi
        self.qf = qf
        self.tf = tf
        self.tc = tc
        self.a = a  # 부호 있는 가속도
        
        # v_max = |a| * tc
        self.v_max = abs(a * tc)
        self.is_triangular = (tc >= tf / 2 - 1e-6)  # tc ≈ tf/2 이면 삼각형
```

| **속성** | **타입** | **설명** |
| --- | --- | --- |
| qi | float | 초기 위치 [rad] |
| qf | float | 최종 위치 [rad] |
| tf | float | 총 이동 시간 [s] |
| tc | float | 가속/감속 시간 [s] |
| a | float | 가속도 (부호 포함) [rad/s²] |
| v_max | float | 최대 속도 [rad/s] |
| is_triangular | bool | 삼각형 프로파일 여부 |

---

### 핵심 함수 ① - get_state() (시간에 따른 상태 계산)

```python
def get_state(self, t: float) -> Tuple[float, float, float]:
    """
    Get (position, velocity, acceleration) at time t.
    
    수식:
    q(t) = | qi + (1/2)*a*t²              0 ≤ t ≤ tc
           | qi + a*tc*(t - tc/2)         tc < t ≤ tf - tc  
           | qf - (1/2)*a*(tf - t)²       tf - tc < t ≤ tf
    """
    t = np.clip(t, 0.0, self.tf)
    
    qi, qf, tf, tc, a = self.qi, self.qf, self.tf, self.tc, self.a
    
    if t <= tc:
        # Phase 1: Acceleration (0 ≤ t ≤ tc)
        q = qi + 0.5 * a * t**2
        v = a * t
        acc = a
    elif t <= tf - tc:
        # Phase 2: Cruise (tc < t ≤ tf - tc)
        q = qi + a * tc * (t - tc / 2)
        v = a * tc
        acc = 0.0
    else:
        # Phase 3: Deceleration (tf - tc < t ≤ tf)
        q = qf - 0.5 * a * (tf - t)**2
        v = a * (tf - t)
        acc = -a
    
    return float(q), float(v), float(acc)
```

**구간별 계산 흐름:**

```
┌─────────────────────────────────────────────────────────────────┐
│  Phase 1: 가속 구간 (0 ≤ t ≤ tc)                                │
│  ─────────────────────────────────                              │
│  q(t) = qi + (1/2)·a·t²                                         │
│  v(t) = a·t                                                     │
│  a(t) = a                                                       │
├─────────────────────────────────────────────────────────────────┤
│  Phase 2: 등속 구간 (tc < t ≤ tf - tc)                          │
│  ─────────────────────────────────────                          │
│  q(t) = qi + a·tc·(t - tc/2)                                    │
│  v(t) = a·tc = v_max                                            │
│  a(t) = 0                                                       │
├─────────────────────────────────────────────────────────────────┤
│  Phase 3: 감속 구간 (tf - tc < t ≤ tf)                          │
│  ─────────────────────────────────────                          │
│  q(t) = qf - (1/2)·a·(tf - t)²                                  │
│  v(t) = a·(tf - t)                                              │
│  a(t) = -a                                                      │
└─────────────────────────────────────────────────────────────────┘
```

| **반환값** | **타입** | **설명** |
| --- | --- | --- |
| q | float | 현재 위치 [rad] |
| v | float | 현재 속도 [rad/s] |
| acc | float | 현재 가속도 [rad/s²] |

---

### 클래스 ② - TrajectoryGenerationScenario (시나리오 관리)

```python
class TrajectoryGenerationScenario:
    """Single joint trajectory generation using trapezoidal profile."""
    
    TARGET_JOINT = "SP"  # 제어할 조인트 이름

    def __init__(self):
        self.articulation = None                    # 로봇 Articulation
        self._joint_index: Optional[int] = None     # 조인트 인덱스
        self._joint_limits: Tuple[float, float] = (-np.pi, np.pi)  # 조인트 한계
        self._trajectory: Optional[TrapezoidalProfile] = None      # 궤적 프로파일
        self._is_executing = False                  # 실행 상태
        self._elapsed_time = 0.0                    # 경과 시간
```

| **속성** | **타입** | **설명** |
| --- | --- | --- |
| articulation | Articulation | 로봇 객체 |
| _joint_index | int | 제어할 조인트의 인덱스 |
| _joint_limits | Tuple | 조인트 각도 한계 (lower, upper) |
| _trajectory | TrapezoidalProfile | 생성된 궤적 |
| _is_executing | bool | 궤적 실행 중 여부 |
| _elapsed_time | float | 실행 경과 시간 |

---

### 핵심 함수 ② - setup() (초기화)

```python
def setup(self, articulation):
    """Initialize with robot articulation."""
    self.articulation = articulation
    if not articulation:
        return
    
    dof_names = list(articulation.dof_names)
    if self.TARGET_JOINT in dof_names:
        self._joint_index = dof_names.index(self.TARGET_JOINT)
        lower = articulation.dof_properties["lower"][self._joint_index]
        upper = articulation.dof_properties["upper"][self._joint_index]
        self._joint_limits = (lower, upper)
        print(f"[TrajGen] Joint: {self.TARGET_JOINT}, Limits: [{np.degrees(lower):.0f}, {np.degrees(upper):.0f}]°")
    else:
        print(f"[TrajGen] ERROR: {self.TARGET_JOINT} not found!")
        self._joint_index = None
```

**초기화 흐름:**

```
Articulation 전달 → 조인트 이름 검색 → 인덱스 저장 → 한계값 획득
```

---

### 핵심 함수 ③ - generate() (궤적 생성)

```python
def generate(self, qi: float, qf: float, tf: float, a_max: float) -> dict:
    """
    Generate trapezoidal trajectory.
    
    tc = tf/2 - (1/2)*sqrt((tf²*a - 4*(qf-qi))/a)
    
    조건: |a| ≥ 4*|qf-qi|/tf²
    
    Args:
        qi: Initial position [rad]
        qf: Final position [rad]
        tf: Final time [s]
        a_max: Max acceleration magnitude [rad/s²]
    
    Returns:
        {"success": bool, "message": str}
    """
    if not self.is_ready:
        return {"success": False, "message": "Robot not ready"}
    
    # 1) 조인트 한계 내로 클램핑
    qi = np.clip(qi, *self._joint_limits)
    qf = np.clip(qf, *self._joint_limits)
    
    # 2) 입력값 검증
    h = qf - qi  # 부호 있는 변위
    if abs(h) < 1e-6:
        return {"success": False, "message": "qi == qf"}
    if tf <= 0:
        return {"success": False, "message": "tf <= 0"}
    if a_max <= 0:
        return {"success": False, "message": "a_max <= 0"}
    
    # 3) 필수 조건 검증: |a| ≥ 4*|qf-qi|/tf²
    a_min_required = 4 * abs(h) / (tf**2)
    if a_max < a_min_required:
        return {"success": False, "message": f"|a| must be ≥ {np.degrees(a_min_required):.1f}°/s²"}
    
    # 4) 가속도 부호 결정 (이동 방향에 따라)
    a = a_max if h > 0 else -a_max
    
    # 5) tc 계산: tc = tf/2 - (1/2)*sqrt((tf²*|a| - 4*|h|)/|a|)
    discriminant = (tf**2 * abs(a) - 4 * abs(h)) / abs(a)
    tc = tf / 2 - np.sqrt(discriminant) / 2
    
    # 6) tc 범위 제한: 0 < tc ≤ tf/2
    tc = np.clip(tc, 1e-6, tf / 2)
    
    # 7) 궤적 프로파일 생성
    self._trajectory = TrapezoidalProfile(qi, qf, tf, tc, a)
    self._elapsed_time = 0.0
    
    return {"success": True, "message": "OK"}
```

**궤적 생성 흐름:**

```
┌────────────────────────────────────────────────────────────────┐
│  Step 1: 입력값 검증                                           │
│  ────────────────────                                          │
│  • qi ≠ qf (이동 거리 존재)                                    │
│  • tf > 0 (양의 이동 시간)                                     │
│  • a_max > 0 (양의 가속도)                                     │
├────────────────────────────────────────────────────────────────┤
│  Step 2: 필수 조건 검증                                        │
│  ────────────────────────                                      │
│  • |a| ≥ 4·|qf-qi|/tf² 만족 여부 확인                          │
│  • 불만족 시 에러 반환 (시간 내 도달 불가)                     │
├────────────────────────────────────────────────────────────────┤
│  Step 3: tc 계산                                               │
│  ──────────────                                                │
│  • discriminant = (tf²·|a| - 4·|h|) / |a|                      │
│  • tc = tf/2 - √(discriminant)/2                               │
├────────────────────────────────────────────────────────────────┤
│  Step 4: TrapezoidalProfile 생성                               │
│  ─────────────────────────────────                             │
│  • TrapezoidalProfile(qi, qf, tf, tc, a) 객체 생성             │
└────────────────────────────────────────────────────────────────┘
```

| **반환값** | **타입** | **설명** |
| --- | --- | --- |
| success | bool | 궤적 생성 성공 여부 |
| message | str | 결과 메시지 |

---

### 핵심 함수 ④ - update() (매 Physics Step 호출)

```python
def update(self, dt: float):
    """Update trajectory (call every physics step)."""
    if not self._is_executing or not self._trajectory:
        return
    
    # 1) 경과 시간 업데이트
    self._elapsed_time += dt
    
    # 2) 종료 조건 체크
    if self._elapsed_time >= self._trajectory.duration:
        self._elapsed_time = self._trajectory.duration
        self._is_executing = False
    
    # 3) 현재 시간의 위치 계산
    q, _, _ = self._trajectory.get_state(self._elapsed_time)
    
    # 4) 로봇에 위치 적용
    if self.articulation:
        positions = self.articulation.get_joint_positions()
        if positions is not None:
            positions[self._joint_index] = q
            self.articulation.set_joint_positions(positions)
```

**실행 흐름:**

```
Physics Step → 경과시간 += dt → get_state(t) → 위치 계산 → 로봇에 적용
```

| **단계** | **동작** |
| --- | --- |
| 1 | 경과 시간 누적 |
| 2 | 종료 조건 체크 (t ≥ tf) |
| 3 | 현재 시간의 q(t) 계산 |
| 4 | 로봇 조인트에 위치 적용 |

---

## 7.4 UI 구축

### UI 클래스 구조

```python
class TrajectoryGenerationUI:
    """UI for single joint trajectory generation."""

    PLOT_SAMPLES = 100  # 플롯 샘플 수

    def __init__(self):
        self._scenario = TrajectoryGenerationScenario()
        self._timeline = omni.timeline.get_timeline_interface()
        self._window = None
        self._physics_sub = None
        
        # UI 요소들
        self._qi_field = None       # 초기 위치 입력
        self._qf_field = None       # 목표 위치 입력
        self._tf_field = None       # 이동 시간 입력
        self._a_max_field = None    # 최대 가속도 입력
        self._status_label = None   # 상태 표시
        self._execute_btn = None    # 실행 버튼
        self._position_plot = None  # 위치 그래프
        self._velocity_plot = None  # 속도 그래프
        self._accel_plot = None     # 가속도 그래프
```

### UI 레이아웃

```
┌─────────────────────────────────────────────────────┐
│  Status: Ready                                      │
├─────────────────────────────────────────────────────┤
│  qi: [  0.0  ]  qf: [ 45.0  ]  deg                  │
│  tf: [  2.0  ]  a:  [114.6  ]  deg/s²              │
├─────────────────────────────────────────────────────┤
│  [Generate]  [Execute]  [Current q]                 │
├─────────────────────────────────────────────────────┤
│  Pos    ┌─────────────────────────────────────┐    │
│  (deg)  │  ════════════════════════════════   │    │
│         └─────────────────────────────────────┘    │
├─────────────────────────────────────────────────────┤
│  Vel    ┌─────────────────────────────────────┐    │
│  (deg/s)│     /‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\         │    │
│         └─────────────────────────────────────┘    │
├─────────────────────────────────────────────────────┤
│  Acc    ┌─────────────────────────────────────┐    │
│  (deg/s²)│ ████                        ████   │    │
│         └─────────────────────────────────────┘    │
└─────────────────────────────────────────────────────┘
```

### 버튼 기능

| **버튼** | **기능** | **동작** |
| --- | --- | --- |
| Generate | 궤적 생성 | qi, qf, tf, a_max로 궤적 계산 및 플롯 업데이트 |
| Execute | 궤적 실행 | Physics callback 등록, 로봇 움직임 시작 |
| Current q | 현재 위치 | 로봇의 현재 조인트 위치를 qi에 설정 |

---

## 7.5 사용 예시

### 기본 사용법

```python
# 1) Scenario 생성 및 초기화
scenario = TrajectoryGenerationScenario()
scenario.setup(articulation)

# 2) 궤적 생성 (0° → 45°, 2초, 가속도 2 rad/s²)
result = scenario.generate(
    qi=np.radians(0),
    qf=np.radians(45),
    tf=2.0,
    a_max=np.radians(114.6)
)

# 3) 실행 시작
if result["success"]:
    scenario.start_execution()

# 4) Physics Step에서 호출
def on_physics_step(dt):
    scenario.update(dt)
```

### 파라미터 예시

| **케이스** | **qi** | **qf** | **tf** | **a_max** | **결과** |
| --- | --- | --- | --- | --- | --- |
| 기본 | 0° | 45° | 2.0s | 114.6°/s² | Trapezoidal |
| 빠른 이동 | 0° | 45° | 1.0s | 200°/s² | Trapezoidal |
| 최소 가속도 | 0° | 45° | 2.0s | 45°/s² | Triangular |
| 가속도 부족 | 0° | 45° | 1.0s | 50°/s² | 실패 (조건 불만족) |

---

## 7.6 주의사항

### 필수 조건

```
|a| ≥ 4·|qf-qi|/tf²
```

- 이 조건을 만족하지 않으면 주어진 시간 내에 목표에 도달할 수 없음
- UI에서 자동으로 필요한 최소 가속도를 계산하여 에러 메시지로 표시

### 삼각형 vs 사다리꼴

| **조건** | **프로파일** | **특징** |
| --- | --- | --- |
| tc < tf/2 | Trapezoidal | 등속 구간 존재 |
| tc = tf/2 | Triangular | 등속 구간 없음, 가속 직후 감속 |

### Physics Callback 관리

```python
# 중복 등록 방지
def _remove_physics_callback(self):
    if self._physics_sub:
        self._physics_sub.unsubscribe()
        self._physics_sub = None
    try:
        world = World.instance()
        if world and world.physics_callback_exists("trajgen_update"):
            world.remove_physics_callback("trajgen_update")
    except Exception:
        pass
```

> ⚠️ Physics callback은 반드시 제거 후 재등록해야 중복 등록 오류를 방지할 수 있습니다.

---

## 7.7 파일 구조

```
trajectory_generation/
├── __init__.py
├── scenario.py      # TrapezoidalProfile, TrajectoryGenerationScenario
└── ui.py            # TrajectoryGenerationUI
```

| **파일** | **역할** |
| --- | --- |
| scenario.py | 궤적 계산 로직 (수학적 계산) |
| ui.py | 사용자 인터페이스 (입력, 플롯, 버튼) |
