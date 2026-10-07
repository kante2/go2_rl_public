# Go2 강화학습: 현재 구성과 학습 기록

이 문서는 `~/go2_rl`에서 Unitree Go2의 속도 추종 보행 정책을 어떻게 학습하고 있는지 기록한다. 학습 코드는 mjlab 환경과 RSL-RL의 PPO를 사용하며, 정책이 목표 속도와 로봇 상태를 받아 12개 관절의 위치 목표를 출력한다. 경로 계획·LiDAR SLAM·자율주행 노드는 보행 정책 학습과 별도 구성이다.

- 기록 기준: **2026-10-07, Asia/Seoul**.
- 원본 기준 커밋: `35605c5c01887d7ab099736c357536e5006d64e6`.
- 원본 작업 트리의 **커밋되지 않은 로컬 수정도 반영**했다. 커밋 하나만 체크아웃하면 이 문서의 전체 상태가 재현되는 것은 아니다.
- 본문에 표시한 소스·산출물 경로는 모두 **`~/go2_rl` 기준**이다. 이 공개 저장소는 문서 저장소이므로 해당 소스, 체크포인트, 로그, 로컬 Docker 이미지를 포함하지 않는다.
- 이번 정리는 코드·설정·기존 산출물을 읽고 검사한 결과다. 새 학습이나 로봇 조작을 실행한 기록이 아니다.

관련 문서: [시뮬레이터](../simulator/README.md), [실제 로봇](../real/README.md), [시뮬레이션과 실기 매핑](../sim2real/README.md).

## 현재 확인된 상태

| 항목 | 현재 상태 | 근거 경로 |
|---|---|---|
| 등록된 태스크 | `Unitree-Go2-Flat`, `Unitree-Go2-Rough` | `src/tasks/go2/registry.py` |
| 평지 입력·출력 | Actor 47, Critic 74, 행동 12 | `step_02_learning/observations.py`, 저장 모델·ONNX |
| 짧은 학습 실행 | Flat에서 환경 8개 × 8스텝 × 3회 PPO 업데이트를 수행한 로그 2개 존재 | `logs/rsl_rl/go2_velocity/*_smoke/` |
| 별도 보행 체크포인트 | 루트 `model_10000.pt` 존재, 내부 `iter=10000`, Actor 47·Critic 74와 일치 | `model_10000.pt` |
| 실기용 정책 추출 | 정규화가 포함된 ONNX, 매핑 JSON, 비교용 관측·행동 100프레임 존재 | `artifacts/go2_real/` |
| 확인 범위 | 저장 모델·학습 지표 유한값, 정책 가중치 변경, ONNX 구조, 오프라인 추론·관측 매핑 검사 | 아래 검증 기록 |
| 남은 평가 | 동일 조건의 장시간 학습 재현, Rough 학습 완료, 정량 보행 성능과 실기 성능을 증명하는 기록은 이 조사에서 확보하지 않음 | 모델 파일 존재와 실행 성능은 별도로 판단 |

`model_10000.pt`의 반복 번호는 직접 확인했지만, 이 모델의 원래 전체 학습 로그·당시 설정·코드 버전이 현재 `logs/`에 함께 남아 있지는 않다. 현재 소스로 처음부터 10,001회 학습한 결과라고 단정할 수 없다. 기존 `doc/go2_autonomous_driving/kante_2.md`에는 이 모델을 Flat 환경에 불러온 Viser 실행 로그가 있다. 그 기록은 정책 로딩과 환경 구성을 보여주며, 속도 오차·낙상률 같은 성능 수치를 제공하지 않는다.

## 코드 구조와 실제 학습 흐름

```text
scripts/train.py
  → src.tasks 로딩: Flat/Rough 환경·PPO 설정·Runner 등록
  → 선택한 환경/PPO 설정에 CLI 값을 덮어쓰기
  → ManagerBasedRlEnv 생성
  → RslRlVecEnvWrapper로 RSL-RL 인터페이스 연결
  → VelocityOnPolicyRunner
      관측 → Actor → 관절 목표 → MuJoCo 물리 → 보상·종료·다음 관측
      각 환경에서 rollout 수집 → GAE/return 계산 → PPO 업데이트
      체크포인트·설정·학습 지표·추론용 ONNX 저장

scripts/play.py
  → play 환경 설정 → 체크포인트의 Actor 로딩 → 시뮬레이션 추론
```

PPO와 rollout 버퍼의 실제 구현은 설치된 `rsl-rl-lib`에 있다. 이 저장소는 환경·보상·관측·학습 옵션과 Runner의 저장 동작을 정의한다. Actor는 배포에 사용할 행동을 만들고, Critic은 학습 중 더 많은 시뮬레이션 정보를 사용해 상태 가치를 추정한다. 실기 추론에는 Critic이 필요하지 않다.

`src/tasks/go2/` 아래의 역할은 다음과 같다.

| 원본 파일 | 역할 |
|---|---|
| `registry.py` | 두 태스크의 학습 환경, play 환경, PPO 설정, `VelocityOnPolicyRunner` 등록 |
| `step_01_environment/base_env.py` | 지형·물리·제어 시간 간격·에피소드와 각 manager 설정 조립 |
| `step_01_environment/go2_env.py` | Go2 모델·발 센서·Go2 보상 파라미터 연결, Rough/Flat 및 play 차이 적용 |
| `step_01_environment/scene.py` | Rough의 지형 raycast 센서 |
| `step_01_environment/actions.py` | 정책 출력을 관절 위치 목표로 변환 |
| `step_01_environment/commands.py` | 목표 몸통 속도·heading 명령 설정 |
| `step_01_environment/events.py` | 리셋, 마찰·인코더·무게중심 랜덤화, 외란 |
| `step_02_learning/observations.py` | Actor/Critic 관측 순서·노이즈·이력 |
| `step_02_learning/rewards.py` | 활성 보상 항목과 가중치 |
| `step_02_learning/terminations.py` | 시간 제한과 기울기 종료 설정 |
| `step_02_learning/curriculum.py` | 지형·속도 범위 난이도 스케줄 |
| `step_02_learning/metrics.py` | 행동 가속도 진단 지표 |
| `step_02_learning/mdp/observations.py`, `rewards.py`, `curriculums.py` | 프로젝트에서 추가한 관측·보상·난이도 조정 계산 |
| `step_03_training/ppo.py` | Actor/Critic 구조와 PPO·Runner 설정 |
| `step_03_training/train.py` | CLI, GPU·seed 선택, 환경 생성, resume, 로깅, 학습 시작 |
| `step_03_training/runner.py` | 체크포인트 저장 시 ONNX 추출·메타데이터 첨부 |
| `step_04_evaluation/play.py` | 체크포인트·zero/random 정책 재생, Native/Viser viewer |

로봇 자세·모터 설정은 `src/assets/robots/unitree_go2/go2_constants.py`, 기구학·질량·관절 범위·충돌 형상은 `src/assets/robots/unitree_go2/xmls/go2.xml`에서 가져온다. `scripts/train.py`와 `scripts/play.py`는 위 구현에 연결하는 진입점이다.

**실제로 사용되는 클래스에 주의한다.** `commands.py`는 `mjlab.tasks.velocity.mdp.UniformVelocityCommandCfg`를 import한다. 프로젝트의 `step_02_learning/mdp/velocity_command.py`에 별도 구현이 있어도 현재 기본 명령 생성에 선택된 클래스는 mjlab의 구현이다. 또한 `go2_env.py`의 `illegal_contact` 설정은 `mjlab.tasks.velocity.mdp`의 함수를 사용한다. 파일이 존재하는 것과 해당 실행에서 활성화되는 것은 다르므로 import와 설정을 함께 확인해야 한다.

## 환경과 태스크

| 설정 | Flat | Rough |
|---|---|---|
| 지형 | 평면 `plane` | mjlab `ROUGH_TERRAINS_CFG`를 사용하는 생성 지형 |
| 지형 스캔 | 없음 | yaw 정렬 raycast, 1.6 × 1.0 m, 간격 0.1 m, 최대 거리 5 m |
| 지형 난이도 | 제거 | 초기 최대 레벨 5, 이동 거리 기반 curriculum |
| Actor 차원 | 47 | 설정상 234 = 47 + 지형 스캔 187 |
| Critic 차원 | 74 | 설정상 261 = 74 + 지형 스캔 187 |
| 행동 차원 | 12 | 12 |

Rough의 187개 ray는 현재 설치된 mjlab의 grid 생성 함수를 물리 환경 없이 호출해 확인했다. Rough 차원은 소스의 관측 구성에 따른 합계이며, 이번 조사에서 Rough rollout을 실행한 검증값은 아니다.

공통 시간 설정은 물리 `timestep=0.005 s`와 `decimation=4`다. 정책·환경 스텝은 `0.02 s`, 즉 **50 Hz**이며 한 행동을 4개의 물리 스텝 동안 적용한다. 학습 에피소드는 최대 20초, 즉 최대 1,000개의 환경 스텝이다. 소스의 기본 `scene.num_envs`는 **1개**이므로 병렬 학습 규모는 CLI에서 지정해야 한다. README에 있는 4,096개는 실행 예시 값이다.

발 센서는 `FR, FL, RR, RL` 순서로 접촉·힘·공중 시간을 제공한다. 발 이외 충돌 센서는 접촉 힘 이력 4개를 유지한다. 현재 로봇 구성은 `FULL_COLLISION`을 사용하며 지면 충돌은 활성화하고 자기 충돌은 제외한다. `FEET_ONLY_COLLISION`도 정의되어 있으나 현재 선택된 구성은 아니다.

### 목표 속도와 난이도

학습 명령 `twist`는 `[vx, vy, yaw_rate]`이며, 3~8초마다 재샘플링한다. 5%의 환경은 정지 명령을 받는다. Heading 제어가 활성화되어 있으며 목표 방향 범위는 `[-π, π]`, 방향 오차에서 회전 속도를 만드는 계수는 0.5다. 현재 mjlab 기본 `rel_heading_envs=1.0`을 사용한다.

| 단계 | 전후 속도 vx (m/s) | 좌우 속도 vy (m/s) | 회전 속도 (rad/s) |
|---|---:|---:|---:|
| 명령 설정의 원래 범위 | [-1.0, 2.0] | [-1.0, 1.0] | [-1.0, 1.0] |
| 학습 curriculum 첫 단계 | [-0.5, 1.0] | [-0.5, 0.5] | [-1.0, 1.0] |
| 공통 환경 스텝 > 120,000 | [-1.0, 2.0] | [-1.0, 1.0] | 기존 범위 유지 |

120,000은 `5000 × 24`로 설정한 **공통 환경 스텝 수**다. 환경 수를 곱한 총 transition 수가 아니며, 코드가 `>`로 비교한다. rollout을 24에서 8로 바꾸면 같은 임계값까지 필요한 PPO 반복 수가 달라진다. 난이도 변경은 curriculum이 호출되는 리셋 시점에 반영된다.

Rough 지형 curriculum은 로봇이 지형 길이의 절반 이상 이동하면 난이도를 올리고, 목표 속도·에피소드 시간으로 계산한 기대 이동 거리의 절반보다 덜 이동하면 낮춘다. Flat에서는 지형 curriculum만 제거하고 속도 curriculum은 유지한다.

## 관측: 무엇을 보고 행동하는가

Flat Actor는 아래 순서대로 연결한 **47개 값**, 이력 1프레임을 받는다. 표의 인덱스는 0부터 시작한다.

| 인덱스 | 관측 | 차원 | 의미 | 학습 시 균등 노이즈 |
|---|---|---:|---|---|
| 0~2 | `base_ang_vel` | 3 | 로봇 IMU 각속도 | ±0.2 |
| 3~5 | `projected_gravity` | 3 | 몸체 좌표계로 투영한 중력 방향 | ±0.05 |
| 6~8 | `command` | 3 | 목표 vx, vy, yaw rate | 없음 |
| 9~10 | `phase` | 2 | 보행 위상의 sin·cos | 없음 |
| 11~22 | `joint_pos` | 12 | 기본 자세에 대한 관절 위치 차이 | ±0.01 rad |
| 23~34 | `joint_vel` | 12 | 기본 관절 속도에 대한 속도 차이 | ±1.5 rad/s |
| 35~46 | `actions` | 12 | 직전 정책 행동 | 없음 |

위상은 `episode_length_buf × step_dt`에서 계산하며 주기는 **0.6초**다. 명령 벡터의 norm이 0.1보다 작으면 두 위상 값을 0으로 만든다. 관절 위치·속도·이전 행동은 모두 정책의 관절 순서를 사용해야 한다.

Flat Critic은 같은 47개 값에 아래 **27개**를 추가하여 74개를 받는다. Critic은 관측 노이즈를 적용하지 않는다.

| 추가 관측 | 차원 | 계산 |
|---|---:|---|
| `base_lin_vel` | 3 | 시뮬레이터의 몸체 선속도 센서 |
| `foot_height` | 4 | 각 발 site의 월드 z 좌표 |
| `foot_air_time` | 4 | 각 발이 현재 공중에 있는 시간 |
| `foot_contact` | 4 | 지면 접촉 여부를 0/1로 표현 |
| `foot_contact_forces` | 12 | 월드 좌표계 발 접촉력에 `sign(F) × log(1 + abs(F))` 적용 |

Rough는 Actor에서 `actions` 다음에 `height_scan`을 추가한다. Critic에서는 이 스캔이 `base_lin_vel`보다 앞에 위치한다. 스캔 scale은 `1/5`이고 Actor의 스캔 노이즈 설정은 ±0.1이다. 두 그룹 모두 전체 이력 길이는 1이다.

Actor/Critic은 네트워크 내부의 경험적 관측 정규화를 사용한다. 따라서 배포할 때도 **정규화가 포함된 Actor**를 내보내야 한다. 원시 관측의 순서·단위·기본 자세가 맞아야 하며, ONNX 입력 차원만 맞는 것으로는 충분하지 않다.

## 행동: 관절 위치 목표와 모터 설정

Actor의 행동은 토크가 아니라 관절별 기본 자세의 변화량이다.

```text
q_target = q_default + 0.25 × action
```

`use_default_offset=True`이며 wrapper의 기본 `clip_actions=None`이므로 정책 행동에 `[-1, 1]` 범위가 자동 보장되는 것은 아니다. 시뮬레이션의 위치 액추에이터가 위치 오차와 속도를 사용해 힘을 만든다.

| 관절 종류 | 기본 각도 (rad) | stiffness / damping | effort limit | armature |
|---|---:|---|---:|---:|
| 왼쪽 hip | -0.1 | 20 / 1 | 23.5 | 0.01 |
| 오른쪽 hip | +0.1 | 20 / 1 | 23.5 | 0.01 |
| thigh | 0.9 | 20 / 1 | 23.5 | 0.01 |
| calf | -1.8 | 40 / 2 | 45 | 0.02 |

초기 몸통 높이는 0.32 m이고 관절 기본 속도는 0이다. 관절의 soft limit은 모델 범위의 90%를 사용한다. 현재 추출된 정책의 순서는 **FL → FR → RL → RR**, 각 다리 안에서는 **hip → thigh → calf**다. 실제 SDK 순서와 변환 방법은 [sim2real 문서](../sim2real/README.md)에 정리한다.

## 보상과 에피소드 종료

`step_02_learning/rewards.py`에 15개 항목이 활성화되어 있다. 실제 계산은 해당 함수 또는 mjlab 공통 함수에 있다. 현재 `scale_rewards_by_dt=True`이므로 환경이 받는 한 스텝의 보상은 각 항목의 **계산값 × 가중치 × 0.02초**를 합산한다.

| 항목 | 가중치 | 유도하는 동작 |
|---|---:|---|
| `track_linear_velocity` | +1.0 | 목표 몸체 vx·vy 추종, 수직 속도 억제 |
| `track_angular_velocity` | +1.0 | 목표 yaw 속도 추종, roll·pitch 각속도 억제 |
| `body_orientation_l2` | -1.0 | 몸통이 수평에 가까운 자세 유지 |
| `pose` | +1.0 | 속도별 허용 폭 내에서 기본 관절 자세 유지 |
| `body_ang_vel` | -0.05 | 몸통의 roll·pitch 각속도 억제 |
| `angular_momentum` | -0.025 | 전체 각운동량 크기 억제 |
| `is_terminated` | -200.0 | 실패 종료 회피 |
| `joint_acc_l2` | -2.5e-7 | 관절 가속도 억제 |
| `joint_pos_limits` | -10.0 | soft 관절 범위 이탈 억제 |
| `action_rate_l2` | -0.05 | 연속 행동의 급격한 변화 억제 |
| `foot_gait` | +0.5 | 발 접촉 순서가 목표 보행 위상과 일치하도록 유도 |
| `foot_clearance` | -1.0 | 움직이는 발 높이를 목표 0.10 m에 가깝게 유도 |
| `foot_slip` | -0.25 | 지면에 닿은 발의 수평 미끄러짐 억제 |
| `soft_landing` | -0.001 | 첫 접촉 순간의 발 접촉력 억제 |
| `stand_still` | -1.0 | 정지 명령에서 기본 관절 자세 유지 |

주요 계산은 다음과 같다.

```text
linear reward = exp(-(‖v_cmd_xy - v_body_xy‖² + 2·v_body_z²) / 0.25)
angular reward = exp(-((yaw_cmd - yaw_body)² + 0.05·‖omega_body_xy‖²) / 0.5)
```

각속도 보상의 주석에 heading error라는 설명이 있으나, 실제 함수는 **생성된 yaw 속도와 실제 yaw 속도의 차이**를 사용한다. 목표 방향에서 yaw 속도를 만드는 처리는 command manager에서 수행한다.

Go2 보행 리듬은 주기 0.6초, 발 순서 `FR, FL, RR, RL`의 phase offset `[0.0, 0.5, 0.5, 0.0]`으로 대각선 두 발을 같은 위상에 둔다. 접지 위상 기준은 0.56이다. 움직임 관련 발 보상은 `‖command_xy‖ + abs(command_yaw)`가 0.1을 넘을 때 활성화된다.

자세 보상은 `exp(-mean((q-q_default)² / std²))`이다. 명령 속도 기준 0.1 미만에서 hip/thigh/calf의 std는 `0.05/0.1/0.15`, 걷기·달리기에서는 `0.15/0.35/0.5`다. 달리기 분기 기준은 1.5이며, 현재 걷기와 달리기의 std는 같다.

`foot_clearance`는 **월드 z 좌표와 0.10 m의 차이**에 발 수평 속도를 곱한다. Rough에서도 지면으로부터의 상대 높이를 직접 사용하는 계산은 아니므로, 지형별 보상 의도를 바꿀 때 이 점을 먼저 확인해야 한다. `feet_air_time`, `feet_swing_height`, `self_collision_cost` 등 함수가 존재하더라도 현재 15개 보상 목록에 없는 함수는 활성 보상으로 계산하지 않는다.

에피소드는 20초 timeout, 몸통 기울기 70도 초과, 또는 발 이외 형상의 지면 접촉력이 10 N을 넘을 때 종료한다. Timeout은 실패 종료와 구분한다. 이는 **학습 환경**의 종료 조건이며, 실기 제어기의 안전 전환 조건과 동일하다는 뜻은 아니다.

## 랜덤화와 play 모드

| 이벤트 | 시점 | 현재 범위 |
|---|---|---|
| 몸통 리셋 | reset | x·y ±0.5 m, yaw ±3.14 rad; z 추가 오프셋 0 |
| 관절 리셋 | reset | 기본 자세·속도로 리셋; 현재 추가 랜덤 오프셋 0 |
| 발 마찰 | startup | 0.3~1.6, 네 발이 같은 샘플 사용 |
| encoder bias | startup | ±0.015 rad |
| 몸통 무게중심 | startup | x·y·z 각 ±0.05 m |
| 외부 밀기 | 5~6초 간격 | 선속도 x·y ±0.5, z ±0.4 m/s; 각속도 roll·pitch ±0.52, yaw ±0.78 rad/s |

이 목록에 모터 지연·통신 지연·질량·모터 이득 랜덤화는 별도로 설정되어 있지 않다. 실제 로봇과의 차이를 모두 학습에서 처리한 상태라고 볼 수 없다.

Play에서는 Actor 관측 노이즈와 외부 밀기를 제거하고 curriculum을 비활성화하며 에피소드 시간 제한을 `1e9초`로 늘린다. **startup의 마찰·encoder bias·COM 랜덤화와 실패 종료 조건은 유지**한다. Rough는 5 × 5 지형 격자로 바꾸고 reset에서 지형을 재선택한다. Flat play 명령 범위는 vx `[-0.5, 1.0]`, vy `[-0.5, 0.5]`, yaw `[-0.5, 0.5]`다. 따라서 play는 학습 당시 조건 전체를 그대로 재생하는 평가가 아니다.

## PPO 설정과 저장

| 설정 | 기본값 |
|---|---|
| Actor / Critic | 각각 hidden layers 512 → 256 → 128, ELU |
| 관측 정규화 | Actor / Critic 모두 활성화 |
| 행동 분포 | `GaussianDistribution`, `init_std=1.0`, `std_type=scalar` |
| rollout | 환경당 24스텝 |
| 최대 학습 반복 | 10,001회 |
| 저장 간격 | 100회 |
| seed | 42; 멀티 GPU는 local rank를 더함 |
| optimizer / learning rate | Adam / 0.001 |
| learning rate schedule | adaptive, 목표 KL 0.01 |
| epochs / mini batches | 5 / 4 |
| PPO clip / value loss coefficient | 0.2 / 1.0; clipped value loss 활성화 |
| entropy coefficient | 0.01 |
| gamma / GAE lambda | 0.99 / 0.95 |
| max gradient norm | 1.0 |
| experiment / 기본 logger | `go2_velocity` / W&B |

예를 들어 환경 4,096개와 rollout 24개를 지정하면 PPO 한 번에 transition 98,304개를 수집한다. smoke 설정은 8 × 8 = 64개씩 수집하고 3회 업데이트하여 총 192개를 처리한다. 학습 시작 시 `init_at_random_ep_len=True`로 환경들의 초기 에피소드 진행도를 분산한다.

```text
logs/rsl_rl/go2_velocity/<실행시각>_<run_name>/
├── model_<iteration>.pt    Actor·Critic·optimizer·반복 번호
├── policy.onnx            정규화 포함 Actor, 정책 메타데이터
├── events.out.tfevents.*   TensorBoard를 선택한 경우 학습 지표
├── params/agent.yaml       실제 사용한 PPO·Runner 설정
├── params/env.yaml         실제 사용한 환경·관측·보상 설정
├── git/                    Runner가 남기는 코드 기록
└── videos/train/           영상 기록을 선택한 경우
```

반복 번호는 0부터 시작한다. Runner는 종료 시에도 모델을 저장하므로 10,001회 반복의 마지막 번호는 10000이다. `VelocityOnPolicyRunner.save()`는 체크포인트를 저장할 때마다 **같은 이름 `policy.onnx`를 갱신**한다. 따라서 `model_0.pt`와 실행 폴더의 최종 `policy.onnx`가 같은 반복의 정책이라고 가정하면 안 된다. 반복별 ONNX를 보관하려면 별도 이름으로 저장해야 한다.

현재 실제 존재하는 ONNX는 단일 파일이다. 외부 데이터 파일을 사용하는 모델만 해당 `.data` 파일이 함께 필요하다. 원본 README의 설명과 달리 모든 export에서 `policy.onnx.data`가 생성되는 것은 아니다. 로그 폴더 이름은 실행 프로세스의 시간 설정으로 만들어지므로 폴더의 시각 표기를 자동으로 한국 시간이라고 단정하지 않는다.

## 현재 환경에서 재현하는 명령

아래 명령은 **문서 저장소가 아니라 `~/go2_rl`에서 실행**한다. 현재 `docker/docker-compose.yml`의 service 이름은 `unitree-rl`, 컨테이너 이름은 `unitree-rl-mjlab`, 소스 mount는 `/workspace/unitree_rl_mjlab`이다. 이미지 `unitree-rl-mjlab:go2-rl-relocation`은 현재 머신에 보존된 로컬 이미지이며 공개 배포 이미지나 Dockerfile이 함께 제공된 구성은 아니다.

2026-10-07 현재 컨테이너에서 읽어 확인한 버전은 Python **3.11.16**, PyTorch **2.14.0+cu130**, mjlab **1.2.0**, MuJoCo **3.5.0**, mujoco-warp **3.5.0**, warp-lang **1.12.0**, rsl-rl-lib **5.0.1**이다. `setup.py`가 직접 고정한 버전은 `mjlab==1.2.0`, `mujoco-warp==3.5.0`뿐이다. 원본 `kante_.md`의 2026-09-30 검증 환경(Python 3.11.15, PyTorch 2.7.0+cu128)은 과거 기록으로 구분한다.

현재 로컬 구성을 올리고 태스크 목록 확인:

```bash
cd ~/go2_rl
docker compose -f docker/docker-compose.yml up -d unitree-rl
docker exec -w /workspace/unitree_rl_mjlab unitree-rl-mjlab \
  python scripts/list_envs.py
```

환경 생성·학습·저장 경로를 확인하는 짧은 Flat 학습:

```bash
docker exec -w /workspace/unitree_rl_mjlab unitree-rl-mjlab \
  python scripts/train.py Unitree-Go2-Flat \
  --env.scene.num-envs 8 \
  --agent.max-iterations 3 \
  --agent.num-steps-per-env 8 \
  --agent.save-interval 1 \
  --agent.logger tensorboard \
  --agent.run-name pipeline_smoke
```

평지 본학습 예시:

```bash
docker exec -w /workspace/unitree_rl_mjlab unitree-rl-mjlab \
  python scripts/train.py Unitree-Go2-Flat \
  --env.scene.num-envs 4096 \
  --agent.logger tensorboard \
  --agent.run-name flat_training
```

Rough는 태스크 이름을 `Unitree-Go2-Rough`로 바꾼다. 환경 수는 GPU 메모리에 맞게 조정한다. 여러 GPU를 사용하는 경우 `--gpu-ids 0 1`을 추가하며 GPU마다 동일한 환경 수를 사용한다. 이러한 본학습 명령은 사용 예시이며 이번 문서화 과정에서 실행하지 않았다.

로그가 남아 있는 smoke 체크포인트 재생:

```bash
docker exec -w /workspace/unitree_rl_mjlab unitree-rl-mjlab \
  python scripts/play.py Unitree-Go2-Flat \
  --checkpoint-file logs/rsl_rl/go2_velocity/2026-10-07_01-37-23_docker_setup_smoke/model_2.pt \
  --num-envs 1 --device cuda:0 --viewer viser
```

별도 보행 모델을 보려면 `--checkpoint-file model_10000.pt`로 바꾼다. `--viewer native`는 그래픽 세션이 있는 환경에서 사용한다. `auto`는 DISPLAY/WAYLAND_DISPLAY의 유무로 Native 또는 Viser를 고른다. Play는 Actor 추론만 수행하며, 불러온 체크포인트를 학습 중 파일 변화에 맞추어 자동 갱신하지 않는다.

TensorBoard 확인:

```bash
docker exec -w /workspace/unitree_rl_mjlab unitree-rl-mjlab \
  tensorboard --logdir logs/rsl_rl/go2_velocity
```

실기용 Actor와 검증 데이터를 추출하는 명령은 다음과 같다. 기존 `artifacts/go2_real/`를 지정하면 그 안의 출력 파일이 갱신된다.

```bash
docker exec -w /workspace/unitree_rl_mjlab unitree-rl-mjlab \
  python scripts/export_go2_real_policy.py \
  --checkpoint model_10000.pt --output artifacts/go2_real
```

이 스크립트는 Flat의 정규화 포함 Actor를 ONNX로 내보내고 `policy.json`에 관측·관절 이름·행동 배율·PD·joint limit·SDK 매핑·시간 간격·파일 hash를 저장한다. 이후 시뮬레이터 100스텝에서 `parity.npz`와 `samples.json`을 만든다. 이 명령은 실기 모터 명령을 발행하는 스크립트가 아니다.

## 산출물과 검증 기록

현재 학습 로그 폴더는 다음 두 개다.

```text
logs/rsl_rl/go2_velocity/2026-09-30_18-51-15_refactor_smoke/
logs/rsl_rl/go2_velocity/2026-10-07_01-37-23_docker_setup_smoke/
```

두 폴더의 `params/agent.yaml`, `params/env.yaml`은 seed 42, 평면 지형, 환경 8개, rollout 8, 학습 반복 3, 저장 간격 1, TensorBoard 사용을 기록한다. 각 폴더에는 `model_0.pt`, `model_1.pt`, `model_2.pt`, `policy.onnx`, 설정 YAML, TensorBoard 이벤트가 실제로 있다. `git/`는 첫 번째 실행에 존재한다.

2026-10-07 문서 작성 시 기존 파일에 대한 읽기 전용 검사를 수행한 결과:

| 검사 | 결과 |
|---|---|
| 두 smoke의 최종 체크포인트 | 내부 `iter=2`; Actor·Critic state의 텐서에 NaN/Inf 없음 |
| 두 smoke의 정책 업데이트 | `model_0.pt`와 `model_2.pt` 사이 Actor MLP 가중치 변경 확인 |
| TensorBoard | 각 실행 36종 scalar tag, 기록 step 0·1·2의 값 유한 |
| 루트 `model_10000.pt` | 내부 `iter=10000`; Actor 입력 47, Critic 입력 74; 저장 state 텐서 유한 |
| ONNX 검사 | 두 smoke 및 실기용 ONNX 모두 `onnx.checker` 통과; 입력 `[1,47]`, 출력 `[1,12]` |
| 실기 추출 데이터 | `parity.npz`: 관측 `[100,47]`, 행동 `[100,12]`, 모두 유한값 |
| 추출물 출처 | `policy.json`의 체크포인트·ONNX SHA-256이 현재 파일과 일치 |
| 태스크 등록 | 현재 컨테이너에서 `scripts/list_envs.py`로 Flat/Rough 목록 확인 |
| 오프라인 실기 정책 검사 | 100프레임 통과; ONNX/원본 Actor 최대 오차 `8.344650268554688e-7`, 실기 관측 재구성 최대 오차 `1.6689300537109375e-6` |

오프라인 검사는 `scripts/check_go2_real_policy.py`로 수행했으며 상세 매핑과 해석은 [sim2real 문서](../sim2real/README.md)에 있다. 이는 입력 재구성과 추론 결과의 일치 검사다. 통신 지연, 모터 응답, 로봇의 실제 낙상률·보행 안정성까지 검증한 결과는 아니다.

Smoke는 학습 파이프라인이 기존 실행에서 동작했고 가중치가 변경되었다는 근거다. 192개의 transition으로 보행 성능이 충분히 학습되었다고 판단하지 않는다. 정책의 성능을 비교할 때는 체크포인트뿐 아니라 당시 환경·PPO YAML, 코드 변경 기록, 평가 조건과 속도 오차·낙상률·episode length·외란 회복 같은 결과를 함께 남겨야 한다. 원본 저장소의 `logs/`와 `artifacts/go2_real/`는 Git에서 제외되어 있으므로 새 머신에서는 원본 산출물을 별도로 확보하거나 재실행해야 한다.
