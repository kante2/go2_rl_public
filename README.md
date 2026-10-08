# Go2 RL 작업 기록

`~/go2_rl`에서 진행하는 Unitree Go2 강화학습, 시뮬레이션, 실제 로봇 제어와 sim2real 연결을 정리하는 문서 저장소다. 코드·체크포인트·모델·실행 로그는 `~/go2_rl`에서 관리하며, 여기에는 현재 구현과 확인 결과를 한국어로 기록한다.

## Progress Videos

Videos documenting policy deployment and robot experiments. Latest updates appear first.

### 2026-10-07 — Learned Locomotion Policy Deployed on the Real Robot

Synchronized side-by-side footage of the MuJoCo visualization and the real Go2 robot.

[![MuJoCo visualization and real Go2 robot](video/2026-10-07-learned-locomotion-real-robot-preview.jpg)](https://youtu.be/tbz4j0fWV4E?si=EWEwPF1a_iUDlggj)

[▶ Watch on YouTube](https://youtu.be/tbz4j0fWV4E?si=EWEwPF1a_iUDlggj)

## 문서 구성

| 폴더 | 기록하는 내용 |
|---|---|
| [rl](rl/README.md) | mjlab 환경, Actor/Critic 관측, 행동과 보상, PPO 학습, 체크포인트와 현재 학습 확인 상태 |
| [real](real/README.md) | 실제 Go2 센서 피드백, 호스트 ONNX 제어, 키보드·DDS·상태 전환·복귀 절차 |
| [simulator](simulator/README.md) | 학습·재생, ROS 2 내비게이션 시뮬레이션, C++ MuJoCo DDS 브리지의 역할과 실행 방법 |
| [sim2real](sim2real/README.md) | 정책 관절 ↔ SDK 모터 매핑, 47차원 관측, 행동·PD·시간 간격, 정책 내보내기와 대응 검증 |

각 문서에 나오는 `scripts/...`, `src/...`, `deploy/...` 등의 경로와 실행 명령은 **원본 작업 폴더 `~/go2_rl` 기준**이다. 이 문서 저장소를 복제한 위치에서 제어·학습 명령을 실행하려면 원본 코드와 해당 실행 환경을 별도로 준비해야 한다.

## 2026-10-08 업데이트

오늘 원본의 `03921ca` (`25_1008_REFACTORING_go2_autodrive`)와 `472850e` (`1008_refactor_2`)를 반영했다. 이번 변경의 중심은 코드 구조와 실행 환경 정리이며, 기존 보행 모델을 새로 학습한 것으로 기록하지 않는다.

| 항목 | 현재 구성 |
|---|---|
| 보행 학습 | `src/locomotion_rl/go2_baseline/`에서 Flat/Rough 태스크를 등록한다. `scripts/train.py`, `scripts/play.py`의 CLI는 유지된다. |
| 새 시뮬레이션 패키지 | `src/go2_autodrive/`에 UDP↔ROS 브리지, 키보드, MuJoCo 정책 실행기와 launch가 있다. 기존 `scripts/run_go2_simulation.sh`는 `go2_autonomous_driving.sim.simulation`을 실행하므로 두 경로의 명령을 [시뮬레이터 문서](simulator/README.md)에 구분했다. |
| 실기 제어 | `src/go2_autonomous_driving/go2_autonomous_driving/real/`로 구현을 모았다. 실제 피드백 제어기와 키보드는 호스트에서, 측정 자세 뷰어는 컨테이너에서 실행한다. |
| Docker | `docker/Dockerfile`로 `go2-rl:local`을 빌드한다. ROS 플래너·시뮬레이션·학습을 같은 컨테이너에서 실행하며 ROS Python 3.12와 학습 Python 3.11을 분리한다. |
| 체크포인트 | 실기·기존 시뮬레이션은 `src/go2_autonomous_driving/model_pt/model_10000.pt`, 새 시뮬레이션은 `src/go2_autodrive/config/model_pt/model_10000.pt`를 사용한다. 두 파일의 SHA-256은 같고 루트의 `model_10000.pt`는 현재 없다. |

## 기록 기준

- 확인일: **2026-10-08, Asia/Seoul**.
- 원본 작업 경로: `~/go2_rl`.
- 원본 Git 기준 커밋: `472850eebd2194eb71141b1349087145e4974f30` (`1008_refactor_2`). 이전 기록의 기준은 `35605c5c01887d7ab099736c357536e5006d64e6`이다.
- 확인 당시 원본 Git 작업 트리는 깨끗했으며 변경된 추적 파일이나 미추적 파일은 없었다. 이전 문서에 적었던 로컬 수정·추가 항목은 오늘 커밋의 소스·설정과 대조했다.
- `.gitignore`로 제외된 `artifacts/go2_real/`, `logs/`, `log/`와 호스트 `.venv_go2_real`, 실행 중인 Docker 환경도 확인했다. 이 산출물·의존성은 원본 커밋만으로 모두 재현되지 않으며 공개 문서 저장소에도 포함하지 않는다.

기존 `kante_.md`, `sim2real_mapping.md`의 과거 설명은 현재 소스·설정·산출물과 대조했다. 설정값, 저장된 로그 결과, 이번에 직접 실행한 검증 결과를 구분해 적었다.

## 현재 상태

현재 실기용 정책은 Flat의 **Actor 47차원 → 행동 12차원**이며, 정책 주기는 **50 Hz**다. Python 실기 폐루프는 실제 IMU·관절 상태를 받아 호스트에서 ONNX를 추론하고 **500 Hz 설정의 LowCmd 루프**로 목표 관절각을 전달한다. 시뮬레이션 내비게이션과 실제 로봇 제어, 실측 상태 뷰어는 각각 독립된 경로다.

오늘 문서 갱신 중 로봇 통신이나 제어를 시작하지 않고 호스트 `.venv_go2_real`에서 `scripts/check_go2_real_policy.py`의 저장 데이터 검증을 다시 실행했다. 100프레임에서 ONNX/Torch 출력 최대 오차는 `8.344650268554688e-07`, 관측 매핑 최대 오차는 `1.6689300537109375e-06`으로 기존 `1e-4` 기준을 통과했다. 이는 현재 정책 파일과 관측 변환의 일치성을 확인한 결과다.

현재 컨테이너에서 `scripts/check_go2_docker.py --require-cuda`도 통과했다. ROS 플래너 조회, 새 `src.locomotion_rl` 태스크 등록, Go2 XML 로딩·EGL 렌더링, Torch/Warp의 GPU 할당을 확인했다. 새 `go2_autodrive` 패키지 등록, MuJoCo 실행기의 CLI와 기본 설정·체크포인트 경로도 물리 실행 없이 확인했다.

저장된 `log/docker_ros/validation.json`에는 오늘 수행한 컨테이너 통합 시험의 1,500 정책 스텝, 약 **4.9213 m** 전진과 Space 정지, 16개 환경·2회 PPO 업데이트, 가상 측정 자세 138프레임 뷰어 수신이 기록되어 있다. 이 로그는 최종 코드 이동 이전의 기존 패키지 실행 결과이며, 이번 문서 갱신에서는 학습·내비게이션·실기 보행을 새로 실행하지 않았다. 최신 구조의 전체 통합 재검증, 실기 보행 안정성과 고속 명령의 관절 한계 문제는 별도 확인 대상이다.

현재 C++ 배포 소스는 `deploy/include/unitree_articulation.h` 끝의 셸 명령문이 **원본 기준 커밋에도 포함**되어 있고, 기울기 종료 검사도 항상 `false`를 반환한다. Python 제어 경로의 검사와 구분해 [실기 문서](real/README.md)와 [매핑 문서](sim2real/README.md)에 설명했다.

## 기록 갱신 방법

원본의 환경·관측·행동·학습 설정이 바뀌면 `rl/`을, 실제 실행·네트워크·제어 전환이 바뀌면 `real/`을, 시뮬레이터·센서·내비게이션 구성이 바뀌면 `simulator/`를 갱신한다. 정책이나 로봇 모델을 교체할 때는 `sim2real/`의 관절 순서·기본 자세·관측 순서·배율·PD·주기를 함께 확인한다. 확인일과 검증에 사용한 모델·설정·로그 출처도 남긴다.
