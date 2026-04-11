# `approach_icp`용 `fast_gicp` 로컬 설치/연동 가이드

이 문서는 현재 이 프로젝트의 CMake 설정을 기준으로,

- `fast_gicp`를 워크스페이스 안에 두되
- `colcon`으로 직접 같이 빌드하지 않고
- 로컬 `install prefix`로 따로 설치한 뒤
- `approach_icp`가 그 위치를 자동으로 찾게

하는 방법을 정리한 문서입니다.

현재 `approach_icp`는 아래 경로를 우선 찾도록 되어 있습니다.

```text
~/robocup_ws/src/approach_icp
~/robocup_ws/src/fast_gicp
~/robocup_ws/src/fast_gicp/local_install
```

즉, `approach_icp`와 `fast_gicp`가 **형제 폴더**에 있어야 가장 편합니다.

---

## 1. 권장 디렉토리 구조

```text
~/robocup_ws/
  src/
    approach_icp/
    fast_gicp/
```

`approach_icp/CMakeLists.txt`는 기본적으로 아래 위치를 먼저 찾습니다.

```text
../fast_gicp/local_install
```

그래서 `fast_gicp`를 다른 위치에 두면 추가 수정이 필요합니다.

---

## 2. `fast_gicp` 다운로드

`~/robocup_ws/src` 아래에서 `fast_gicp`를 받습니다.

```bash
cd ~/robocup_ws/src
git clone <사용할 fast_gicp 저장소 주소> fast_gicp
```

주의:

- 가능하면 **지금 이 프로젝트에서 이미 검증한 `fast_gicp` 소스**를 그대로 쓰는 것을 권장합니다.
- upstream 원본을 바로 쓰는 경우, CUDA/Thrust/CUDA architecture 관련 추가 수정이 필요할 수 있습니다.

---

## 3. `COLCON_IGNORE` 유지

이 프로젝트는 `fast_gicp`를 ROS 패키지처럼 `colcon`에 같이 태우는 방식이 아닙니다.

즉, `fast_gicp` 폴더에는 `COLCON_IGNORE`를 유지하는 편이 좋습니다.

```bash
touch ~/robocup_ws/src/fast_gicp/COLCON_IGNORE
```

이렇게 하면 `colcon build`를 할 때 `fast_gicp`를 워크스페이스 패키지로 직접 빌드하지 않습니다.

---

## 4. `fast_gicp`를 standalone으로 빌드/설치

### 4-1. CPU only 빌드

CUDA를 쓰지 않을 경우:

```bash
cd ~/robocup_ws/src/fast_gicp
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_VGICP_CUDA=OFF \
  -DCMAKE_INSTALL_PREFIX=$PWD/local_install

cmake --build build -j$(nproc)
cmake --install build
```

### 4-2. CUDA 빌드

CUDA를 쓸 경우:

```bash
cd ~/robocup_ws/src/fast_gicp
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_VGICP_CUDA=ON \
  -DCMAKE_INSTALL_PREFIX=$PWD/local_install

cmake --build build -j$(nproc)
cmake --install build
```

### 4-3. Jetson AGX Orin 같은 타겟에서 CUDA architecture를 명시하고 싶을 때

Jetson AGX Orin 계열이면 보통 `sm_87` 기준으로 빌드합니다.

```bash
cd ~/robocup_ws/src/fast_gicp
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_VGICP_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=87 \
  -DCMAKE_INSTALL_PREFIX=$PWD/local_install

cmake --build build -j$(nproc)
cmake --install build
```

데스크탑 GPU면 그 머신에 맞는 architecture로 바꿔야 합니다.

예:

- Ada: `89`
- Ampere 일부: `86`
- Orin: `87`

---

## 5. 설치 결과 확인

아래 파일이 있어야 `approach_icp`가 자동으로 찾을 수 있습니다.

```bash
ls ~/robocup_ws/src/fast_gicp/local_install/share/fast_gicp/cmake/fast_gicpConfig.cmake
ls ~/robocup_ws/src/fast_gicp/local_install/lib/libfast_gicp.so
```

CUDA 버전이면 보통 이것도 같이 생깁니다.

```bash
ls ~/robocup_ws/src/fast_gicp/local_install/lib/libfast_vgicp_cuda.so
```

---

## 6. `approach_icp` 빌드

이제 `approach_icp`만 빌드하면 됩니다.

```bash
cd ~/robocup_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select approach_icp
source ~/robocup_ws/install/setup.bash
```

현재 `approach_icp/CMakeLists.txt`는

```text
~/robocup_ws/src/fast_gicp/local_install
```

를 자동으로 찾도록 되어 있어서, 별도의 `-Dfast_gicp_DIR=...`를 매번 줄 필요가 없습니다.

---

## 7. 실행

예:

```bash
source ~/robocup_ws/install/setup.bash
ros2 run approach_map_runner node_icp_cuda --ros-args -p use_sim_time:=true
```

현재 CMake에는 `fast_gicp/local_install/lib` 쪽 RPATH도 들어가 있으므로,
정상 설치되었다면 `LD_LIBRARY_PATH`를 따로 수동으로 넣지 않아도 실행되는 구성을 목표로 하고 있습니다.

---

## 8. 경로를 바꾸고 싶을 때

만약 `fast_gicp`를 형제 폴더가 아닌 다른 위치에 둘 거면,
`approach_icp/CMakeLists.txt`의 아래 변수 부분을 바꿔야 합니다.

```cmake
set(FAST_GICP_LOCAL_PREFIX
  "${CMAKE_CURRENT_SOURCE_DIR}/../fast_gicp/local_install"
)
```

예를 들어 다른 절대경로를 쓰고 싶으면:

```cmake
set(FAST_GICP_LOCAL_PREFIX "/some/other/path/fast_gicp/local_install")
```

이후 `colcon build --packages-select approach_icp`를 다시 하면 됩니다.

---

## 9. 트러블슈팅

### 9-1. `Could not find fast_gicpConfig.cmake`

보통 아래 중 하나입니다.

- `fast_gicp`를 아직 `cmake --install` 하지 않음
- `local_install`이 아니라 `build`만 있음
- `approach_icp`와 `fast_gicp`가 형제 폴더 구조가 아님

확인:

```bash
ls ~/robocup_ws/src/fast_gicp/local_install/share/fast_gicp/cmake/fast_gicpConfig.cmake
```

### 9-2. `libfast_gicp.so` 또는 `libfast_vgicp_cuda.so`를 못 찾음

대부분 아래 중 하나입니다.

- `fast_gicp` 설치가 불완전함
- `approach_icp`를 `fast_gicp` 설치 전에 빌드했음

해결:

1. `fast_gicp` 다시 `cmake --install`
2. `approach_icp` 다시 빌드

```bash
cd ~/robocup_ws
colcon build --packages-select approach_icp
source ~/robocup_ws/install/setup.bash
```

### 9-3. CUDA 빌드는 되는데 런타임에 PTX/toolchain 오류가 남

주로 아래가 원인입니다.

- CUDA architecture가 타겟 GPU와 안 맞음
- 다른 머신에서 빌드한 바이너리를 그대로 가져옴

가장 안전한 방법은 **실제로 돌릴 머신에서 직접 빌드**하는 것입니다.

Jetson AGX Orin이면 보통:

```bash
-DCMAKE_CUDA_ARCHITECTURES=87
```

를 명시하는 편이 안전합니다.

---

## 10. 권장 요약

현재 프로젝트 기준으로 가장 추천하는 방식은 아래입니다.

1. `~/robocup_ws/src/fast_gicp`에 소스를 둔다.
2. `COLCON_IGNORE`는 유지한다.
3. `fast_gicp`를 standalone으로 `local_install`에 설치한다.
4. `approach_icp`는 `colcon build --packages-select approach_icp`만 한다.
5. `approach_icp`가 `../fast_gicp/local_install`를 자동으로 찾게 둔다.

이 방식이 가장 덜 번거롭고, 매번 `fast_gicp_DIR`를 커맨드라인에 직접 주지 않아도 됩니다.
