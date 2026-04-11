# fast_gicp 연동 진행 보고서

## 이번에 반영한 것

### 1. fast_gicp CUDA 12.6 대응

다음 헤더에서 오래된 `thrust` 전방 선언을 제거하고, 필요한 Thrust 헤더를 직접 include 하도록 수정했다.

- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/ndt_cuda.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/fast_vgicp_cuda.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/gaussian_voxelmap.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/compute_derivatives.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/brute_force_knn.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/find_voxel_correspondences.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/ndt_compute_derivatives.cuh`

결과:

- `fast_gicp` 단독 `make -j4` 성공
- `libfast_vgicp_cuda.so` 링크 성공
- `libfast_gicp.so` 링크 성공

### 2. fast_gicp를 local install prefix로 설치

수동 빌드한 `fast_gicp`를 아래 경로로 install 했다.

- `/home/gw/robocup_ws/src/fast_gicp/local_install`

즉, `COLCON_IGNORE`는 유지하면서도 `approach_icp`가 `find_package(fast_gicp)`로 잡을 수 있는 install tree를 만든 상태다.

### 3. approach_icp CMake에서 local fast_gicp 자동 탐색 추가

아래 경로를 자동으로 먼저 보도록 추가했다.

- [CMakeLists.txt](/home/gw/robocup_ws/src/approach_icp/CMakeLists.txt#L26)

현재 로직은:

- `../fast_gicp/local_install/share/fast_gicp/cmake/fast_gicpConfig.cmake`가 있으면 그걸 사용
- 없으면 기존 `find_package(fast_gicp REQUIRED)` fallback

## 현재 상태

`fast_gicp` 연동 문제는 정리됐다.

즉 지금 `approach_icp` 빌드가 막히는 이유는 더 이상 `fast_gicp`를 못 찾아서가 아니라,
`approach_icp` 코드 자체의 컴파일 오류 때문이다.

## 현재 남은 approach_icp 컴파일 에러

### 1. PCL PointCloud 선언에 필요한 헤더 누락

다음 위치에서 `pcl::PointCloud<...>`를 쓰는데 `pcl/point_cloud.h`가 포함되지 않아 컴파일이 깨진다.

- [types.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_icp/types.hpp#L49)
- [tf_utils.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_icp/tf_utils.hpp#L30)

현재 `types.hpp`는:

- `#include <pcl/point_types.h>`만 있음

그런데 `pcl::PointCloud` 템플릿 자체는 `pcl/point_cloud.h`가 필요하다.

### 2. passthroughFilter가 반환형과 구현이 맞지 않음

- [preprocess.cpp](/home/gw/robocup_ws/src/approach_icp/src/preprocess.cpp#L65)

문제:

```cpp
return;
```

반환형은 `pcl::PointCloud<pcl::PointXYZ>::Ptr`인데 값 없이 `return` 하고 있다.

즉 이 함수는 아직 TODO 상태라서 빌드가 깨진다.

### 3. fast_gicp 타입 이름이 현재 설치된 라이브러리와 안 맞음

- [registration.cpp](/home/gw/robocup_ws/src/approach_icp/src/registration.cpp#L14)

현재 코드:

```cpp
fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp;
```

그런데 현재 설치된 `fast_gicp` 쪽에서는 이 이름이 없고, 빌드 로그 기준으로는 `FastVGICP`만 잡힌다.

즉 `approach_icp`가 기대하는 API와 현재 `fast_gicp` API가 다르다.

이건 다음 둘 중 하나가 필요하다.

- `approach_icp` 쪽을 현재 `fast_gicp` API에 맞춰 바꾸기
- 또는 `fast_gicp` 쪽 버전을 `approach_icp`가 기대하는 API 버전으로 맞추기

### 4. RegistrationResult 멤버 사용 불일치

- [registration.cpp](/home/gw/robocup_ws/src/approach_icp/src/registration.cpp#L29)
- [types.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_icp/types.hpp#L45)

로그 기준으로는 `RegistrationResult`에 `aligned_cloud`가 없다고 나왔지만,
현재 헤더에는 `aligned_cloud`가 선언돼 있다.

이건 캐시/헤더 포함 순서 문제였을 가능성도 있고, 위의 PCL 헤더 누락 때문에 구조체 해석이 망가지면서 연쇄적으로 나타난 에러일 가능성이 크다.

즉 1번을 먼저 고치고 다시 보는 게 맞다.

## 우선순위 추천

1. `types.hpp`, `tf_utils.hpp`에 `pcl/point_cloud.h` 추가
2. `preprocess.cpp`의 `passthroughFilter()`를 임시라도 반환값 있게 정리
3. `fast_gicp::FastVGICPCuda` 타입명을 현재 설치된 `fast_gicp` API와 맞춰 확인
4. 그다음 다시 `colcon build --packages-select approach_icp --cmake-clean-cache`

## 한 줄 요약

지금은 `fast_gicp` CUDA 12.6 연동은 끝났고,
남은 건 `approach_icp` 코드 자체의 헤더/API 불일치 정리 단계다.
