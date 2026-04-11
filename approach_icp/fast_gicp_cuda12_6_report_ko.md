# fast_gicp CUDA 12.6 대응 보고서

## 현재 결론

`/usr/include/thrust`, `/usr/include/cub`를 치운 뒤에도 CUDA 빌드가 계속 깨지는 이유는,
환경 중복 문제만이 아니라 `fast_gicp`의 CUDA 헤더 일부가 **오래된 Thrust 전방 선언 방식**을 사용하고 있기 때문으로 보인다.

특히 CUDA 12.x 계열의 Thrust는 `thrust` 내부에 ABI namespace를 사용한다.
그런데 `fast_gicp`는 바깥 `namespace thrust`에 직접 타입을 forward declaration 하고 있어서,
실제 Thrust 타입과 다른 선언이 생기고, 그 결과 아래와 같은 에러가 이어진다.

- `thrust::pair is ambiguous`
- `too few arguments for class template "thrust::device_vector"`
- `incomplete type ... thrust::device_vector ... is not allowed`

## 가장 의심되는 파일

### 1. `include/fast_gicp/cuda/ndt_cuda.cuh`

문제 구간:

```cpp
namespace thrust {
template <typename T1, typename T2>
class pair;

template <typename T>
class device_allocator;

template <typename T, typename Alloc>
class device_vector;
}
```

이 전방 선언은 CUDA 12.6의 최신 Thrust 구현과 맞지 않을 가능성이 크다.

### 2. `include/fast_gicp/cuda/fast_vgicp_cuda.cuh`

위와 동일한 형태의 `thrust` 전방 선언이 들어 있다.

## 왜 이게 문제인가

CUDA 12.6의 Thrust는 단순히 `namespace thrust { ... }`만 있는 구조가 아니라,
내부 ABI namespace를 함께 사용한다.

즉 `fast_gicp`가 직접 적어둔:

```cpp
namespace thrust {
template <typename T, typename Alloc>
class device_vector;
}
```

이 선언은 겉보기엔 비슷하지만 실제 라이브러리 타입과 정확히 같은 선언이 아니다.

그래서 이후 `.cu` 파일에서 `<thrust/device_vector.h>`를 include 했을 때:

- `fast_gicp`가 생각하는 `thrust::device_vector`
- CUDA 12.6 Thrust가 제공하는 실제 `thrust::device_vector`

가 섞여서 모호성, 불완전 타입, 템플릿 인자 수 불일치 같은 에러가 터진다.

## 우선 수정 방향

### A. 잘못된 전방 선언 제거

아래 두 파일에서 `namespace thrust { ... }` 블록을 제거하는 게 1순위다.

- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/ndt_cuda.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/fast_vgicp_cuda.cuh`

### B. 실제 Thrust 헤더를 명시적으로 include

두 파일 상단에 아래 헤더들을 명시적으로 넣는 방향이 안전하다.

```cpp
#include <thrust/device_allocator.h>
#include <thrust/device_vector.h>
#include <thrust/pair.h>
```

필요 시 `device_ptr`를 쓰는 파일은 아래도 추가:

```cpp
#include <thrust/device_ptr.h>
```

## 추가로 같이 정리하면 좋은 파일

CUDA 12.6에서 간접 include에 기대지 않도록, `thrust::pair`를 쓰는 헤더는 `thrust/pair.h`를 직접 include 하는 편이 좋다.

후보 파일:

- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/gaussian_voxelmap.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/compute_derivatives.cuh`
- `/home/gw/robocup_ws/src/fast_gicp/include/fast_gicp/cuda/brute_force_knn.cuh`

현재 이 파일들은 `thrust::pair`를 쓰면서 `thrust/pair.h`를 직접 include 하지 않거나,
간접 include에 기대는 형태다.

## 예상 수정안 요약

### `ndt_cuda.cuh`

현재:

```cpp
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fast_gicp/ndt/ndt_settings.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>

namespace thrust {
...
}
```

권장:

```cpp
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <thrust/device_allocator.h>
#include <thrust/device_vector.h>
#include <thrust/pair.h>

#include <fast_gicp/ndt/ndt_settings.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>
```

그리고 `namespace thrust { ... }` 블록 제거.

### `fast_vgicp_cuda.cuh`

동일하게:

- `namespace thrust { ... }` 전방 선언 제거
- `thrust/device_allocator.h`
- `thrust/device_vector.h`
- `thrust/pair.h`

추가

### `compute_derivatives.cuh`

`thrust::pair<int, int>`를 사용하므로:

```cpp
#include <thrust/pair.h>
```

를 직접 추가하는 편이 안전하다.

### `gaussian_voxelmap.cuh`

`thrust::pair<Eigen::Vector3i, int>`를 사용하므로:

```cpp
#include <thrust/pair.h>
```

를 직접 추가하는 편이 안전하다.

### `brute_force_knn.cuh`

`thrust::pair<float, int>`를 사용하므로:

```cpp
#include <thrust/pair.h>
```

를 직접 추가하는 편이 안전하다.

## 빌드 시스템 쪽 참고

`fast_gicp/CMakeLists.txt`는 CUDA 쪽에서 예전 스타일인:

- `find_package(CUDA REQUIRED)`
- `cuda_add_library(...)`

를 사용한다.

이 자체가 당장 주원인이라고 단정할 수는 없지만,
CUDA 12.6에서는 헤더/ABI 변화에 민감하므로 헤더 수정 후에도 문제가 남으면 이 부분도 확인 대상이다.

다만 현재 로그만 보면 1차 원인은 CMake보다는 **Thrust 타입 선언 충돌** 쪽이다.

## 지금 상태에서 가장 유력한 패치 순서

1. `ndt_cuda.cuh`와 `fast_vgicp_cuda.cuh`에서 `thrust` 전방 선언 제거
2. 두 파일에 `thrust/device_allocator.h`, `thrust/device_vector.h`, `thrust/pair.h` 직접 include
3. `thrust::pair` 쓰는 다른 `.cuh`들에 `thrust/pair.h` 직접 include
4. `fast_gicp` 단독 빌드 재시도
5. 남는 에러가 있으면 그다음 CMake/CUDA 플래그 쪽 점검

## 메모

이번 분석 기준으로는, 처음의 `/usr/include/thrust`, `/usr/include/cub` 중복 문제는 실제로 있었고 제거할 가치가 있었다.
하지만 그걸 치운 뒤에도 남는 에러는 `fast_gicp` 코드 자체가 CUDA 12.6 Thrust 구조를 전제로 작성되지 않았다는 쪽으로 보는 게 맞다.

즉 현재 결론은:

- 중복 Thrust 문제: 일부 있었음
- 남은 핵심 문제: `fast_gicp` CUDA 헤더의 오래된 `thrust` 전방 선언 방식
