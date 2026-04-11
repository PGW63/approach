# approach_icp 현재 빌드 블로커 보고서

## 요약

현재 `fast_gicp` 연동 문제는 정리된 상태다.

- `fast_gicp` CUDA 12.6 빌드 성공
- `approach_icp`가 로컬 `fast_gicp` install prefix를 찾도록 CMake 연결 완료
- `registration`은 `fast_vgicp_cuda.hpp`를 보도록 수정 완료

지금 남아 있는 문제는 전부 `approach_icp` 내부 코드 정리다.

---

## 1. `tf_buffer_` 생성 방식이 ROS2 Humble과 맞지 않음

### 위치

- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L4)
- [node_icp_cuda.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_map_runner/node_icp_cuda.hpp#L60)

### 현재 문제

`tf2_ros::Buffer`는 기본 생성이 안 된다.
ROS2 Humble에서는 clock이 필요하다.

현재는:

```cpp
NodeICPCuda::NodeICPCuda() : Node("node_icp_cuda")
```

인데, 멤버에는:

```cpp
tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_{tf_buffer_};
```

가 있어서 생성자에서 바로 막힌다.

### 해결 방법

생성자 initializer list에서 명시적으로 초기화해야 한다.

예시 방향:

```cpp
NodeICPCuda::NodeICPCuda()
: Node("node_icp_cuda"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
```

그리고 헤더에서 `tf_listener_{tf_buffer_}` 같은 inline 초기화는 제거하는 편이 깔끔하다.

---

## 2. 서비스 콜백 바인딩 방식이 잘못됨

### 위치

- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L7)

### 현재 문제

현재는 멤버 함수 포인터만 넘기고 있다.

```cpp
accumulation_service_ = this->create_service<...>(
    service_config_.accumulation_service_name,
    &NodeICPCuda::service_callback
);
```

이건 `this` 객체와 바인딩되지 않아서 ROS2 서비스 callback 시그니처로 인식되지 않는다.

### 해결 방법

`std::bind` 또는 lambda로 `this`를 묶어야 한다.

예:

```cpp
std::bind(&NodeICPCuda::service_callback, this, std::placeholders::_1, std::placeholders::_2)
```

---

## 3. `create_publisher` / `create_subscription` 이름이 Node API를 가림

### 위치

- [node_icp_cuda.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_map_runner/node_icp_cuda.hpp#L30)
- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L66)
- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L74)

### 현재 문제

클래스에 아래 helper 함수들이 있다.

```cpp
void create_publisher();
void create_subscription();
```

그런데 구현 안에서 다시:

```cpp
this->create_publisher<sensor_msgs::msg::PointCloud2>(...)
this->create_subscription<sensor_msgs::msg::PointCloud2>(...)
```

를 호출하고 있어서, base class `rclcpp::Node`의 template 함수가 아니라
자기 자신의 non-template helper 함수 이름과 충돌한다.

그래서 `expected primary-expression before '>' token` 에러가 난다.

### 해결 방법

둘 중 하나로 정리하면 된다.

1. helper 함수 이름 변경

- `create_publisher()` -> `setup_publishers()`
- `create_subscription()` -> `setup_subscription()`

2. base class를 명시

```cpp
this->rclcpp::Node::create_publisher<...>(...)
this->rclcpp::Node::create_subscription<...>(...)
```

실무적으로는 1번이 더 읽기 쉽다.

---

## 4. `checkFrameId_only_Base_and_Map` 선언/호출이 서로 안 맞음

### 위치

- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L39)
- [types.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_icp/types.hpp#L71)
- [types.cpp](/home/gw/robocup_ws/src/approach_icp/src/types.cpp#L8)

### 현재 문제

헤더에는:

```cpp
bool checkFrameId(const std::string& frame_id, const FrameConfig& config);
```

만 선언돼 있다.

소스에는:

```cpp
bool checkFrameId_only_Base_and_Map(const std::string& frame_id, const FrameConfig& config)
```

가 구현돼 있다.

게다가 호출은:

```cpp
checkFrameId_only_Base_and_Map(request->target_frame, frame_id_)
```

로 되어 있는데, 두 번째 인자는 `FRAME_ID frame_id_`다.
실제 함수가 기대하는 타입은 `FrameConfig`다.

즉 문제는 세 가지다.

- 이름 불일치
- 선언 누락
- 인자 타입 불일치

### 해결 방법

가장 단순한 방향:

1. 헤더/소스 이름을 하나로 통일
2. 두 번째 인자는 `frame_id_`가 아니라 `frame_config_`를 넘김

즉 개념적으로는:

```cpp
checkFrameId_only_Base_and_Map(request->target_frame, frame_config_)
```

또는 함수 이름을 그냥 `checkFrameId`로 통일해도 된다.

---

## 5. `cloud_sub_.destroy()`는 ROS2 SharedPtr 방식과 맞지 않음

### 위치

- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L99)

### 현재 문제

`cloud_sub_`는 `SharedPtr`이고, 여기에 `.destroy()`는 없다.

### 해결 방법

구독 해제는 보통:

```cpp
cloud_sub_.reset();
```

으로 처리한다.

---

## 6. `preprocess` 네임스페이스 사용 준비가 안 되어 있음

### 위치

- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L144)
- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L169)
- [node_icp_cuda.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_map_runner/node_icp_cuda.hpp#L1)
- [preprocess.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_icp/preprocess.hpp#L11)

### 현재 문제

현재 `node_icp_cuda.cpp`는:

```cpp
approach_icp::preprocess::downsample(...)
```

를 호출한다.

그런데 `node_icp_cuda.hpp` 쪽에는 `preprocess.hpp`가 포함되지 않았고,
`preprocess_config_` 멤버도 선언돼 있지 않다.

그래서 다음 두 에러가 함께 난다.

- `approach_icp::preprocess` not declared
- `preprocess_config_` not declared

### 해결 방법

1. `node_icp_cuda.hpp` 또는 `node_icp_cuda.cpp`에:

```cpp
#include "approach_icp/preprocess.hpp"
```

추가

2. 클래스 멤버에:

```cpp
PreprocessConfig preprocess_config_;
```

추가

---

## 7. 로그 매크로 오타

### 위치

- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L210)

### 현재 문제

현재:

```cpp
RCLCPP_Error(...)
```

인데 ROS2 매크로는:

```cpp
RCLCPP_ERROR(...)
```

이다.

### 해결 방법

대문자 매크로 이름으로 수정.

---

## 8. 빌드는 막지 않지만 바로 같이 손보는 게 좋은 항목

### 8-1. `hz_` 초기값

- [node_icp_cuda.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_map_runner/node_icp_cuda.hpp#L63)

현재 초기값이 없다.
잘못된 서비스 요청이 왔을 때 로그에서 쓰거나 타이머 주기 계산 전에 값이 애매할 수 있다.

권장:

```cpp
float hz_ = 1.0f;
```

같이 기본값 부여.

### 8-2. `target_frame_` 기본값

- [node_icp_cuda.hpp](/home/gw/robocup_ws/src/approach_icp/include/approach_map_runner/node_icp_cuda.hpp#L64)

권장:

```cpp
std::string target_frame_ = "map";
```

또는 `frame_config_.map_frame` 기준으로 시작.

### 8-3. `publish_cloud()`의 `frame_id` / `topic_name` 문자열 인터페이스

- [node_icp_cuda.cpp](/home/gw/robocup_ws/src/approach_map_runner/src/node_icp_cuda.cpp#L176)

문자열 기반이라 실수 여지가 있다.
빌드가 먼저지만, 이후에는:

- `publish_aligned_cloud()`
- `publish_accumulated_cloud()`

로 분리하면 더 안전하다.

---

## 추천 수정 순서

1. `tf_buffer_`, `tf_listener_` 생성자 초기화 정리
2. 서비스 콜백 바인딩 수정
3. `create_publisher` / `create_subscription` 이름 충돌 정리
4. `checkFrameId_only_Base_and_Map` 선언/이름/인자 통일
5. `cloud_sub_.reset()`로 정리
6. `preprocess.hpp` include + `PreprocessConfig preprocess_config_` 추가
7. `RCLCPP_ERROR` 오타 수정

이 순서대로 가면 `node_icp_cuda.cpp` 쪽 빌드 에러 대부분이 한 번에 정리될 가능성이 크다.

---

## 한 줄 결론

지금 남은 문제는 전부 `node_icp_cuda`의 ROS2 클래스 구성과 선언 정리 문제다.
`fast_gicp` 연동은 이미 통과했고, 이제는 노드 클래스 자체를 ROS2 Humble 방식에 맞게 다듬는 단계다.
