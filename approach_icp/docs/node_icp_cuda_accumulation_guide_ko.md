# `node_icp_cuda` 누적 맵 구조 가이드

이 문서는 현재 `node_icp_cuda` 코드를 직접 수정하지 않고,
"어떤 부분을 왜 고쳐야 하는지"를 읽기 쉽게 정리한 설명 파일이다.

목표는 다음과 같다.

1. Livox 포인트클라우드를 받는다.
2. TF로 `target_frame` 기준 초기 위치를 잡는다.
3. 현재 스캔을 기존 누적 맵에 정합한다.
4. 정합된 현재 스캔을 누적 맵에 합친다.
5. 누적 맵을 계속 퍼블리시한다.

---

## 1. 현재 코드에서 가장 핵심적인 문제

현재 코드의 가장 큰 문제는 "누적에 필요한 상태"와 "정합 처리 흐름"이 서로 맞지 않는다는 점이다.

특히 아래가 중요하다.

- `cloud_callback()`에서는 최신 cloud만 저장하고 있다.
- 그런데 `process_callback()`에서는 TF lookup에 필요한 `stamp`가 필요하다.
- 이 프로젝트 전제에서는 source frame은 메시지 header가 아니라 `frame_config_.sensor_frame` 고정값을 사용한다.
- 현재 스캔을 어디 기준으로 정합할지 source/target 방향도 뒤집혀 있다.
- 정합 결과를 누적 맵에 실제로 합치는 코드가 없다.
- downsample 결과를 다시 저장하지 않아서 누적 맵이 줄어들지 않는다.

즉, 지금 상태는 "누적 맵을 만드는 구조"라기보다는
"현재 스캔 하나를 가지고 뭔가 처리하려는 구조"에 더 가깝다.

---

## 2. 먼저 머릿속에서 정리해야 하는 흐름

누적 맵은 아래 순서로 생각하면 가장 깔끔하다.

```text
센서 스캔 수신
  -> 최신 스캔 저장
  -> 최신 스캔의 timestamp 저장
주기적으로 process 실행
  -> target <- frame_config_.sensor_frame TF 조회
  -> TF를 초기 guess로 사용
  -> 현재 스캔을 기존 누적 맵에 정합
  -> 정합된 현재 스캔을 누적 맵에 merge
  -> 누적 맵 downsample
  -> 현재 정합 결과 퍼블리시
  -> 누적 맵 퍼블리시
```

이 구조에서 역할은 이렇게 나뉜다.

- `cloud_callback()`: 데이터 저장만 담당
- `process_callback()`: TF, ICP, merge, publish 담당

이렇게 나누는 이유는 입력 callback이 너무 무거워지지 않게 하기 위해서다.

---

## 3. 왜 `stamp`를 저장해야 하는가

현재 전제에서는 source frame은 항상 `frame_config_.sensor_frame`이다.
즉 메시지의 `frame_id`를 따로 저장할 필요는 없다.
하지만 TF lookup은 cloud 자체만으로는 할 수 없고, 최소한 스캔 시각은 필요하다.

TF lookup에 필요한 정보는 최소 2개다.

1. 이 스캔이 언제 찍혔는가
2. 어느 프레임으로 옮길 것인가

즉, `cloud_callback()`에서 최소한 이것들을 저장해야 한다.

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
rclcpp::Time latest_stamp_;
bool has_latest_cloud_;
```

왜 필요한가:

- `latest_cloud_`: 실제 포인트 데이터
- `latest_stamp_`: 그 시점의 TF를 조회하기 위해 필요
- `has_latest_cloud_`: 지금 처리할 새 데이터가 있는지 구분하기 위해 필요

특히 `stamp`가 중요한 이유는,
로봇이 움직이는 중이면 TF가 시간에 따라 달라지기 때문이다.
`now()`를 쓰면 "스캔이 찍힌 시각"이 아니라 "지금 시각"이 되어 오차가 생길 수 있다.

---

## 4. `cloud_callback()`은 저장만 하는 게 좋다

추천 형태는 아래와 같다.

```cpp
void NodeICPCuda::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    latest_cloud_->clear();
    pcl::fromROSMsg(*msg, *latest_cloud_);

    latest_stamp_ = msg->header.stamp;
    has_latest_cloud_ = !latest_cloud_->empty();
}
```

이렇게 하는 이유:

- 입력 callback은 자주 들어오므로 가볍게 유지하는 게 좋다.
- TF 조회와 ICP는 시간이 들 수 있으니 타이머 쪽에서 처리하는 편이 구조가 깔끔하다.
- 이 프로젝트 전제에서는 source frame을 `frame_config_.sensor_frame` 고정값으로 두면 된다.

---

## 5. `process_callback()`에서 가장 먼저 해야 할 일

`process_callback()`은 시작하자마자 "처리할 최신 데이터가 있는가"를 먼저 봐야 한다.

추천 형태:

```cpp
if (!has_latest_cloud_ || latest_cloud_->empty()) {
    return;
}
```

왜 이렇게 해야 하는가:

- 새 데이터가 없으면 TF lookup과 ICP를 할 이유가 없다.
- 포인터가 살아 있어도 내부 cloud가 비어 있을 수 있다.
- "포인터 존재 여부"와 "데이터 존재 여부"는 구분해서 생각해야 한다.

현재 코드처럼 처리 후 `latest_cloud.reset()`을 해버리면,
다음 callback에서 null 포인터를 건드릴 위험이 있다.
그래서 reset으로 포인터 자체를 없애기보다는,

- `latest_cloud_->clear()`
- `has_latest_cloud_ = false`

같은 방식이 더 안전하다.

---

## 6. TF는 어디에 어떻게 써야 하는가

TF는 현재 스캔을 `target_frame` 기준으로 옮기기 위한 "초기 위치 추정값"으로 쓰는 게 가장 자연스럽다.

예를 들면:

```cpp
auto tf_msg = tf2_utils::lookupTransform(
    tf_buffer_,
    target_frame_,
    frame_config_.sensor_frame,
    latest_stamp_,
    rclcpp::Duration::from_seconds(0.2));

Eigen::Matrix4f initial_guess =
    tf2_utils::transformToMatrix(tf_msg);
```

이 코드가 의미하는 것:

- `latest_frame_id_`에 있는 점들을
- `target_frame_` 기준으로 옮기는 변환을 가져온다.
- 그 결과를 4x4 행렬로 바꿔 ICP의 초기 guess로 사용한다.

수학적으로는:

```text
p_target = T_target_source * p_source
```

즉 `lookupTransform(target, source, ...)`는
"source 좌표의 점을 target 좌표로 옮기는 transform"이라고 이해하면 된다.

---

## 7. TF를 두 번 적용하면 안 된다

현재 코드에서 가장 조심해야 할 부분 중 하나가 이거다.

아래 패턴은 좋지 않다.

```cpp
transformed_cloud = transform(latest_cloud_, initial_guess);
result = registerPointClouds_with_cuda(
    transformed_cloud,
    accumulated_cloud_,
    initial_guess,
    ...);
```

왜 문제인가:

- 이미 `transformed_cloud`를 만들 때 `initial_guess`를 한 번 적용했다.
- 그런데 registration에도 같은 `initial_guess`를 또 넘기면,
  같은 의미의 초기 위치 정보를 두 번 반영하는 꼴이 된다.

그래서 둘 중 하나만 선택해야 한다.

### 방법 A

원본 scan은 그대로 두고,
ICP에만 `initial_guess`를 넘긴다.

```cpp
result = registerPointClouds_with_cuda(
    latest_cloud_,
    accumulated_cloud_,
    initial_guess,
    registration_config_);
```

### 방법 B

먼저 TF로 scan을 target frame으로 옮긴 뒤,
ICP에는 `Identity`를 초기값으로 넣는다.

```cpp
auto transformed_cloud = tf2_utils::transformPointCloud(latest_cloud_, initial_guess);
result = registerPointClouds_with_cuda(
    transformed_cloud,
    accumulated_cloud_,
    Eigen::Matrix4f::Identity(),
    registration_config_);
```

이 노드에서는 방법 A가 더 단순하고 읽기 쉽다.

---

## 8. source / target 방향은 왜 중요할까

누적 맵 정합에서는 보통 이렇게 잡는다.

- source = 현재 들어온 스캔
- target = 기존에 쌓여 있는 누적 맵

즉:

```cpp
result = registerPointClouds_with_cuda(
    latest_cloud_,
    accumulated_cloud_,
    initial_guess,
    registration_config_);
```

왜 이렇게 해야 하는가:

- ICP 결과인 `aligned_cloud`는 source가 target에 맞춰진 결과다.
- 우리는 "현재 스캔을 기존 맵에 맞추고 싶다".
- 누적 맵 자체를 매번 새 스캔 쪽으로 움직이면 기준이 흔들린다.

쉽게 말해:

- 현재 스캔이 움직여야 함
- 누적 맵은 기준으로 남아 있어야 함

---

## 9. 첫 번째 스캔은 따로 처리해야 한다

첫 스캔에는 아직 target cloud가 없다.
즉 누적 맵이 비어 있으므로 registration을 돌릴 기준이 없는 상태다.

그래서 첫 프레임은 보통 이렇게 한다.

```cpp
if (accumulated_cloud_->empty()) {
    auto first_cloud_in_target =
        tf2_utils::transformPointCloud(latest_cloud_, initial_guess);

    *accumulated_cloud_ = *first_cloud_in_target;
    accumulated_cloud_ =
        preprocess::downsample(accumulated_cloud_, preprocess_config_);

    publish_aligned_cloud(first_cloud_in_target);
    publish_accumulated_cloud();

    has_latest_cloud_ = false;
    return;
}
```

왜 이렇게 해야 하는가:

- 첫 스캔은 비교 대상이 없으므로 ICP보다 TF만으로 맵 seed를 만드는 게 맞다.
- 이렇게 만든 첫 맵이 이후 모든 프레임의 registration target이 된다.

---

## 10. 정합 후에는 반드시 누적 맵에 합쳐야 한다

현재 코드에서 빠진 가장 중요한 단계가 이 부분이다.

정합이 끝난 뒤 해야 할 일:

```cpp
*accumulated_cloud_ += *result.aligned_cloud;
```

이걸 왜 해야 하는가:

- `result.aligned_cloud`는 "이번 프레임이 target 기준으로 맞춰진 결과"다.
- 누적 맵은 이걸 계속 붙여가면서 커져야 한다.
- 이 단계가 없으면 매 프레임마다 정합만 하고 맵은 커지지 않는다.

즉:

- 정합 성공
- 현재 프레임을 전역 맵에 merge

이 두 단계가 연결되어야 accumulation이 된다.

---

## 11. downsample은 꼭 다시 대입해야 한다

현재 `downsample()`은 새 cloud를 반환하는 함수 형태다.
즉 아래처럼 호출만 하면 아무 변화가 없다.

```cpp
preprocess::downsample(accumulated_cloud_, preprocess_config_);
```

반드시 이렇게 다시 저장해야 한다.

```cpp
accumulated_cloud_ =
    preprocess::downsample(accumulated_cloud_, preprocess_config_);
```

왜 중요한가:

- 누적 맵은 시간이 지날수록 포인트 수가 계속 증가한다.
- downsample을 제대로 적용하지 않으면 메모리와 속도가 금방 무너진다.
- 특히 registration target이 커질수록 ICP 비용도 커진다.

---

## 12. 누적 맵의 기준 프레임은 보통 고정 프레임이어야 한다

`target_frame_`로 `map` 또는 `base`를 받도록 생각하고 있을 수 있는데,
누적 맵 목적이라면 `base`는 대부분 좋지 않다.

왜냐하면:

- `base`는 로봇과 함께 움직이는 프레임이다.
- 과거 스캔과 현재 스캔이 같은 월드 기준에 쌓이지 않는다.
- 누적 맵이 흔들리는 좌표계에 저장되는 셈이 된다.

누적 맵에는 보통 아래 중 하나를 쓴다.

- `map`
- `odom`

정리하면:

- "현재 프레임을 base 기준으로 보고 싶다"는 가능
- "여러 프레임을 base 기준으로 누적하고 싶다"는 대체로 부적절

---

## 13. 퍼블리시는 역할을 나눠서 보는 게 좋다

현재 정렬된 프레임과 누적 맵은 디버깅 의미가 다르다.

따라서 퍼블리셔를 분리하는 게 좋다.

### 현재 정렬 결과 퍼블리시

```cpp
void publish_aligned_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = target_frame_;
    msg.header.stamp = latest_stamp_;
    aligned_cloud_pub_->publish(msg);
}
```

### 누적 맵 퍼블리시

```cpp
void publish_accumulated_cloud()
{
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*accumulated_cloud_, msg);
    msg.header.frame_id = target_frame_;
    msg.header.stamp = latest_stamp_;
    accumulation_cloud_pub_->publish(msg);
}
```

왜 분리하는가:

- `aligned_cloud_pub_`를 보면 이번 프레임 정합이 잘 됐는지 알 수 있다.
- `accumulation_cloud_pub_`를 보면 전체 맵이 잘 자라는지 알 수 있다.
- 둘을 섞으면 문제 원인을 찾기 어렵다.

---

## 14. `removeRobotPoints()`는 이름과 실제 의미를 다시 확인해야 한다

현재 전처리 코드의 `removeRobotPoints()`는 이름상 "로봇 점 제거"처럼 보이지만,
실제 필터 흐름은 특정 x/y 범위 내부 점들을 남기는 방향에 가깝다.

누적 맵을 만들 때 이 부분이 중요한 이유:

- 센서가 자기 로봇 몸체를 찍으면 그 점들이 맵에 누적될 수 있다.
- 그러면 registration 품질이 나빠질 수 있다.
- 따라서 로봇 본체 주변 영역은 보통 제외하는 편이 좋다.

즉 이 함수는 나중에 실제로 붙일 때
"박스 내부를 제거하는 로직"인지 다시 확인해야 한다.

---

## 15. 추천하는 전체 의사코드

아래 흐름으로 짜면 구조가 가장 자연스럽다.

```cpp
void NodeICPCuda::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    latest_cloud_->clear();
    pcl::fromROSMsg(*msg, *latest_cloud_);

    latest_stamp_ = msg->header.stamp;
    has_latest_cloud_ = !latest_cloud_->empty();
}

void NodeICPCuda::process_callback()
{
    if (!has_latest_cloud_ || latest_cloud_->empty()) {
        return;
    }

    auto tf_msg = tf2_utils::lookupTransform(
        tf_buffer_,
        target_frame_,
        frame_config_.sensor_frame,
        latest_stamp_,
        rclcpp::Duration::from_seconds(0.2));

    Eigen::Matrix4f initial_guess = tf2_utils::transformToMatrix(tf_msg);

    if (accumulated_cloud_->empty()) {
        auto first_cloud =
            tf2_utils::transformPointCloud(latest_cloud_, initial_guess);

        *accumulated_cloud_ = *first_cloud;
        accumulated_cloud_ =
            preprocess::downsample(accumulated_cloud_, preprocess_config_);

        publish_aligned_cloud(first_cloud);
        publish_accumulated_cloud();

        has_latest_cloud_ = false;
        return;
    }

    auto result = registration::registerPointClouds_with_cuda(
        latest_cloud_,
        accumulated_cloud_,
        initial_guess,
        registration_config_);

    if (!result.valid) {
        return;
    }

    publish_aligned_cloud(result.aligned_cloud);

    *accumulated_cloud_ += *result.aligned_cloud;
    accumulated_cloud_ =
        preprocess::downsample(accumulated_cloud_, preprocess_config_);

    publish_accumulated_cloud();

    has_latest_cloud_ = false;
}
```

---

## 16. 이 구조의 핵심 요약

가장 중요한 포인트만 다시 정리하면:

1. `cloud_callback()`은 최신 스캔과 메타데이터를 저장만 한다.
2. `process_callback()`은 TF, registration, merge, publish를 담당한다.
3. 현재 스캔은 기존 누적 맵에 맞춰야 한다.
4. 첫 프레임은 registration 없이 seed map으로 써야 한다.
5. 정합된 결과는 반드시 누적 맵에 merge해야 한다.
6. downsample은 반환값을 다시 저장해야 한다.
7. 누적 맵 기준 프레임은 보통 `map` 또는 `odom`이 적절하다.

---

## 17. 한 줄 결론

현재 코드에서 가장 본질적인 수정은 이것이다.

```text
"최신 스캔 1개를 저장"하는 구조를
"최신 스캔 + 시각 + frame_id를 저장하고,
그 스캔을 기존 누적 맵에 정합해서 계속 합치는 구조"로 바꾸기
```

이 방향으로 고치면 `node_icp_cuda`가 진짜 누적 맵 노드처럼 동작하게 된다.
