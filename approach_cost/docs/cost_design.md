# Cost Design

## 목적
이 패키지는 상호작용 접근 후보 셀들에 대해 공통 비용(`cost_common`)을 계산하고,
이를 기반으로 향후 모드별 최종 비용(`compute_final_cost`)으로 확장하기 위한 라이브러리다.

## 현재 지원 비용
1. 타겟과 각 셀 중심 사이의 거리
2. 각 셀 주변의 상태 전환 횟수 기반 안정성 비용
3. 로봇 위치와 각 셀 중심 사이의 거리 기반 접근 편의성 비용

## cost_common
현재 `cost_common`은 위 세 비용을 각각 min-max 정규화한 뒤 가중 평균으로 결합한 값이다.
정규화는 candidate 셀만 대상으로 수행한다.

## compute_final_cost
현재 Mode1에서는 `cost_common`을 그대로 최종 비용으로 사용한다.
추후 다른 모드가 추가되면 `compute_final_cost()`에서 mode별 추가 항을 더하는 구조로 확장한다.
