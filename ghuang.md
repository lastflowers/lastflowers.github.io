---
title: Guoquan Huang - Univ. of Delaware
use_math: true
---

## [IROS 2018] LIPS: LiDAR-Inertial 3D Plane SLAM

* 그래프 최적화 기반에서 IMU-preintegration + cloest point (CP) 평면 표현법으로 factor를 설계한 시스템

  * Iterative closest point (ICP)의 경우 평면과 같이 기하학적인 정보를 무시함
  * 기존의 평면 표현방법인 Hesse form $ax+by+cz+d=0$ 은 over-parametrization 이라서 최적화 문제를 푸는데 있어서 정보행렬 (문맥상 헤시안 역행렬을 의미)의 singularity를 야기함.
  * CP 표현방법은 기준 프레임에서 가장 가까운 점의 위치로 $\mathbb{R}^3$ 평면을 표현하는 방법임
  * CP의 장점은 최소차수로 평면을 표현, additive 오차 모델에 용이
  * 즉, 아래의 비선형 최적화 문제를 푸는것 :

  $$
  \hat{\mathbf{x}} = \mathrm{argmin} \sum_i \left|| \mathbf{r}_{I_i}(\mathbf{x}) \right||^2_{\mathbf{R}_{I_i}} + \sum_j \left|| \mathbf{r}_{\Pi_j}(\mathbf{x}) \right||^2_{\mathbf{R}_{\Pi_i}}
  $$

  

* **3D plane factor** : 

  * CP representation  : $\Pi^G = \mathbf{n}^G d^G$ 
  * 가장 가까운 점이 관측 프레임의 원점에 존재한다는 singularity가 존재함
  * 이를 피하기 위해 **anchor** 프레임을 정의하는데, 이는 평면을 관측한 첫번째 프레임을 의미함. 따라서 로컬 프레임에서  LiDAR로 얻은 point cloud로 평면을 구한다고 생각하면 물리적으로 singularity가 발생할 수 없음
  * Plane factor는 다음과 같이 정의되며 여기서 $\hat{\Pi}$ 는 LiDAR로 부터 측정한 값임
  * 흥미로운점은 CP의 시점 변환은 단순히 SE(3)로 바뀌는게 아니라 아래의 식처럼 변환됨

  $$
  \mathbf{r}_\Pi(\mathbf{x}) = \Pi^L(\mathbf{x}) - \hat{\Pi}^L \\
  \Pi^L(\mathbf{x}) = \left( \mathbf{R}^L_A \mathbf{n}^A \right) \left( d^A - \mathbf{p}^{A,T}_L \mathbf{n}^A \right)
  $$

  * 평면 피팅을 통해 LiDAR의 점구름을 CP 측정치로 변환하는 과정이 있음

* 실험에서 짝관계는 측정된 CP와 기존의 CP의 마할라노비스 거리를 통해 구했다고함
  * 미리 정해진 갯수의 평면이 관측된다고 가정한듯.
  * 아래의 그림과 같이 **좁은 영역에서 평면 피팅이 잘되는 환경**을 세팅하여 실험을 수행함



## [IROS 2018] Robocentric Visual-Inertial Odometry

* Summary : 

  * MSCKF의 inconsistency를 해결하고자 robocentric MSCKF를 유도함. 여기서 말하는 robocentric formulation이란 항체의 상태변수를 body frame 기준으로 나타내는 표현법을 의미한다.
  * 저자가 주장하는 robocentric VIO의 장점은 1) 필터 초기화가 간단해지고 2) 필터 일관성을 준수한다. 하지만 필터 초기화에서 여전히 중력 방향을 초기화 해줘야 하는데 기존 방법이랑 같은 것 아닌가 ?
  * Supplementary report를 포함하여 제안 방법의 완성도는 있는 듯 하나 Lie group으로 표현하면 해결 될 것을 굳이 복잡한 robocentric 형태로 써야하는지 의문이 든다.

* Details :

  * 상태변수: robocentric reference frame $\{R_k\}$에서 본 $\{G\}$의 포즈/중력방향, $\{I_{\tau}\}$ 의 IMU 상태변수, SLW 끼리 연쇄적으로 reference frame을 잡는 SLW

  $$
  \mathbf{x} = \begin{bmatrix} ^{R_k}\mathbf{x}_G & ^{R_k}\mathbf{x}_{I_\tau} & \mathbf{w}_k \end{bmatrix}
  $$

  * Propagation : INS in inertial frame

  * Update: anchor frame 포즈 + $\phi,\psi,\rho$ 로 표현하는 inverse-depth parametrization. (TRO 2008에 제안된 이후 거의 표준이된 표현법)

    ​				$\mathbf{w}$에 대해 측정치 Jacobian 행렬이 dense 해져서 "more efficient" 하다고 하는데 뒷받침이 부족한 주장인듯 하다.

  * State augmentation: 기존 MSCKF와 동일하게 SLW 확장

  * Composition: $\{R\}$이 계속 갱신되므로 이를 업데이트 해주는 과정. 추정값과 공분산 (커짐)이 변화된다. 

  * Observability analysis: $\{G\}$의 위치,자세 변화 그리고 $\{L\}$과 $\{I\}$의 위치변화는 가관측하지 않은 방향임 9-DOF. Global로 표현했다면 5-DOF가 나왔을 텐데 robocentric으로 바꾸면서 9-DOF로 커졌다. 바람직한 것인가?

* Experiments:

  * Simulation: NEES는 여전히 1보다 큰 듯한데 consistent하다고 말할 수 있나?
  * EuRoC & driving test



## [RAL 2019] Visual-Inertial Localization WIth Prior LiDAR Map Constraints

* 이미 만들어진 LiDAR 맵을 이용하여 MSCKF기반의 스테레오 카메라 VIO의 위치 오차를 bound 한다. 특히, 기하학적 정보를 가진 LiDAR 맵과 밝기 정보를 가진 카메라를 융합하기 위해서 semi-dense 맵을 작성하고 이를 맵과 매칭함	
* EKF 상태변수 : MSCKF 상태변수 + 맵 프레임, $\{M\}$ 의 포즈를 추가함

$$
\mathbf{x} = \begin{bmatrix} \mathbf{x}_{INS} & \mathbf{q}_{GM} & \mathbf{p}^G_M & \mathbf{x}_{SLW} \end{bmatrix}^T
$$

* sparse feature track과 semi-dense 점구름을 LiDAR 지도에 매칭하여 나오는 (공분산포함) $q_{CM}, p^M_C$ 가 있다고 하고 측정치 자코비안을 유도함
* **Semi-dense reconstruction** : 
  * 계산량을 고려하여 keyframe에 대해서만 reconstruction 수행
  * 고전적인 스테레오 매칭 (에피폴라라인에 따라 SAD 비교)을 통해 초기 깊이를 구하고 그 후 keyframe이 더 들어오면 깊이를 refine 하는 형태. 
  * 깊이를 refine하기 위해 keyframe 의 reconstruction 간의 짝관계를 구해야 하며 이를 위해 모든 후보에 대해 시점변환을 한뒤 일련의 compatibility 테스트를 수행하여 구한다.
  * LiDAR 지도에 등록하기전 reference 키프레임 (여기서는 가장 최근의 키프레임으로 정의)으로 각각 키프레임의 semi-dense 점들을 변환한다. 여기서 연산이 많이든다고 기술되어 있으며 이를 2번째 스레드에서 처리함.
* **NDT (normal distribution transform) pointcloud registration**
  * NDT에서는 점구름을 가우시안 분포라고 가정하여 두 점구름 집합간의 pdf의 차이를 최소화하는 기법임.
  * ICP 같이 각 점을 매칭하는 방법에 비해 샘플들의 uneveness에도 강건하다는 장점이 있음.
  * 즉, LiDAR 맵 (템플릿)과 비전의 맵 (소스)의 가우시안 분포를 피팅하는 상대포즈를 구하는것이 목적 



## [ICRA 2019] A Linear-Complexity EKF for Visual-Inertial Navigation with Loop Clousres

* MSCKF에 loop-closure 을 적용하여 넓은 영역에서 위치 드리프트를 줄이고자 하는 것이 motivation 임.

* 단순하게 키프레임을 상태변수에 저장하여 필터 업데이트하는 형태는 논문에 따르면 상태변수의 차원 $n$에 대하여 $O(n^2)$ 의 복잡도를 가지고 있어서 넓은 영역에서 수백개의 키프레임을 가질 때 실시간 구동이 어려움.

* 또 한가지 단순한 방법으로는 상태변수의 키프레임은 오랜 시간 tracking 되었으므로 불확실성이 낮아졌기 때문에 지역 최적화에서는 known으로 가정해서 풀지만, 이는 추정기의 inconsistent를 유발할 수 있다. 실제 예로 VINS-MONO가 이런식으로 했다고함.

* 따라서 본 논문에서는 상태변수의 불확실성은 고려하되 필터 업데이트는 수행하지 않는 고전적인 Schmidt KF를 이용하여 계산 복잡도를 $O(n)$ 으로 낮춤

* **방법론**

  * **1. 상태변수** : 슬라이딩 윈도우에서 고정 시간 너비로 키프레임 상태변수를 복사하여 저장함

  $$
  \mathbf{x} = \begin{bmatrix} \mathbf{x}_{INS} & \mathbf{x}_{SLW} & \mathbf{x}_{Key}  \end{bmatrix}
  $$

  * **2. Propagation** : 기존 MSCKF와 동일하나 키프레임 상태변수의 큰 크기를 고려하여 IMU 주기로 공분산을 업데이트 하지 않고 state-transition 행렬을 이미지 싱크로 모아서 처리함
  * **3. Loop-closing** : DBoW2를 이용하여 프레임간의 룹을 결정하는데, 현재 이미지에 대한 가장 유사한 키프레임 하나만 추출해온다. (여러개를 추출할 수도 있지만 계산량을 고려하여 하나만 한다고함.) 그 후 active  특징점 트랙과 동일한 특징점을 키프레임에서 추출하여 트랙을 형성하여 필터를 업데이트함. 즉 특징점의 관측 baseline을 길게하는 implicit loop-closing을 하는 것
  * **4. Filter update** : Schmidt KF에 따라 키프레임의 공분산과 추정치는 업데이트 하지않고 $\mathbf{x}_{INS}, \mathbf{x}_{SLW}$ 의 추정치, 공분산 그리고 $\mathbf{x}_{Key}$ 간의 cross correlation 만 업데이트 해주며, 이 과정에서 복잡도가 선형으로 떨어진다고함

* 필터의 복잡도 분석

  * EKF : 공분산 측정치 업데이트 식 ($(I-KH)P$) 에서 $K_S S K_S^T$ 가 있는데 이 행렬곱의 복잡도는 $O(mn^2+m^2n)$  에서 $O(n^2)$, m은 측정치 갯수, n은 키프레임 상태변수 차원수
  * Schmidt-KF : $K_S=0$ 이라서 위의 복잡도는 사라지고, 다음으로 강한 복잡도는 공분산 측정치 업데이트에서 $L_A S^{-1} L^T_S$이고 이것의 복잡도는 $O(amn+m^2n)$ 이 되고, 최종적으로 키프레임 스테이트에 대한 복잡도는 $O(n)$ 이 된다. 여기서 a는 active 상태변수의 차원임.

* **Idea) 키프레임에서 특징점 트랙을 추가하는 방법은 키프레임 정보를 다 이용한다고 보기는 힘들거 같음. pose-graph 최적화의 출력값을 필터 업데이트 측정치로 활용하는건 어떨까**



## [IROS 2019] LIC-Fusion: LiDAR-Inertial-Camera Odometry

* LiDAR의 단점인 점구름 sparsity, 비싼 가격 그리고 카메라 보다 낮은 샘플링 시간을 Inertial-Camera 시스템으로 보완. 반면, Inertial-Camera는 직접적인 깊이 측정치가 없다는 단점이 있음

* MSCKF기반의 카메라의 sparse 특징점, 라이다 스캔의 모서리, 평면 특징점을 측정치로 이용.

* 상태변수

  * 카메라 슬라이딩 윈도우와 더불어 라이다 스캔의 슬라이딩 윈도우도 저장

* LiDAR 측정치 모델 (LOAM과 동일) : 

  * Edge : 삼각형의 넓이 공식으로 부터 아래의 측정치 모델이 유도 가능하며, i번째 LiDAR 점 측정치와 2개의 LiDAR 프레임의 상대포즈로 표현이 가능하다. 여기서 j와 k번째 점은 $p^{L_{l+1}}_{f_i}$ 을 현재 필터 추정치를 이용하여 $p^{L_{l}}_{f_i}$ 로 시점변환 하였을 때 $L_l$ 스캔 점 중 서로 다른 scanline에 존재하는 가장 가까운 2점이다.

  $$
  r\left( p^{L_{l+1}}_{f_i}, R^{L_{l+1}}_{L_l}, p^{L_{l+1}}_{L_l} \right) = \frac {\left\| (p^{L_l}_{f_i} - p^{L_l}_{f_j}) \times (p^{L_l}_{f_i} - p^{L_l}_{f_k}) \right\|}{|| (p^{L_l}_{f_j} - p^{L_l}_{f_k}) ||}
  $$

  

  * Surface : edge의 경우와 유사하게 가장 근접한 3점을 추출하여 평면을 만들어 평면에 대한 거리가 측정치가 된다.

* **기존 MSCKF에 시공간 보정 상태변수를 넣고 LOAM의 측정치 방정식을 MSCKF 형태로 구현한것. **

[back](./)