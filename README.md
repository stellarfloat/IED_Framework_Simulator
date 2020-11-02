# IED_Framework_Simulator
2020-2 창업연계공학설계입문의 프로젝트의 프레임워크를 컴퓨터로 시뮬레이션
하는 소프트웨어입니다.


## 특징
- [ ] Hot-Reload 지원 (추후 지원예정)
- [ ] GUI 파라미터 슬라이더 제공 (추후 지원예정)
- [x] 적은 리소스 사용 (CPU: ~5%, MEM: ~30MB)
- [ ] 정확한 물리량 (현재 실제 물리량과 일치하도록 내부 변수값 조절 중입니다)
    - [x] 길이 (1px == 1mm)
    - [ ] 질량 (프레임워크 무게 측정 필요. 단, 내부 단위 자체는 구현되어 있습니다.)
    - [x] 시간 (실제 세계와 동일)
    - [x] 서보 토크 (1.875kg/cm)
- [x] 탁구공 유효 접촉 지름 고려 (사용법 참조)
- [x] 서보 Stall 상태 감지
- [x] 파라미터 유효성 자동 검증 (사용법 참조)
- [x] 오픈소스


## 설치하기
* Prerequisite:
`Python 3.7+`

로컬 디렉토리에 이 저장소를 클론합니다.

    git clone https://github.com/stellarfloat/IED_Framework_Simulator.git

클론한 디렉토리 안에서 다음의 명령을 실행하여 패키지들을 설치합니다.

    pip install -r requirements.txt

이제 시뮬레이터 실행 환경 구성이 완료되었습니다.


## 사용법
**_아래의 설명은 버전 0.1.0의 CLI 버전을 기준으로 작성되었습니다. GUI 버전이 업데이트 된다면 아래의 내용 또한 그에 맞게 수정될 예정입니다._**
### 파라미터 설정하기


CLI 버전(main_CLI.py)에서는 실행 인자값으로 파라미터들을 전달합니다. 각 파라미터의 단위는 다음과 같습니다. 
- _A~F_: `mm`
- _*_angle_: `°(도)`
- _speed_: `°/s (도/1초)`
- _rail_margin_: `mm`

서보암이 하늘을 가리키고 있을 경우 0°, 바닥면을 가리키고 있을 경우 180°입니다. (90°는 프레임워크 바깥쪽입니다.) <br>
레일 사이의 간격은 각 레일 끝부터 끝까지를 기준으로 합니다. 따라서 각 레일 반경의 중심을 이은 선의 길이에서 5mm를 뺀 값과 같습니다.

    python main_CLI.py [A] [B] [C] [D] [E] [F] [lower_angle] [upper_angle] [speed] [rail_margin]
서보는 speed의 속도로 lower_angle과 upper_angle 사이를 왕복하게 됩니다. (lower_angle < upper_angle 이어야 합니다.)

예시:

    python main_CLI.py 50 100 75 50 50 0 0 180 550 30

* 위의 값은 임의의 값입니다.

_현재 버전(0.1.0)에서는 F 파라미터가 아무런 동작을 하지 않습니다. 추후 적외선 센서 관련 구현이 된 다음 적용될 예정입니다._

파라미터는 숫자 형식(int, float)이어야 하며, 9개의 파라미터를 모두 입력해야 정상 동작합니다.

### 파라미터 유효성 자동 검증
시뮬레이터가 실행될 때 입력된 값들이 유효한 값인지 자동으로 검증합니다. 유효하지 않은 값이라고 판단되면 시뮬레이터가 시작되지 않습니다. 예를 들어, 서보 암이 너무 짧아서 레일플레이트와 서보를 연결할 수 없는 경우 ValueError가 일어납니다. 너무 길어도 마찬가지로 오류가 발생합니다. 또한 값들이 음수이거나, 숫자가 아니거나, 각도가 0~180도 바깥이거나, 레일 간격이 공 지름보다 크거나 등 현실적으로 불가능한 값들이 입력되면 오류가 발생하도록 되어 있습니다.

### 서보모터 Stall 상태 감지
이 시뮬레이터에는 1.875kg/cm의 토크를 기준으로 구현된 서보 모터가 있습니다 (MG90S, 동작전압 5V 기준). 실제 프레임워크에서 서보에 무리한 힘이 가해져서는 곤란하므로, 이를 시뮬레이터 단에서 미리 고려해볼 수 있게 했습니다. 서보가 Stall 상태, 즉 최대 토크로 동작할 때 `Servo Stall | t = {}` 로그가 콘솔에 출력됩니다. t 값은 시뮬레이션이 시작된 후 경과한 시간이며, 서보가 Stall 상태로 변할 때마다 출력됩니다.

### 탁구공의 접촉 지름
탁구공이 접촉하는 부분은 레일플레이트가 아닙니다. 실제로는 레일에 의해 지지되는데, 따라서 레일 사이의 간격에 따라 (같은 기울기에서) 공의 회전 각속도가 변할 수 있습니다. 이를 고려하기 위해, 시뮬레이터는 입력된 간격을 기반으로 탁구공의 접촉 지름을 자동으로 계산합니다. 계산된 지름은 탁구공의 실제 지름인 40mm보다 작습니다. <br> 시뮬레이터 실행 시 반드시 본인 값에 맞게 간격 값을 설정하시기 바랍니다. 


## 주의사항

* _(수정 예정, 현재 버전 0.1.0)_ 현재 시뮬레이터에선 프레임워크 각 요소의 정확한 무게가 반영되어 있지 않습니다. 어림값으로 설정되어 있으며, 실제 무게 측정 후 반영 예정입니다.

* 시뮬레이터의 특성상, 현실 세계와 오차가 존재할 수밖에 없습니다. 이 점을 고려해서 A~F의 값들을 결정하시기 바랍니다. <br>현재 버전에선 간결성을 위해 다음과 같은 요소들이 무시되도록 설정되어 있습니다: <br> - 연결부 사이의 마찰 <br> - 탁구공과 레일플레이트 사이의 마찰 <br> - 공기저항 <br> - 프레임워크 구성요소 간의 유격이나 비틀림

* 이 시뮬레이터는 2D 시뮬레이터입니다. z축 방향(화면의 법선벡터)의 충격이나 흔들림은 고려되어 있지 않습니다. 따라서 실제 프레임워크의 레일 사이에서 탁구공이 벗어나지 않는 것을 가정합니다.





## 버그 리포트, 기여, 문의사항 등
버그 리포트나 PR 등은 언제나 환영합니다! 또한 새로운 기능에 대한 아이디어가 있으시다면 제안해주시면 감사하겠습니다. 이러한 기여는 이 레포에서 Issue를 열어서 하시면 됩니다. 문의사항은 2020-2 공학설계입문 슬랙의 관련 채널에서 메시지를 보내시면 됩니다.



## 업데이트 로그
[2020. 11. 2.] **0.1.0** : initial release(alpha, CLI)


## 크레딧
만든 사람: 소프트웨어학부_20203155_추헌준

사용된 라이브러리: PyQt(GUI), Pygame(2D Draw), Pymunk(Physics)