# K-SW-Project-9
K-SW project 9 Git


Development diary. I wrote it on the 'paul123k/K-SW-Project-9 Wiki'
***
Hello!
We are participating in 'IITP Summer Program 2018' at Purdue University.

# [Automate the aiming of a mortar]
> Name : BGP (박격포)   
> Member : Kim Seok-won, Lee Yu-jin, Park Jin-hui `In alphabetical order`

GPS 좌표를 이용하여 원하는 장소에 자동으로 박격포 에임을 맞추고 발사하는 프로젝트

|포탄(3D프린트)|타겟위치 선택(어플)|박격포&Pan-tilt&라즈베리파이|
|:--:|:--:|:--:|
|![image](https://user-images.githubusercontent.com/37680108/116300166-8b323f80-a7d9-11eb-8b42-61874c4cf203.png)|![image](https://user-images.githubusercontent.com/37680108/116299967-47d7d100-a7d9-11eb-8fc8-feeb892e24e2.png)| ![image](https://user-images.githubusercontent.com/37680108/116299976-4ad2c180-a7d9-11eb-817d-b9a659607fc3.png)|

***
### 1st Week (6/27 ~ 7/03)

* 오리엔테이션, 팀 구성, 라즈베리파이와 부속품 구비
* 박격포 모형 구경
* pan-tilt 작동방식 파악
* 라즈베리파이를 이용하여 센서를 작동시키는 원리 공부
* 선행 연구 검토

***
### 2nd Week (7/04 ~ 7/10)

* Break Week - New York Travel

***
### 3rd Week (7/11 ~ 7/17)

* pan-tilt의 현재 위치와 각도를 받아오기 위해 BerryGPS-IMU 모듈 신청 (GPS, Gyroscope)
* 기존에 있던 GPS 모듈을 이용, 라즈베리파이에서 GPS 값을 받아오는 코드 작성
* 타켓위치를 잡기 위한 어플 제작 시작
* 라즈베리파이로 pan-tilt 동작 및 3차원 각도 변환법 연구 시작
* 탄도 공식 정리 (항력 고려X)

***
### 4th Week (7/18 ~ 7/24)

* 발사위치와 타겟위치의 GPS값을 이용하여 pan-tilt의 각도를 구하는 공식 완성
* 어플 (데모버전) 완성
* BerryGPS-IMU 모듈 도착
* BerryGPS-IMU 이용, tilt 모터의 현재각도 받아오기 성공
* 라즈베리파이를 이용, pan-tilt 조작 성공
* 블루투스를 이용한 안드로이드와 라즈베리파이 통신 성공

***
### 5th Week (7/25 ~ 7/31)

* 라즈베리파이에 모듈 및 센서 탑재
* pan-tilt모터와 라즈베리파이 합체
* BerryGPS-IMU 이용, pan-tilt의 GPS값 parsing 성공
* 어플 완성

***
### 6th Week (8/01 ~ 8/07)

* 이전까지 만든 코드 결합
* 박격포 탄의 초기속도 측정
* 어플을 통해 GPS값을 전송받아 해당하는 각도만큼 tilt 회전 성공

***
### 7th Week (8/08 ~ 8/14)

* 어플을 통해 GPS값을 전송받아 해당하는 각도만큼 pan 회전 성공
* 큰 pan-tilt로 교체 (박격포와 라즈베리파이 모두 장착하기 위해)
* 박격포와 pan-tilt(with 라즈베리파이) 합체
* 어플로 타켓위치를 선택하면 pan-tilt를 통해 박격포의 에임을 자동으로 맞추기 성공
* 농장에 나가서 (미니)박격포 발사
* 발표자료 제작
* 13일 최종발표
