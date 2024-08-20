# **이중주차 가능한 무인형 주차장 시스템** 

---

## 📜 프로젝트 개요

- 해를 거듭할수록 증가하는 **차량 등록대수**
- 확장비용이 큰 기존 건물의 **주차장**
- **불법 주차 & 사고**로 인한 법적 책임 



<p align="center">
  <img src="https://github.com/user-attachments/assets/455dd859-c9d4-462d-aeae-b32612b6d9ca" alt="Overview Image"/>
</p>

---

## 👥 팀원 소개

| ![김요한](https://avatars.githubusercontent.com/u/163791820?v=4) | ![신재훈](https://avatars.githubusercontent.com/u/86091697?v=4) | ![이민영](https://avatars.githubusercontent.com/u/163792019?v=4) | ![조성오](https://github.com/user-attachments/assets/b2f930fc-befa-4edd-b423-3b90a591ac57) |
|:-------:|:-------:|:-------:|:-------:|
| **김요한**<br> [GitHub: `@1yohanyo1`](https://github.com/1yohanyo1) <br> 로봇 동작 및 제어 |**신재훈**<br> [GitHub: `@Shoons23`](https://github.com/Shoons23) <br> AI & 임베디드 시스템 | **이민영**<br> [GitHub: `@whiteblue7`](https://github.com/whiteblue7) <br> GUI & DB | **조성오**<br>  <br> 설계 및 협업 관리 |




## 🚀 기술 스택

<table>
  <tr>
    <td><strong>🔧 개발 환경</strong></td>
    <td>
      <img src="https://img.shields.io/badge/Docker-333333?style=flat&logo=docker&logoColor=2496ED"/> 
      <img src="https://img.shields.io/badge/Linux-333333?style=flat&logo=Linux&logoColor=FCC624"/>
      <img src="https://img.shields.io/badge/ROS2-333333?style=flat&logo=ROS&logoColor=22314E"/>
    </td>
  </tr>

  <tr>
    <td><strong>🤝 협업 툴</strong></td>
    <td>
      <img src="https://img.shields.io/badge/Confluence-333333?style=flat&logo=confluence&logoColor=172B4D"/>
      <img src="https://img.shields.io/badge/Github-333333?style=flat&logo=github&logoColor=181717"/>
      <img src="https://img.shields.io/badge/jira-333333?style=flat&logo=jira&logoColor=0052CC"/>
    </td>
  </tr>

  <tr>
    <td><strong>👨‍💻​ 사용 언어</strong></td>
    <td>
      <img src="https://img.shields.io/badge/-C++-333333?logo=cplusplus&logoColor=00599C"/>
      <img src="https://img.shields.io/badge/Python-333333?style=flat&logo=Python&logoColor=3776AB"/>
      <img src="https://img.shields.io/badge/MySQL-333333?style=flat&logo=mysql&logoColor=4479A1"/>
    </td>
  </tr>

 <tr>
    <td><strong>💻 사용 기술</strong></td>
    <td>
      <img src="https://img.shields.io/badge/CMake-333333?style=flat&logo=cmake&logoColor=064F8C"/>
      <img src="https://img.shields.io/badge/OpenCV-333333?logo=opencv&logoColor=5C3EE8"/>
      <img src="https://img.shields.io/badge/PyQt-333333?style=flat&logo=qt&logoColor=41CD52"/>
    </td>
  </tr>

  <tr>
    <td><strong>🔌 임베디드 시스템</strong></td>
    <td>
      <img src="https://img.shields.io/badge/Arduino-333333?style=flat&logo=Arduino&logoColor=00878F"/>
      <img src="https://img.shields.io/badge/RaspberryPi-333333?style=flat&logo=RaspberryPi&logoColor=A22846"/>
      <img src="https://img.shields.io/badge/Platform-ESP32-333333?style=flat&logo=Arduino&logoColor=333333&color=333333"/>
    </td>
  </tr>
</table>

---

## 🗺 주차장 맵 설계

<p align="center">
  <img src="https://github.com/user-attachments/assets/78a76d6b-039e-42f0-a815-7441e0971cc1" alt="Parking Map Design"/>
</p>

---

## 🛠 하드웨어 구성도

<p align="center">
  <img src="https://github.com/user-attachments/assets/59d6eb3c-927a-4ef0-b4dc-9e54871fdea2" alt="Hardware Configuration"/>
</p>

### 주차 로봇 구성도

<p align="center">
  <img src="https://github.com/user-attachments/assets/ff77e969-05ac-4c80-b0b8-200ef5b49f97" alt="Robot Configuration 1"/>
  <img src="https://github.com/user-attachments/assets/f0df6231-766a-43c6-b8f7-46ce38aa5820" alt="Robot Configuration 2"/>
</p>

---

## 🧩 시스템 구성도

<p align="center">
  <img src="https://github.com/user-attachments/assets/78510331-320d-43da-90dd-e7c7ce9a9fde" alt="System Configuration"/>
</p>

---

## 🔄 로봇 상태전이도

<p align="center">
  <img src="https://github.com/user-attachments/assets/f6b51221-64ee-4b88-b4b9-e460732cec4e" alt="Robot State Transition Diagram"/>
</p>

---

## 📊 작업 할당 우선순위 공식

우선순위 점수(S)가 높을수록 작업 할당 받을 확률이 낮음:

$$
S = w_1 B + w_2 T + w_3 D
$$

### 전제 조건

$$
w_1 > 0 , w_2 < 0, w_3 < 0 
$$

$$
|w_2| > |w_1| > |w_3|
$$


### 1. 배터리 상태(B)

- 배터리 잔량이 20% 미만인 경우 \( B < 20 \) 우선 충전(B=-100)으로 설정
- **효과:** 배터리가 적을수록 우선순위 점수가 높아져 작업 할당 확률 감소

### 2. 작업 예상 시간(T)

- 작업이 없는 경우 \( T = 0 \)으로 설정
- **효과:** 작업이 많을수록 우선순위 점수가 낮아져서 작업 할당 확률 감소

### 3. 로봇 위치와 목표 위치의 거리(D)

- 현재 로봇 위치와 목표 위치 간의 거리 계산
- **효과:** 거리가 가까울수록 \( D \) 값이 커져 S가 커져 작업 할당 확률 감소

---

## 🧠 핵심 기술

### 번호판 추출

<p align="center">
  <img src="https://github.com/user-attachments/assets/8b59acbf-5429-40f8-a7b2-ad23ca40d767" alt="License Plate Extraction"/>
</p>

### 주차 체크

<p align="center">
  <img src="https://github.com/user-attachments/assets/1145afe2-43ce-4e78-a9ac-76803a114b7e" alt="Double Check"/>
</p>

### 주행

<p align="center">
  <img src="https://github.com/user-attachments/assets/23e9ee98-889f-4d31-b99c-50fcfb4d5869" alt="Driving"/>
</p>

---

## 🛠 시나리오

### 일반 주차 시나리오

<p align="center">
  <img src="https://github.com/user-attachments/assets/8ad0e26d-7881-4fee-affc-8aa6280991a4" alt="Normal Parking Scenario"/>
</p>

### 이중 주차 시나리오

<p align="center">
  <img src="https://github.com/user-attachments/assets/06e3401e-9807-4f8d-9fdd-4c00100b1f8d" alt="Double Parking Scenario"/>
</p>

### 일반 출차 시나리오

<p align="center">
  <img src="https://github.com/user-attachments/assets/3120cc54-9425-48d0-82da-c553b46512e7" alt="Normal Exit Scenario"/>
</p>

### 이중 출차 시나리오

<p align="center">
  <img src="https://github.com/user-attachments/assets/2bc0b1e1-542f-4d3d-9474-185c45a36bc6" alt="Double Exit Scenario"/>
</p>

---

## 🤖 다중 로봇 제어

<p align="center">
  <img src="https://github.com/user-attachments/assets/20e13b2c-24d5-464e-864e-c7221b090a82" alt="Multi-Robot Control"/>
</p>

---

## 🎥 시연 영상

<p align="center">
  <a href="https://youtu.be/TfLEBPAAAMk">
    <img src="https://img.youtube.com/vi/TfLEBPAAAMk/0.jpg" alt="Demo Video">
  </a>
</p>

---

