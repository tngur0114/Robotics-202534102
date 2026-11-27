# [1주차 - 11/15] ESP32 기반 임베디드 입출력 실습

## 학습 내용

- ESP32 핀 구조 및 레지스터
- GPIO를 이용한 LED 제어
- 버튼 입력과 풀업·풀다운 저항
- 바운싱과 디바운싱
- Serial Communication
- ADC와 PWM을 이용한 LED 밝기 조절
- Arduino IDE 개발 환경 구성
- PCB 키트의 센서 및 액추에이터

## 1. 임베디드 회로 기초

이전 강의에서 학습한 전압, 전류, 저항, GPIO, ADC 개념을 바탕으로 ESP32와 외부 부품을 연결하여 입출력을 제어한다.

- **VCC**: 회로에 전원을 공급하는 단자
- **GND**: 회로의 기준 전위
- **과전류(Overcurrent)**: 회로의 허용 범위를 초과하는 전류가 흐르는 현상
- **저항(Resistance)**: 전류의 흐름을 제한하는 부품

LED에 과도한 전류가 흐르면 부품이 손상될 수 있으므로 저항을 직렬로 연결한다. 실습에서는 약 220Ω의 저항을 사용했다.

## 2. ESP32 핀 구조

### 2-1. 전원 핀

| Pin | 기능 |
|---|---|
| GND | 회로의 기준 전위(0V) |
| 5V | 보드의 5V 전원 연결 |
| 3V3 | 3.3V 전원 공급 |
| VIN | 외부 전원 입력 |

ESP32의 GPIO는 일반적으로 3.3V 로직을 사용하므로 5V 신호를 GPIO에 직접 입력하지 않도록 주의해야 한다.

VIN의 허용 입력 전압과 전원 핀의 사용 방법은 보드 종류에 따라 달라질 수 있으므로 핀 배치도와 사양을 확인해야 한다.

### 2-2. GPIO와 Analog Pin

| 구분 | 기능 | 관련 함수 |
|---|---|---|
| Digital Input | HIGH/LOW 상태 입력 | `digitalRead()` |
| Digital Output | HIGH/LOW 상태 출력 | `digitalWrite()` |
| Analog Input | ADC를 이용한 전압 측정 | `analogRead()` |
| PWM Output | 펄스 폭을 이용한 출력 조절 | `analogWrite()` |

GPIO마다 지원하는 기능이 다르며, 아날로그 입력은 ADC를 지원하는 핀에서만 가능하다.

ESP32에서는 출력이 가능한 여러 GPIO에 PWM 신호를 할당할 수 있다.

## 3. 레지스터(Register)와 저수준 동작

### 3-1. 레지스터의 역할

MCU는 내부 레지스터를 이용해 GPIO의 입출력 방향, 출력 상태, 주변 장치의 동작 등을 설정한다.

- **데이터 방향 레지스터**: 핀을 입력 또는 출력으로 사용할지 설정
- **출력 레지스터**: 출력 핀의 HIGH/LOW 상태 설정
- **ADC 관련 레지스터**: ADC 동작 설정 및 변환 결과 관리

AVR 계열 MCU에서는 데이터 방향 레지스터를 DDR(Data Direction Register)이라고 한다. ESP32는 AVR과 레지스터 구조가 다르지만 레지스터를 통해 하드웨어를 제어한다는 기본 원리는 같다.

### 3-2. Arduino API와 레지스터

Arduino에서 사용하는 `pinMode()`와 `digitalWrite()` 등의 함수는 내부적인 하드웨어 설정 과정을 직접 작성하지 않아도 GPIO를 제어할 수 있도록 제공하는 API이다.

```text
Arduino API
     ↓
Hardware Driver / Register
     ↓
GPIO Pin
     ↓
LED, Button, Sensor
```

사용자가 함수를 호출하면 내부적으로 드라이버와 레지스터 설정을 통해 실제 하드웨어가 동작한다.

## 4. LED 제어 실습

### 4-1. setup()과 loop()

- `setup()`: 프로그램 시작 시 한 번 실행되며 핀 모드, 시리얼 통신 등을 초기화한다.
- `loop()`: `setup()` 실행 이후 반복적으로 호출된다.

사용하는 핀 번호는 코드 상단에 상수로 정의하면 관리하기 편하다.

### 4-2. Digital Output

| 함수 | 기능 |
|---|---|
| `pinMode(pin, OUTPUT)` | 출력 모드 설정 |
| `digitalWrite(pin, HIGH)` | HIGH 신호 출력 |
| `digitalWrite(pin, LOW)` | LOW 신호 출력 |

일반적인 Active-high LED 회로의 연결 구조는 다음과 같다.

```text
GPIO ─── 저항 ─── LED ─── GND
```

GPIO가 HIGH일 때 LED에 전류가 흐르고, LOW일 때 LED가 꺼진다.

LED의 연결 방향과 회로 구성에 따라 HIGH와 LOW에 대한 동작이 달라질 수 있다.

## 5. 버튼 입력 실습

### 5-1. Digital Input

버튼의 눌림 여부는 GPIO를 입력 모드로 설정한 뒤 `digitalRead()`를 사용해 확인한다.

| 함수 | 기능 |
|---|---|
| `pinMode(pin, INPUT)` | 입력 모드 설정 |
| `digitalRead(pin)` | HIGH 또는 LOW 상태 읽기 |

### 5-2. Floating과 Pull-down Resistor

**Floating**은 입력 핀이 HIGH나 LOW 중 어느 쪽에도 안정적으로 연결되지 않은 상태이다.

버튼을 누르지 않았을 때 입력 핀이 Floating 상태가 되면 주변 잡음에 의해 입력값이 불규칙하게 변할 수 있다. 이를 방지하기 위해 풀다운 또는 풀업 저항을 사용한다.

**Pull-down 회로**

```text
3.3V
  │
Button
  │
  ├──── GPIO
  │
Resistor
  │
 GND
```

- 버튼을 누르지 않으면 저항을 통해 GPIO가 GND에 연결되어 LOW로 유지된다.
- 버튼을 누르면 GPIO가 전원에 연결되어 HIGH가 된다.

**Pull-up 회로**

```text
3.3V
  │
Resistor
  │
  ├──── GPIO
  │
Button
  │
 GND
```

- 버튼을 누르지 않으면 HIGH로 유지된다.
- 버튼을 누르면 GPIO가 GND에 연결되어 LOW가 된다.

| 구분 | 버튼을 누르지 않음 | 버튼을 누름 |
|---|---|---|
| Pull-down | LOW | HIGH |
| Pull-up | HIGH | LOW |

ESP32에서는 내부 풀업·풀다운을 지원하는 GPIO에 다음 설정을 사용할 수 있다.

```cpp
pinMode(button_pin, INPUT_PULLUP);
```

```cpp
pinMode(button_pin, INPUT_PULLDOWN);
```

### 5-3. Bouncing과 Debouncing

**Bouncing**은 버튼의 기계적 접점이 붙거나 떨어지는 순간 접촉 상태가 여러 번 바뀌는 현상이다.

버튼을 한 번 눌렀더라도 MCU에서는 여러 번 입력된 것으로 인식할 수 있다.

```text
정상적인 입력

HIGH              ┌──────────
LOW  ─────────────┘

Bouncing이 발생한 입력

HIGH              ┌─┐ ┌─┐ ┌──
LOW  ─────────────┘ └─┘ └─┘
```

**Debouncing**은 이러한 불필요한 신호 변화를 제거하는 방식이다.

입력 상태가 일정 시간 동안 유지되었을 때만 실제 버튼 입력으로 판단하도록 처리할 수 있다.

## 6. Serial Communication

시리얼 통신은 데이터를 순서대로 전송하는 직렬 통신 방식이다. 실습에서는 ESP32가 읽은 버튼 및 센서 값을 PC의 Serial Monitor에서 확인하는 데 사용한다.

### 6-1. Baud Rate

Baud Rate는 초당 전송되는 심벌 수를 의미한다. 일반적인 UART 통신에서는 비트 전송 속도와 같은 수치로 사용된다.

대표적으로 9600, 115200 baud 등이 사용된다.

송신 측과 수신 측의 통신 속도를 동일하게 설정해야 정상적으로 데이터를 확인할 수 있다.

### 6-2. Serial 함수

| 함수 | 기능 |
|---|---|
| `Serial.begin(115200)` | 시리얼 통신 초기화 |
| `Serial.print()` | 데이터 출력 |
| `Serial.println()` | 데이터 출력 후 줄바꿈 |

버튼의 입력 상태나 가변저항의 ADC 값을 출력하여 코드가 의도한 대로 동작하는지 확인할 수 있다.

## 7. 가변저항과 ADC

### 7-1. 가변저항(Potentiometer)

회전축의 위치에 따라 저항의 비율을 변경할 수 있는 부품이다.

가변저항의 양 끝을 전원과 GND에 연결하고 가운데 단자를 ADC 핀에 연결하면 회전 위치에 따라 변하는 전압을 읽을 수 있다.

```text
3.3V ────── 가변저항 ────── GND
                │
                ↓
            ADC Pin
```

### 7-2. Analog Input

`analogRead()`는 ADC를 이용해 아날로그 입력 전압을 디지털 값으로 변환한다.

```cpp
int adc_value = analogRead(pot_pin);
```

ADC 해상도가 n비트일 때 출력 범위는 다음과 같다.

**0 \sim (2^n - 1)**

| 해상도 | 출력 범위 |
|---|---|
| 8 bit | 0 ~ 255 |
| 10 bit | 0 ~ 1023 |
| 12 bit | 0 ~ 4095 |

ESP32의 기본 `analogRead()` 해상도는 일반적으로 12비트이며, 설정에 따라 변경할 수 있다.

가변저항의 위치를 바꾸면 ADC 핀에 입력되는 전압이 달라지고, 이에 따라 `analogRead()`의 반환값도 변한다.

## 8. PWM(Pulse Width Modulation)

PWM은 디지털 신호의 HIGH와 LOW 상태를 빠르게 반복하면서 HIGH 상태의 시간 비율을 조절하는 방식이다.

LED의 밝기나 모터의 출력 등을 조절할 때 사용한다.

### 8-1. Duty Cycle

Duty Cycle은 한 주기에서 HIGH 상태가 차지하는 시간의 비율이다.

**D = \frac{T_{on}}{T} \times 100(\%)**

- **T_{on}**: HIGH 상태가 유지되는 시간
- **T**: PWM 신호의 전체 주기

```text
Duty Cycle 25%

    ┌──┐      ┌──┐
────┘  └──────┘  └──────

Duty Cycle 50%

    ┌────┐    ┌────┐
────┘    └────┘    └────

Duty Cycle 75%

    ┌──────┐  ┌──────┐
────┘      └──┘      └──
```

| Duty Cycle | 출력 상태 |
|---|---|
| 0% | 항상 LOW |
| 25% | 한 주기의 25% 동안 HIGH |
| 50% | 한 주기의 50% 동안 HIGH |
| 75% | 한 주기의 75% 동안 HIGH |
| 100% | 항상 HIGH |

Duty Cycle을 높이면 LED에 흐르는 평균 전류가 증가하여 밝기를 조절할 수 있다.

### 8-2. PWM과 DAC의 차이

- **PWM**: HIGH와 LOW를 반복하며 평균 출력을 조절하는 방식
- **DAC(Digital-to-Analog Converter)**: 디지털 값을 아날로그 전압으로 변환하는 장치

PWM은 아날로그 출력처럼 활용할 수 있지만 신호 자체는 디지털이다.

일반 ESP32에는 GPIO25와 GPIO26에 연결된 8비트 DAC가 내장되어 있다.

## 9. 가변저항을 이용한 LED 밝기 조절

### 9-1. ADC와 PWM 범위 변환

ESP32에서 기본 12비트 ADC와 8비트 PWM을 사용하는 경우 두 값의 범위가 서로 다르다.

| 구분 | 해상도 | 값의 범위 |
|---|---|---|
| ADC Input | 12 bit | 0 ~ 4095 |
| PWM Output | 8 bit | 0 ~ 255 |

따라서 ADC에서 읽은 값을 PWM 출력 범위에 맞게 변환해야 한다.

```text
가변저항
   ↓
analogRead()
   ↓
ADC Value (0 ~ 4095)
   ↓
map()
   ↓
PWM Value (0 ~ 255)
   ↓
analogWrite()
   ↓
LED 밝기 조절
```

### 9-2. map() 함수

`map()`은 입력값을 지정된 출력 범위로 비례 변환하는 함수이다.

```cpp
map(value, fromLow, fromHigh, toLow, toHigh);
```

| Parameter | 의미 |
|---|---|
| `value` | 변환할 값 |
| `fromLow` | 입력 범위 최솟값 |
| `fromHigh` | 입력 범위 최댓값 |
| `toLow` | 출력 범위 최솟값 |
| `toHigh` | 출력 범위 최댓값 |

### 9-3. LED 밝기 조절 코드

12비트 ADC와 8비트 PWM을 사용하는 경우의 기본 코드이다.

```cpp
int adc_value = analogRead(pot_pin);
int pwm_value = map(adc_value, 0, 4095, 0, 255);

analogWrite(led_pin, pwm_value);
```

`pot_pin`과 `led_pin`은 실제 회로에서 사용한 핀 번호로 미리 정의해야 한다.

가변저항의 회전 위치에 따라 ADC 값이 변하고, 변환된 PWM 값이 LED 출력에 반영된다.

## 10. Arduino IDE 개발 환경 구성

### 10-1. Linux 설치 및 실행

| 구분 | 설명 |
|---|---|
| `/opt` | 추가 소프트웨어 설치에 사용하는 디렉토리 |
| `sudo` | 관리자 권한으로 명령 실행 |
| `chmod +x` | 파일에 실행 권한 추가 |
| `#!/bin/sh` | 스크립트 실행에 사용할 인터프리터 지정 |

실행 스크립트를 작성하고 시스템 명령어 검색 경로에 심볼릭 링크를 등록하면 다른 디렉토리에서도 명령어처럼 실행할 수 있다.

### 10-2. Serial Port 권한 설정

ESP32를 USB로 연결하면 Linux에서 다음과 같은 장치 파일로 인식될 수 있다.

```text
/dev/ttyUSB0
/dev/ttyACM0
```

시리얼 포트에 대한 접근 권한이 없으면 Arduino IDE에서 보드를 연결하거나 코드를 업로드할 때 오류가 발생할 수 있다.

Ubuntu에서는 사용자를 `dialout` 그룹에 추가하여 접근 권한을 설정한다.

```bash
sudo usermod -aG dialout $USER
```

설정 이후 로그아웃 및 재로그인하거나 재부팅하면 변경된 그룹 권한이 적용된다.

## 11. PCB 키트 구성품

### 11-1. IMU(Inertial Measurement Unit)

로봇의 움직임과 자세를 추정하는 데 사용하는 센서이다.

| 센서 | 측정 정보 |
|---|---|
| Accelerometer | X, Y, Z축 가속도 |
| Gyroscope | X, Y, Z축 회전 각속도 |
| Magnetometer | 자기장 |

가속도계의 측정값에는 중력 성분도 포함된다. 지자기 센서는 주변 금속이나 자성 물체의 영향을 받을 수 있어 실내 환경에서는 측정 오차에 주의해야 한다.

IMU의 구성에 따라 지자기 센서가 포함되지 않을 수도 있다.

### 11-2. Servo Motor

목표 각도나 위치를 지정하여 제어할 수 있는 모터이다.

- 실습용 서보 모터는 일반적으로 약 0~180° 범위에서 동작한다.
- 제어 신호의 펄스 폭에 따라 목표 각도를 지정한다.
- 일반적인 RC 서보 모터는 약 50Hz의 제어 신호를 사용한다.

실습용 서보 모터의 배선은 다음과 같다.

| 색상 | 연결 |
|---|---|
| 갈색 | GND |
| 빨간색 | VCC |
| 노란색 | Signal |

제품마다 동작 범위, 정격 전압 및 소비 전류가 다르므로 연결 전에 사양을 확인해야 한다.

### 11-3. LiDAR

LiDAR(Light Detection and Ranging)는 레이저를 이용해 주변 물체까지의 거리를 측정하는 센서이다.

회전형 2D LiDAR는 주변을 스캔하여 거리 정보를 수집하며, 로봇에서는 장애물 감지, 위치 추정, 지도 작성 등에 사용한다.

### 11-4. Buzzer

전기적 신호를 소리로 변환하는 장치이다.

실습에서는 5번 핀에 연결하며, 버저 종류에 따라 출력 신호나 주파수를 조절해 소리를 발생시킬 수 있다.
