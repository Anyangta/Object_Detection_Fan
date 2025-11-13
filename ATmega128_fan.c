/**
 * @file main.c
 * @brief ATmega128 Full Fan & Servo Controller (Final Version)
 * @details
 * - ATmega128 @ 16MHz
 * - 24V BLDC Fan (8kHz PWM, Inverted Duty Cycle Control)
 * - Servo Motor (50Hz PWM, SPI Control)
 * - Bidirectional SPI communication with Raspberry Pi
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* -------------------------------------------------------------------------- */
/* 핀 및 설정값 정의 */
/* -------------------------------------------------------------------------- */

// SPI 상태 코드 (ATmega -> RPi)
#define STATUS_RUNNING    222  // 작동 중
#define STATUS_READY      111  // 준비 완료 (팬 꺼짐 + 90도 + 사용자 준비)
#define STATUS_HOMING_OFF 0    // 정지/복귀 중

// 선풍기 제어 핀
#define SWITCH_DDR         DDRD
#define SWITCH_PIN         PIND
#define SWITCH_SPEED_PIN   PD0  // 속도 조절 버튼
#define SWITCH_TOGGLE_PIN  PD1  // 시스템 활성화 버튼

#define LED_DDR            DDRC
#define LED_PORT           PORTC
#define LED_LOW_PIN        PC0  // 미풍 LED
#define LED_MEDIUM_PIN     PC1  // 약풍 LED
#define LED_HIGH_PIN       PC2  // 강풍 LED

#define FAN_PWM_DDR         DDRE
#define FAN_PWM_PORT        PORTE
#define FAN_PWM_PIN         PE3  // Timer3 OC3A

// 선풍기 PWM 설정 (8kHz, 듀티비 반전)
#define ICR_8KHZ          1999
#define DUTY_LOW          (uint16_t)(ICR_8KHZ * 0.2)  // 20% -> 강풍
#define DUTY_MEDIUM       (uint16_t)(ICR_8KHZ * 0.4)  // 40% -> 약풍
#define DUTY_HIGH         (uint16_t)(ICR_8KHZ * 0.6)  // 60% -> 미풍

// 서보모터 제어 핀
#define SERVO_DDR          DDRB
#define SERVO_PIN          DDB5  // Timer1 OC1A

// 서보모터 PWM 설정 (50Hz)
#define SERVO_CW_MAX     610  // 170도
#define SERVO_CCW_MAX    140  // 10도
#define SERVO_CENTER     375  // 90도

// SPI 통신 핀
#define SPI_DDR            DDRB
#define SPI_PIN_SS         DDB0
#define SPI_PIN_SCK        DDB1
#define SPI_PIN_MOSI       DDB2
#define SPI_PIN_MISO       DDB3

/* -------------------------------------------------------------------------- */
/* 전역 변수 */
/* -------------------------------------------------------------------------- */

volatile uint8_t motor_running = 0;        // 팬 동작 상태
volatile uint8_t speed_level = 0;          // 속도 단계 (0, 1, 2)
volatile uint8_t user_ready_flag = 0;      // 사용자 준비 상태
volatile uint8_t servo_homing_required = 0; // 서보 복귀 필요 플래그

volatile uint16_t servo_current_ocr;       // 서보 현재 위치
volatile uint16_t servo_target_ocr;        // 서보 목표 위치

volatile uint8_t current_spi_status = 0;   // 현재 상태 코드

/* -------------------------------------------------------------------------- */
/* 함수 선언 */
/* -------------------------------------------------------------------------- */

void init_ports(void);
void init_spi_slave(void);
void init_timer1_servo(void);
void init_timer3_fan_pwm(void);
void set_fan_speed(uint8_t level);
void update_leds(void);
void start_fan(void);
void stop_fan(void);

/* -------------------------------------------------------------------------- */
/* SPI 인터럽트 서비스 루틴 */
/* -------------------------------------------------------------------------- */

ISR(SPI_STC_vect) {
    uint8_t received_data = SPDR;

    if (received_data == 200) {
        // [명령 200]: 리셋 (자동 정지)
        if (motor_running) {
            stop_fan();
        }
        user_ready_flag = 0;
        servo_homing_required = 1;  // 90도 복귀 시작
        
    } else if (received_data >= 10 && received_data <= 170) {
        // [명령 10-170]: 각도 설정
        // 작동 중이고 준비 상태일 때만 각도 명령 수신
        if (user_ready_flag == 1 && motor_running == 1) {
            long ocr_value;
            ocr_value = (long)(received_data - 10) * (SERVO_CW_MAX - SERVO_CCW_MAX) / 160 + SERVO_CCW_MAX;
            servo_target_ocr = ocr_value;
        }
    } 
    else if (received_data == 255) {
        // [명령 255]: 팬 시작
        if (current_spi_status == STATUS_READY) {
            start_fan();
        }
    }
    // 0: 폴링 명령 (응답만 보냄)

    // 다음 응답 준비
    SPDR = current_spi_status;
}

/* -------------------------------------------------------------------------- */
/* 메인 함수 */
/* -------------------------------------------------------------------------- */

int main(void) {
    // 하드웨어 초기화
    init_ports();
    init_timer1_servo();
    init_timer3_fan_pwm();
    init_spi_slave();

    // 서보 초기 위치
    servo_current_ocr = SERVO_CENTER;
    servo_target_ocr = SERVO_CENTER;
    OCR1A = servo_current_ocr;
    
    // 시스템 초기 상태
    stop_fan();
    user_ready_flag = 0;
    servo_homing_required = 0;

    // 첫 SPI 응답 준비
    SPDR = STATUS_HOMING_OFF;
    
    // 전역 인터럽트 활성화
    sei();

    // 버튼 디바운싱 변수
    uint8_t toggle_button_pressed = 0;
    uint8_t speed_button_pressed = 0;

    while (1) {
        
        // ===== PD1 버튼 처리 (시스템 ON/OFF) =====
        if (SWITCH_PIN & (1 << SWITCH_TOGGLE_PIN)) {
            if (!toggle_button_pressed) {
                toggle_button_pressed = 1;
                _delay_ms(50);
                
                if (user_ready_flag == 1) {
                    // 켜져있으면 끄기
                    user_ready_flag = 0;
                    if (motor_running) {
                        stop_fan();
                    }
                    servo_homing_required = 1;
                } else {
                    // 꺼져있으면 켜기 (90도 복귀 시작)
                    user_ready_flag = 1;
                    servo_target_ocr = SERVO_CENTER;
                    servo_homing_required = 1;
                }
            }
        } else {
            toggle_button_pressed = 0;
        }

        // ===== 팬 작동 상태별 처리 =====
        if (motor_running) {
            // ----- 팬 작동 중 -----
            
            // PD0 버튼: 속도 조절
            if (SWITCH_PIN & (1 << SWITCH_SPEED_PIN)) {
                if (!speed_button_pressed) {
                    speed_button_pressed = 1;
                    _delay_ms(50);
                    speed_level++;
                    if (speed_level > 2) speed_level = 0;
                    set_fan_speed(speed_level);
                }
            } else {
                speed_button_pressed = 0;
            }
            
            // 서보 추적 (SPI 명령 따라가기) - 1씩 천천히
            if (servo_current_ocr < servo_target_ocr) {
                servo_current_ocr++;
                OCR1A = servo_current_ocr;
            } else if (servo_current_ocr > servo_target_ocr) {
                servo_current_ocr--;
                OCR1A = servo_current_ocr;
            }
            
            current_spi_status = STATUS_RUNNING;

        } else {
            // ----- 팬 정지 중 -----
            
            // 90도 복귀 처리 - 1씩 천천히
            if (servo_homing_required || user_ready_flag == 1) {
                if (servo_current_ocr < SERVO_CENTER) {
                    servo_current_ocr++;
                    OCR1A = servo_current_ocr;
                } else if (servo_current_ocr > SERVO_CENTER) {
                    servo_current_ocr--;
                    OCR1A = servo_current_ocr;
                } else {
                    // 90도 도착 완료
                    servo_homing_required = 0;
                }
            }
            
            // 상태 결정
            if (user_ready_flag == 1 && servo_current_ocr == SERVO_CENTER) {
                current_spi_status = STATUS_READY;  // 준비 완료
            } else {
                current_spi_status = STATUS_HOMING_OFF;  // 정지/복귀 중
            }
        }
        
        _delay_ms(2);  // 2ms 딜레이 (서보 속도 조절)
    }
}

/* -------------------------------------------------------------------------- */
/* 함수 정의 */
/* -------------------------------------------------------------------------- */

void init_ports(void) {
    // 출력 핀
    FAN_PWM_DDR |= (1 << FAN_PWM_PIN);
    LED_DDR |= (1 << LED_LOW_PIN) | (1 << LED_MEDIUM_PIN) | (1 << LED_HIGH_PIN);
    SERVO_DDR |= (1 << SERVO_PIN);
    
    // 입력 핀 (외부 풀다운)
    SWITCH_DDR &= ~((1 << SWITCH_SPEED_PIN) | (1 << SWITCH_TOGGLE_PIN));
}

void init_spi_slave(void) {
    SPI_DDR |= (1 << SPI_PIN_MISO);  // MISO 출력
    SPI_DDR &= ~((1 << SPI_PIN_SS) | (1 << SPI_PIN_SCK) | (1 << SPI_PIN_MOSI));  // 나머지 입력
    SPCR |= (1 << SPE) | (1 << SPIE);  // SPI 활성화, 인터럽트 활성화
}

void init_timer1_servo(void) {
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    ICR1 = 4999;  // 50Hz
}

void init_timer3_fan_pwm(void) {
    TCCR3A = (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32);
    ICR3 = ICR_8KHZ;  // 8kHz
}

void set_fan_speed(uint8_t level) {
    switch (level) {
        case 0: OCR3A = DUTY_HIGH; break;    // 미풍 (60%)
        case 1: OCR3A = DUTY_MEDIUM; break;  // 약풍 (40%)
        case 2: OCR3A = DUTY_LOW; break;     // 강풍 (20%)
    }
    update_leds();
}

void update_leds(void) {
    LED_PORT &= ~((1 << LED_LOW_PIN) | (1 << LED_MEDIUM_PIN) | (1 << LED_HIGH_PIN));
    if (!motor_running) return;

    switch (speed_level) {
        case 2:
            LED_PORT |= (1 << LED_HIGH_PIN);
        case 1:
            LED_PORT |= (1 << LED_MEDIUM_PIN);
        case 0:
            LED_PORT |= (1 << LED_LOW_PIN);
            break;
    }
}

void start_fan(void) {
    motor_running = 1;
    speed_level = 0;
    
    TCCR3A |= (1 << COM3A1);
    set_fan_speed(speed_level);
    TCCR3B |= (1 << CS30);
}

void stop_fan(void) {
    motor_running = 0;
    
    TCCR3B &= ~(1 << CS30);
    TCCR3A &= ~(1 << COM3A1);
    FAN_PWM_PORT &= ~(1 << FAN_PWM_PIN);
    
    speed_level = 0;
    update_leds();
}