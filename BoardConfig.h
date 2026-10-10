#pragma once
#include <Arduino.h>

// Единственное место настройки распиновки. Этот файл виден отдельной вкладкой
// Arduino IDE. Сначала подключаем внешнюю карту сборки, затем задаём defaults:
// test/compile_only_board.h используется только для проверки компиляции.
#ifdef HARDWARE_API_BOARD_HEADER
#include HARDWARE_API_BOARD_HEADER
#endif

// Транспорт Austere: при переносе определите ВСЕ пять макросов одновременно.
// Частичная карта запрещена: иначе часть проводов незаметно осталась бы старой.
#if defined(API_LINK_MISO) || defined(API_LINK_MOSI) || defined(API_LINK_CLK) || defined(API_LINK_CS) || defined(API_LINK_IRQ)
#if !defined(API_LINK_MISO) || !defined(API_LINK_MOSI) || !defined(API_LINK_CLK) || !defined(API_LINK_CS) || !defined(API_LINK_IRQ)
#error "define all five API_LINK_* pins together"
#endif
#else
#define API_LINK_MISO 2
#define API_LINK_MOSI 3
#define API_LINK_CLK 4
#define API_LINK_CS 5
#define API_LINK_IRQ 7
#endif

// Arduino IDE: выберите 1 (четыре вентилятора) или 2 (два вентилятора).
// Один скетч обслуживает обе платы; после изменения обязательно перепрошить
// именно выбранную Arduino. Внешние индексы 4/5 преобразуются хабом в 0/1.
#ifndef API_ACTUATOR_BOARD
#define API_ACTUATOR_BOARD 1
#endif
#if API_ACTUATOR_BOARD == 1
#define API_LOCAL_NODE 14
#define API_FAN_COUNT 4
#define API_GLOBAL_FAN_OFFSET 0
#ifndef API_PWM_PINS
#define API_PWM_PINS {10, 11, 6, 9}
#endif
#ifndef API_TACH_PINS
#define API_TACH_PINS {12, 13, A4, 8}
#endif
#ifndef API_RELAY_PINS
#define API_RELAY_PINS {A0, A1, A2, A3}
#endif
#elif API_ACTUATOR_BOARD == 2
#define API_LOCAL_NODE 17
#define API_FAN_COUNT 2
#define API_GLOBAL_FAN_OFFSET 4
#ifndef API_PWM_PINS
#define API_PWM_PINS {10, 11}
#endif
#ifndef API_TACH_PINS
#define API_TACH_PINS {12, 13}
#endif
#ifndef API_RELAY_PINS
#define API_RELAY_PINS {A0, A1}
#endif
#else
#error "API_ACTUATOR_BOARD must be 1 or 2"
#endif

// Число импульсов тахометра на оборот (это параметр датчика, не регулирование).
// Перед включением вентиляторов установите PinChangeInterrupt 1.2.9.
#ifndef API_PULSES_PER_REV
#define API_PULSES_PER_REV 2
#endif
static_assert(API_PULSES_PER_REV > 0, "tach pulses/revolution must be positive");

// Неполная карта — ошибка сборки, а не чтение за границами массива.
namespace actuator_config {
constexpr uint8_t pwmPins[] = API_PWM_PINS;
constexpr uint8_t tachPins[] = API_TACH_PINS;
constexpr uint8_t relayPins[] = API_RELAY_PINS;
static_assert(sizeof(pwmPins) == API_FAN_COUNT, "wrong PWM pin count");
static_assert(sizeof(tachPins) == API_FAN_COUNT, "wrong tach pin count");
static_assert(sizeof(relayPins) == API_FAN_COUNT, "wrong relay pin count");
}
