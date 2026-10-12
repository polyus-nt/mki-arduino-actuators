#pragma once
#include <Arduino.h>
// Раскомментировать для ручных команд USB Serial (9600 бод).
// #define TEST

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
#define API_LINK_IRQ 6
#endif

// Фиксированная конфигурация отдельного проекта arduino-actuator-2.
// Это вторая Arduino: два вентилятора 4/5 (локально 0/1), шлюз 17.
// Выбор платы не требуется: открывайте .ino из папки нужного проекта.
#define API_LOCAL_NODE 17
#define API_FAN_COUNT 2
#define API_GLOBAL_FAN_OFFSET 4
#ifndef API_PWM_PINS
#define API_PWM_PINS {10, 11}
#endif
#ifndef API_TACH_PINS
// Та же аппаратная карта, что у двухвентиляторной Arduino химреактора.
#define API_TACH_PINS {8, 12}
#endif
#ifndef API_RELAY_PINS
#define API_RELAY_PINS {A0, A1}
#endif

// Число импульсов тахометра на оборот (это параметр датчика, не регулирование).
// Перед включением вентиляторов установите PinChangeInterrupt 1.2.9.
#ifndef API_PULSES_PER_REV
#define API_PULSES_PER_REV 2
#endif
static_assert(API_PULSES_PER_REV > 0, "tach pulses/revolution must be positive");

// Неполная карта — ошибка сборки, а не чтение за границами массива.
// Полярность реле питания: первая Arduino ФабЛаб LOW, остальные HIGH.
#define API_RELAY_ACTIVE_LOW 0
namespace actuator_config {
constexpr uint8_t relayOnLevel = API_RELAY_ACTIVE_LOW ? LOW : HIGH;
constexpr uint8_t relayOffLevel = API_RELAY_ACTIVE_LOW ? HIGH : LOW;
constexpr uint8_t pwmPins[] = API_PWM_PINS;
constexpr uint8_t tachPins[] = API_TACH_PINS;
constexpr uint8_t relayPins[] = API_RELAY_PINS;
static_assert(sizeof(pwmPins) == API_FAN_COUNT, "wrong PWM pin count");
static_assert(sizeof(tachPins) == API_FAN_COUNT, "wrong tach pin count");
static_assert(sizeof(relayPins) == API_FAN_COUNT, "wrong relay pin count");
}
