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

// Реальная разводка шести вентиляторов пока НЕ согласована. Для её включения
// задайте оба массива по шесть пинов; до этого GPIO не включаются и API
// возвращает NOT_IMPLEMENTED. Пины должны быть уникальны во всей карте:
// PWM — аппаратный analogWrite, таховходы — PinChangeInterrupt.
// #define API_PWM_PINS { ... шесть пинов PWM ... }
// #define API_TACH_PINS { ... шесть таховходов ... }
#if defined(API_PWM_PINS) != defined(API_TACH_PINS)
#error "both API_PWM_PINS and API_TACH_PINS must be defined"
#endif

// Число импульсов тахометра на оборот (это параметр датчика, не регулирование).
// Перед включением вентиляторов установите PinChangeInterrupt 1.2.9.
#ifndef API_PULSES_PER_REV
#define API_PULSES_PER_REV 2
#endif
static_assert(API_PULSES_PER_REV > 0, "tach pulses/revolution must be positive");
