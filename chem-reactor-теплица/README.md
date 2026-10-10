# Arduino вентиляторов теплицы-химреактора

Отдельная прошивка в `arduino-actuator`, не фаблаб-профиль второй UNO.
Один шлюз **14**, два вентилятора с индексами **0/1**.

Откройте [arduino-actuator.ino](arduino-actuator/arduino-actuator.ino),
выберите **Arduino Uno**, установите **PinChangeInterrupt 1.2.9** через Library Manager.
Соседние репозитории для сборки не нужны.
Карта пинов: [BoardConfig.h](arduino-actuator/BoardConfig.h).

| Вентилятор | PWM | Тахометр | Реле питания |
| --- | --- | --- | --- |
| 0 | D10 | D8 | A0 |
| 1 | D11 | D12 | A1 |

| Сигнал Austere | Пин UNO |
| --- | --- |
| MISO | D2 |
| MOSI | D3 |
| CLK | D4 |
| CS | D5 |
| IRQ | D7 |

PWM=0 → реле HIGH, выключено; PWM>0 → LOW, включено.
Старт: PWM=0. При потере связи последнее PWM удерживается.
Тахометр: API_PULSES_PER_REV=2, проверьте для своего вентилятора.
PID/температурных решений нет; режимы неисправности 0/1/2 сохранены.
Общий замок и релейный вентилятор находятся на lmp8, не здесь.

Нативные проверки из этой папки: `py -3 scripts/test_firmware.py --cxx g++`.
Другие репозитории не нужны.
Полная карта: `tjc-sketch/chem-reactor-теплица/PINOUT.md`.
Сервис/HTTP-консоль: `tjc-commutator/chem-reactor-теплица/`.
