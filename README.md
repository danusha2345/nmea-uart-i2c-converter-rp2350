# NMEA UART to I2C Converter for RP2350-Zero

[![Build RP2350 Firmware](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/actions/workflows/build.yml/badge.svg)](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/actions/workflows/build.yml)

Конвертер интерфейса UART в I2C для передачи NMEA данных от GNSS приемника на устройство PortaPack H4M. Оптимизирован для платы Waveshare RP2350-Zero.

## 🚀 Быстрый старт

### 1. Скачайте готовую прошивку

1. Перейдите в раздел [Actions](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/actions)
2. Выберите последнюю успешную сборку "Build RP2350 Firmware"
3. Скачайте артефакт `rp2350-firmware-*`
4. Распакуйте архив и найдите файл `.uf2`

### 2. Загрузите прошивку на RP2350-Zero

1. Подключите RP2350-Zero к компьютеру через USB Type-C
2. Зажмите кнопку **BOOT** и нажмите **RESET**
3. Отпустите обе кнопки - появится диск **RPI-RP2**
4. Скопируйте файл `.uf2` на этот диск
5. Плата автоматически перезагрузится

## 🔧 Характеристики v3.0

- **Микроконтроллер**: RP2350 (Dual-core ARM Cortex-M33)
- **UART**: 115200 бод (настраивается в коде)
- **I2C адрес**: 0x42 (настраивается в коде)
- **Размер буфера**: 4096 байт (циклический, thread-safe)
- **I2C пакет**: 96 байт (оптимально для Wire библиотеки)
- **Максимальная длина NMEA**: 255 символов (поддержка современных GNSS)
- **Питание**: 3.3V или через USB Type-C

### 📊 Важные параметры

| Параметр | Значение | Описание |
|----------|----------|----------|
| `I2C_PACKET_SIZE` | 96 байт | Максимальный размер данных за одну I2C транзакцию |
| `NMEA_MAX_LENGTH` | 255 символов | Максимальная длина NMEA предложения |
| `NMEA_BUFFER_SIZE` | 4096 байт | Размер циклического буфера |
| `I2C_SLAVE_ADDRESS` | 0x42 | I2C адрес устройства |

**Важно**: 
- `I2C_PACKET_SIZE` - это размер пакета для передачи по I2C (ограничение Wire библиотеки)
- `NMEA_MAX_LENGTH` - это максимальная длина NMEA строки от GPS модуля

## 📌 Схема подключения

```
GNSS Module         RP2350-Zero         PortaPack H4M
-----------         -----------         --------------
VCC (3.3V) -------> 3V3                   
GND --------------> GND ----------------> GND
TX ---------------> GPIO1 (RX)           
                   GPIO4 (SDA) ---------> SDA
                   GPIO5 (SCL) ---------> SCL
                   3V3 ------------------> 3.3V (опционально)
```

**Примечание**: Внешние подтягивающие резисторы НЕ требуются - используются внутренние pull-up резисторы RP2350.

## 🏗️ Архитектура

### Особенности версии 3.0:

- **Двухъядерная подготовка**: Инфраструктура для использования второго ядра (сейчас в резерве)
- **Thread-safe буфер**: Безопасная работа с данными между ядрами
- **Расширенная поддержка NMEA**: До 255 символов для современных мультисистемных GNSS
- **Оптимизированные I2C пакеты**: 96 байт для эффективной передачи

### Использование ядер:

- **Ядро 0**: Вся основная обработка (UART, I2C, буфер)
- **Ядро 1**: Зарезервировано для будущих функций

## 📡 I2C протокол

### Команды

| Команда | Код | Описание | Ответ |
|---------|-----|----------|-------|
| GET_STATUS | 0x01 | Получить статус устройства | 1 байт статуса |
| GET_COUNT | 0x02 | Количество байт в буфере | 2 байта (little-endian) |
| READ_DATA | 0x03 | Прочитать данные из буфера | До 96 байт NMEA данных |
| CLEAR_BUFFER | 0x04 | Очистить буфер | 0x01 (успех) |
| GET_VERSION | 0x05 | Версия прошивки | 0x30 (версия 3.0) |
| GET_INFO | 0x06 | Информация об устройстве | Строка "RP2350-Zero,UP:время,SC:счетчик" |

### Статусные биты

- **Bit 0**: Data Ready - данные доступны
- **Bit 1**: Buffer Overflow - переполнение буфера
- **Bit 2**: Checksum Error - ошибка контрольной суммы NMEA
- **Bit 3**: UART Error - ошибка UART или слишком длинное предложение
- **Bit 4**: I2C Busy - I2C транзакция в процессе

## 🧪 Тестирование

### Тест без GPS модуля

В папке `test/nmea_test_generator` находится генератор NMEA данных:

1. Загрузите генератор на любую плату (RP2040, ESP32 и т.д.)
2. Подключите TX генератора к RX (GPIO1) конвертера
3. Соедините GND обеих плат

### Проверка работы

Подключите Serial Monitor (115200 бод):

```
=====================================
  NMEA UART to I2C Converter v3.0  
    For Waveshare RP2350-Zero      
=====================================
[OK] Hardware initialized
     - I2C internal pull-ups enabled
[OK] Buffer initialized (4KB)
[OK] UART initialized
     - RX: GPIO1
     - TX: GPIO0
     - Baud: 115200
[OK] I2C slave initialized
     - Address: 0x42
     - SDA: GPIO4
     - SCL: GPIO5
     - I2C packet size: 96 bytes
     - Max NMEA length: 255 characters

[INFO] Features:
     - Dual-core support (Core 1 reserved)
     - Thread-safe circular buffer
     - Extended NMEA support (255 chars)
     - NMEA checksum validation

[READY] Waiting for NMEA data...
```

## 📈 История версий

### v3.0 (текущая) - Унифицированная версия
- **Объединены** все улучшения предыдущих версий
- **Исправлена** путаница в нумерации версий
- **Чёткое разделение**: I2C_PACKET_SIZE (96) vs NMEA_MAX_LENGTH (255)
- **Подготовка** для активного использования второго ядра
- **Улучшенная документация** в коде и выводе

### Предыдущие версии (устаревшие)
- v2.5 - Увеличен NMEA_MAX_LENGTH до 255
- v2.4 - Базовая поддержка dual-core
- v1.3 - Первая попытка dual-core
- v1.2 - Путаница с описанием (неверно указано "255-byte I2C packets")
- v1.1 - Увеличен I2C_PACKET_SIZE до 96 байт
- v1.0 - Начальная версия

## 🛠️ Сборка из исходников

### Через Arduino CLI

```bash
# Установка Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Настройка для RP2350
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040

# Компиляция
arduino-cli compile --fqbn rp2040:rp2040:rpipico2 ./nmea_converter_rp2350_zero
```

### Через Arduino IDE

1. Установите поддержку RP2040/RP2350
2. Выберите плату "Raspberry Pi Pico 2"
3. Откройте `nmea_converter_rp2350_zero.ino`
4. Нажмите "Загрузить"

## 🔍 Отладка

Для включения вывода NMEA предложений добавьте в начало кода:
```cpp
#define DEBUG_OUTPUT
```

## ⚠️ Известные особенности

- RP2350-Zero не имеет встроенного LED (только RGB WS2812)
- Для индикации можно подключить внешний LED к GPIO16
- Максимальная длина I2C кабеля с внутренними pull-up: ~30 см
- Для длинных кабелей добавьте внешние резисторы 2.2kΩ

## 🔮 Планы развития

- Активное использование второго ядра для:
  - Фильтрации NMEA сообщений
  - Кэширования последних координат
  - Расчёта статистики
- Поддержка нескольких GNSS приёмников
- Web-интерфейс для настройки
- OTA обновления

## 📝 Лицензия

MIT License - свободно используйте в своих проектах!

## 🤝 Вклад в проект

Принимаются Pull Request'ы! Пожалуйста:
1. Форкните репозиторий
2. Создайте ветку для изменений
3. Протестируйте
4. Отправьте Pull Request

## 📞 Поддержка

Создайте [Issue](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/issues) если нашли проблему или есть предложения.
