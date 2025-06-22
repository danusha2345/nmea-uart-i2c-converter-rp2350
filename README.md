# NMEA UART to I2C Converter for RP2350-Zero

[![Build RP2350 Firmware](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/actions/workflows/build.yml/badge.svg)](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/actions/workflows/build.yml)

Конвертер интерфейса UART в I2C для передачи NMEA данных от GNSS приемника на устройство PortaPack H4M. Оптимизирован для платы Waveshare RP2350-Zero.

## 🚀 Быстрый старт

### 1. Скачайте готовую прошивку

1. Перейдите в раздел [Actions](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/actions)
2. Выберите последнюю успешную сборку
3. Скачайте артефакт `rp2350-firmware-*`
4. Распакуйте архив и найдите файл `.uf2`

### 2. Загрузите прошивку на RP2350-Zero

1. Подключите RP2350-Zero к компьютеру через USB Type-C
2. Зажмите кнопку **BOOT** и нажмите **RESET**
3. Отпустите обе кнопки - появится диск **RPI-RP2**
4. Скопируйте файл `.uf2` на этот диск
5. Плата автоматически перезагрузится

## 🧪 Тестирование без GNSS модуля

В папке `test/nmea_test_generator` находится генератор NMEA данных для тестирования конвертера.

### Что нужно для теста:
- RP2350-Zero с прошивкой конвертера
- Любая плата: RP2040, ESP32-C3, Arduino и т.д.
- 2 провода для соединения

### Схема подключения для теста:

```
Генератор NMEA          Конвертер (RP2350-Zero)
--------------          -----------------------
TX (GPIO0/1) ---------> RX (GPIO1)
GND ------------------> GND
```

### Инструкция по тестированию:

1. **Загрузите тест-генератор** на вторую плату:
   - Откройте `test/nmea_test_generator/nmea_test_generator.ino`
   - Выберите вашу плату в Arduino IDE
   - Загрузите скетч

2. **Подключите Serial Monitor** к RP2350-Zero (115200 бод)

3. **Вы увидите**:
   - На генераторе: отправляемые NMEA данные
   - На конвертере: статистику приема
   ```
   [STATUS] Sentences: 10, Bytes: 750, Buffer: 750/4096, Errors: 0
   ```

### Что генерирует тестер:
- **GPGGA** - данные о местоположении
- **GPRMC** - навигационные данные
- **GPGSV** - информация о спутниках
- **GPVTG** - скорость и курс

Данные отправляются каждую секунду с симуляцией движения.

## 📌 Схема подключения (рабочая)

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

**Важно**: Внешние подтягивающие резисторы НЕ требуются - используются внутренние pull-up резисторы RP2350.

## 🔧 Характеристики

- **Микроконтроллер**: RP2350 (Dual-core ARM Cortex-M33 + RISC-V)
- **UART**: 115200 бод (настраивается)
- **I2C адрес**: 0x42 (настраивается в коде)
- **Буфер**: 4096 байт (циклический)
- **Питание**: 3.3V или через USB Type-C

## 📡 I2C протокол

### Команды

| Команда | Код | Описание | Ответ |
|---------|-----|----------|-------|
| GET_STATUS | 0x01 | Получить статус | 1 байт статуса |
| GET_COUNT | 0x02 | Количество байт в буфере | 2 байта (little-endian) |
| READ_DATA | 0x03 | Прочитать данные | До 32 байт NMEA данных |
| CLEAR_BUFFER | 0x04 | Очистить буфер | 0x01 (успех) |
| GET_VERSION | 0x05 | Версия прошивки | 1 байт версии |
| GET_INFO | 0x06 | Информация об устройстве | Строка с информацией |

### Статусные биты

- Bit 0: Data Ready (данные доступны)
- Bit 1: Buffer Overflow (переполнение буфера)
- Bit 2: Checksum Error (ошибка контрольной суммы)
- Bit 3: UART Error (ошибка UART)
- Bit 4: I2C Busy (I2C занят)

## 🛠️ Сборка из исходников

### Автоматическая сборка через GitHub Actions

1. Форкните этот репозиторий
2. Внесите изменения в код
3. Сделайте commit и push
4. GitHub Actions автоматически соберет прошивку
5. Скачайте готовый файл из Actions

### Локальная сборка

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

## 📊 Простая проверка работы

### 1. Через Serial Monitor
Подключите RP2350-Zero к компьютеру и откройте Serial Monitor (115200 бод):
```
=====================================
  NMEA UART to I2C Converter v2.3  
    For Waveshare RP2350-Zero      
=====================================
[OK] Hardware initialized
[OK] Buffer initialized (4KB)
[OK] UART initialized
[OK] I2C slave initialized
[READY] Waiting for NMEA data...
```

### 2. Сканирование I2C (Linux)
```bash
i2cdetect -y 1
# Должен показать устройство на адресе 0x42
```

### 3. Тест I2C Master
Используйте скетч из папки `test/i2c_master_test.ino` на другой плате для проверки I2C связи.

## 🔍 Отладка

Для включения отладочного вывода добавьте в начало кода:
```cpp
#define DEBUG_OUTPUT
```

## 📝 Лицензия

MIT License - свободно используйте в своих проектах!

## 🤝 Вклад в проект

Принимаются Pull Request'ы! Пожалуйста:
1. Форкните репозиторий
2. Создайте ветку для ваших изменений
3. Протестируйте изменения
4. Отправьте Pull Request

## ⚠️ Известные особенности

- RP2350-Zero не имеет встроенного светодиода - подключите внешний LED к GPIO16 для индикации
- Максимальная длина I2C кабеля с внутренними pull-up: 30 см
- Для длинных кабелей добавьте внешние резисторы 2.2kΩ

## 📞 Поддержка

Создайте [Issue](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/issues) если нашли проблему или есть предложения по улучшению.