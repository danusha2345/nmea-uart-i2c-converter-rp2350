# NMEA UART to I2C Converter для RP2350-Zero

[![Build NMEA Converter](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/actions/workflows/build.yml/badge.svg)](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/actions/workflows/build.yml)

Высокопроизводительный конвертер NMEA данных от GPS/GNSS приёмников с UART на I2C интерфейс. Оптимизирован для платы Waveshare RP2350-Zero с полноценным использованием двух ядер.

## 🚀 Быстрый старт

### Скачайте прошивку

1. Перейдите в [Releases](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/releases/latest)
2. Скачайте файл `nmea_converter.uf2`
3. Подключите RP2350-Zero к компьютеру, удерживая кнопку BOOT
4. Скопируйте .uf2 файл на появившийся диск RPI-RP2
5. Плата автоматически перезагрузится

## 🔧 Технические характеристики

| Параметр | Значение |
|----------|----------|
| **Версия** | 3.2 |
| **MCU** | RP2350 (Dual-core ARM Cortex-M33) |
| **I2C адрес** | 0x10 |
| **I2C пакет** | 255 байт (максимальный) |
| **UART** | 115200 бод |
| **Буфер** | 8192 байт |
| **NMEA макс.** | 255 символов |

### 🎯 Ключевые особенности

- **Полноценная двухъядерная обработка**
  - Ядро 0: Приём UART, валидация NMEA, запись в буфер
  - Ядро 1: Обработка I2C, статистика, фильтрация, кэширование позиции
- **255-байтные I2C пакеты** для максимальной эффективности передачи
- **Thread-safe операции** с использованием mutex
- **Кэширование позиции** с быстрым доступом через I2C
- **Фильтрация NMEA** по типам сообщений
- **Расширенная статистика** работы обоих ядер

## 📌 Подключение

```
GPS/GNSS модуль     RP2350-Zero        Устройство I2C
--------------      -----------        ---------------
VCC (3.3V) -------> 3V3                   
GND --------------> GND --------------> GND
TX ---------------> GPIO1 (RX)           
                    GPIO4 (SDA) -------> SDA
                    GPIO5 (SCL) -------> SCL
                    3V3 ---------------> VCC (опционально)
```

> **Примечание**: Внешние подтягивающие резисторы НЕ требуются - используются внутренние pull-up.

## 📡 I2C протокол

### Команды

| Команда | Код | Описание | Ответ |
|---------|-----|----------|-------|
| GET_STATUS | 0x01 | Получить статус | 1 байт флагов |
| GET_COUNT | 0x02 | Количество байт в буфере | 2 байта (little-endian) |
| READ_DATA | 0x03 | Прочитать данные | До 255 байт NMEA |
| CLEAR_BUFFER | 0x04 | Очистить буфер | 0x01 (успех) |
| GET_VERSION | 0x05 | Версия прошивки | 0x32 (v3.2) |
| GET_INFO | 0x06 | Информация об устройстве | Строка с uptime и статусом |
| GET_POSITION | 0x07 | Получить кэшированную позицию | 30 байт бинарных данных |
| GET_STATS | 0x08 | Получить статистику | 32 байта счётчиков |
| SET_FILTER | 0x09 | Установить фильтр NMEA | - |

### Статусные биты (команда 0x01)

| Бит | Название | Описание |
|-----|----------|----------|
| 0 | dataReady | Данные доступны в буфере |
| 1 | bufferOverflow | Произошло переполнение буфера |
| 2 | checksumError | Ошибка контрольной суммы NMEA |
| 3 | uartError | Ошибка UART или слишком длинное сообщение |
| 4 | i2cBusy | I2C транзакция в процессе |
| 5 | positionValid | Действительная позиция в кэше |
| 6 | filterActive | Фильтр NMEA активен |
| 7 | core1Active | Ядро 1 работает |

### Формат данных позиции (команда 0x07)

| Смещение | Размер | Тип | Описание |
|----------|--------|-----|----------|
| 0 | 8 | double | Широта (градусы) |
| 8 | 8 | double | Долгота (градусы) |
| 16 | 4 | float | Высота (метры) |
| 20 | 4 | float | Скорость (км/ч) |
| 24 | 4 | float | Курс (градусы) |
| 28 | 1 | uint8 | Количество спутников |
| 29 | 1 | uint8 | Качество фиксации |

### Фильтр NMEA (команда 0x09)

Отправьте 1 байт после команды для настройки фильтра:

| Бит | Тип NMEA |
|-----|----------|
| 0 | GGA |
| 1 | RMC |
| 2 | GSV |
| 3 | VTG |
| 4 | GLL |
| 5 | GSA |
| 6 | GPTXT |
| 7 | GNGGA |

Пример: 0xFF = все типы включены, 0x03 = только GGA и RMC

## 💻 Пример использования

### Arduino (I2C Master)

```cpp
#include <Wire.h>

#define NMEA_I2C_ADDR 0x10
#define CMD_GET_STATUS 0x01
#define CMD_GET_COUNT 0x02
#define CMD_READ_DATA 0x03
#define CMD_GET_POSITION 0x07

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  // Проверка статуса
  Wire.beginTransmission(NMEA_I2C_ADDR);
  Wire.write(CMD_GET_STATUS);
  Wire.endTransmission();
  
  Wire.requestFrom(NMEA_I2C_ADDR, 1);
  if (Wire.available()) {
    uint8_t status = Wire.read();
    if (status & 0x01) { // dataReady
      readNMEAData();
    }
    if (status & 0x20) { // positionValid
      readPosition();
    }
  }
  
  delay(100);
}

void readNMEAData() {
  // Получить количество байт
  Wire.beginTransmission(NMEA_I2C_ADDR);
  Wire.write(CMD_GET_COUNT);
  Wire.endTransmission();
  
  Wire.requestFrom(NMEA_I2C_ADDR, 2);
  uint16_t count = Wire.read() | (Wire.read() << 8);
  
  // Читать данные порциями по 255 байт
  while (count > 0) {
    Wire.beginTransmission(NMEA_I2C_ADDR);
    Wire.write(CMD_READ_DATA);
    Wire.endTransmission();
    
    uint16_t toRead = min(count, 255);
    Wire.requestFrom(NMEA_I2C_ADDR, toRead);
    
    while (Wire.available()) {
      Serial.print((char)Wire.read());
      count--;
    }
  }
}

void readPosition() {
  Wire.beginTransmission(NMEA_I2C_ADDR);
  Wire.write(CMD_GET_POSITION);
  Wire.endTransmission();
  
  Wire.requestFrom(NMEA_I2C_ADDR, 30);
  if (Wire.available() >= 30) {
    // Читаем позицию (упрощённый пример)
    uint8_t posData[30];
    for (int i = 0; i < 30; i++) {
      posData[i] = Wire.read();
    }
    
    double lat = *(double*)&posData[0];
    double lon = *(double*)&posData[8];
    uint8_t sats = posData[28];
    
    Serial.print("Position: ");
    Serial.print(lat, 6);
    Serial.print(", ");
    Serial.print(lon, 6);
    Serial.print(" Sats: ");
    Serial.println(sats);
  }
}
```

## 🛠️ Сборка из исходников

### Требования

- Arduino IDE 2.0+ или Arduino CLI
- Поддержка плат RP2040/RP2350 (Earle Philhower core)

### Установка поддержки RP2350

1. Откройте Arduino IDE
2. Файл → Настройки
3. В поле "Дополнительные ссылки для Менеджера плат" добавьте:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
4. Инструменты → Плата → Менеджер плат
5. Найдите "Raspberry Pi Pico/RP2040/RP2350" и установите

### Компиляция

1. Откройте `src/nmea_converter.ino`
2. Выберите плату: Инструменты → Плата → "Raspberry Pi Pico 2"
3. Нажмите "Загрузить"

### Через Arduino CLI

```bash
arduino-cli compile --fqbn rp2040:rp2040:rpipico2 ./src
arduino-cli upload --fqbn rp2040:rp2040:rpipico2 -p /dev/ttyACM0 ./src
```

## 📊 Мониторинг

Подключите к USB и откройте Serial Monitor (115200 бод):

```
=====================================
  NMEA UART to I2C Converter v3.2  
    Full Dual-Core Implementation   
=====================================
[OK] Hardware initialized
     - I2C internal pull-ups enabled
[OK] UART initialized
[OK] I2C slave initialized at 0x10
     - Max packet size: 255 bytes

[INFO] Core assignment:
     - Core 0: UART RX, validation, buffering
     - Core 1: I2C handling, filtering, caching

[READY] System operational

[STATUS] Total: 1523, Valid: 1520, Filtered: 0, Buffer: 256/8192
[POSITION] Lat: 55.751244, Lon: 37.618423, Sats: 12, Fix: 1
[CORES] C0: 152301, C1: 152298
```

## ⚡ Производительность

- Обработка до 50 NMEA сообщений в секунду
- Задержка I2C < 1мс
- Нулевая потеря данных при нормальной нагрузке
- Эффективное использование обоих ядер RP2350

## 🐛 Известные проблемы и решения

| Проблема | Решение |
|----------|---------|
| Нет данных от GPS | Проверьте подключение TX GPS → GPIO1 |
| I2C не отвечает | Убедитесь в правильном адресе 0x10 |
| Переполнение буфера | Читайте данные чаще или увеличьте буфер |
| Позиция не обновляется | Проверьте, что GPS имеет фиксацию |

## 📝 Лицензия

MIT License - используйте свободно в любых проектах!

## 🤝 Вклад в проект

Принимаются Pull Request'ы! 

1. Fork репозитория
2. Создайте ветку (`git checkout -b feature/amazing`)
3. Commit изменения (`git commit -m 'Add amazing feature'`)
4. Push в ветку (`git push origin feature/amazing`)
5. Откройте Pull Request

## 📞 Поддержка

- 🐛 [Сообщить об ошибке](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/issues/new?labels=bug)
- 💡 [Предложить улучшение](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/issues/new?labels=enhancement)
- 💬 [Обсуждения](https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350/discussions)

---

Сделано с ❤️ для сообщества