# 🧹 Очистка структуры проекта

Этот файл содержит инструкции по удалению старых файлов и папок для завершения реорганизации проекта.

## Файлы и папки для удаления

### Старые версии кода (удалить полностью):
- `nmea_converter_portapack_release/`
- `nmea_converter_portapack_v1_1/`
- `nmea_converter_rp2350_working/`
- `nmea_converter_rp2350_zero/`
- `nmea_converter_v3_final/`
- `nmea_generator_rp2040_tiny/`
- `test/`

### Старые workflow файлы:
- `.github/workflows/build-all-tests.yml`
- `.github/workflows/build-i2c-tests.yml`
- `.github/workflows/build-release.yml`
- `.github/workflows/build-test-generator-fixed.yml`
- `.github/workflows/build-test-generator.yml`
- `.github/workflows/build-v3.yml`
- `.github/workflows/build-working.yml`

### Прочие файлы:
- `trigger-build.txt`

## Команды для локального удаления

```bash
# Клонируйте репозиторий
git clone https://github.com/danusha2345/nmea-uart-i2c-converter-rp2350.git
cd nmea-uart-i2c-converter-rp2350

# Удалите старые папки
rm -rf nmea_converter_portapack_release/
rm -rf nmea_converter_portapack_v1_1/
rm -rf nmea_converter_rp2350_working/
rm -rf nmea_converter_rp2350_zero/
rm -rf nmea_converter_v3_final/
rm -rf nmea_generator_rp2040_tiny/
rm -rf test/

# Удалите старые workflow файлы
rm -f .github/workflows/build-all-tests.yml
rm -f .github/workflows/build-i2c-tests.yml
rm -f .github/workflows/build-release.yml
rm -f .github/workflows/build-test-generator-fixed.yml
rm -f .github/workflows/build-test-generator.yml
rm -f .github/workflows/build-v3.yml
rm -f .github/workflows/build-working.yml

# Удалите прочие файлы
rm -f trigger-build.txt

# Закоммитьте изменения
git add -A
git commit -m "Clean up old project structure"
git push origin main
```

## Финальная структура проекта

После очистки структура должна выглядеть так:

```
nmea-uart-i2c-converter-rp2350/
├── .github/
│   └── workflows/
│       └── build.yml
├── src/
│   └── nmea_converter.ino
├── .gitignore
├── CHANGELOG.md
├── LICENSE
└── README.md
```

## После очистки

После выполнения очистки:
1. Удалите этот файл (CLEANUP.md)
2. Создайте новый релиз v3.2.0
3. Проверьте, что GitHub Actions успешно собирает прошивку