# Генератор карты препятствий и карты описания

Генерирует карту препятствий размера 320х320 пикселей и xml файл - описание объектов.
Положение объектов берётся из gazebo world файла.
Размеры объектов извлекаются из их описания в model.sdf файла (collision->geometry->box->size). 
Это самый предпочтительный вариант получения размеров объектов для карты препятствий.
Если размер в виде box не указан, по размеры берутся из файла самой модели (dae).

## Установка зависимостей

```
pip3 install --user pycollada
```

## Ограничения
Выгрузка размера модели корректно работает, только если объявлена лишь одна секция collision на весь объект.

## Быстрый запуск 
```bash
roscd arctic_model_gazebo
cd ../obstMapCreator
./scripts/gen_map.py --gzworld `rospack find arctic_model_gazebo`/worlds/arctic3.world --models_folder ~/arctic_ws/src/arctic_landscape/models
```
Команда создаст/обновит в текущей директории (obstMapCreator) файлы карт:  препятствий (config/map_obstacles.bmp) и описания (config/map_description.xml)

### Параметры запуска
Для знакомства с параметрами можно воспользоваться параметром --help при запуске скрипта:
```bash
./scripts/gen_map.py --help
```
param | default | description
--- | --- | ---
gzworld | - | gazebo world файл
static_map | config/arcticBasic.png | карта с нанесёнными препятствиями ландшафта и границей карты (готовится пользователями)
map_config | config/map_config.yaml | yaml файл, с конфигурацией маппинга имён моделей
models_folder | - | папка с dae файлами моделей
real_size | False | Карта препятствий не сжимается, а остаётся размера файла, указанного в параметре static_map 
start_id | 1 | начальный ID для объектов в файле-описании
map_description | config/map_description.xml | генерируемый файл описания карт
map_obstacles | config/map_obstacles.bmp | генерируемый файл карты препятствий

## Описание файлов
___config/arcticBasic.png___
    карта препятствий, размечаемых вручную. 
    К таким объектам относится всё то, что не дбавляется через world файл, как то горы, ущелья, склоны, провалы, пропасти, итд. Граница карты также на ней размечается

___config/map_config.yaml___
    конфигурация маппинга имён моделей
```yaml
# Пример 
man_lay: 'human'
house: 'house'
```
