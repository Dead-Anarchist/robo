# Installation

Инструкция по установке (для версии Ubuntu/Kubuntu 18.04 / 20.04, ROS Melodic / ROS Noetic):

1. Скачать separated_arctic_install_2020_10.tar.xz (название может отличаться датой)
2. Создать директорию для catkin workspace в удобном месте (далее будет предполагаться, что это ~/arctic_standalone_ws/) и создать в ней src
3. Разархивировать скачанный архив в ~/arctic_standalone_ws/src/ (в ней появятся директории пакетов)
4. Установить ROS по официальной инструкции http://wiki.ros.org/noetic/Installation/Ubuntu
5. Установить зависимости пакетов (TODO - инструкция обновится когда будет найден более удобный способ, этот может установить не всё) следующим образом:
    1. Из корня воркспейса (~/arctic_standalone_ws) вызвать команду:

        `
        rosdep install --from-paths src --ignore-src -r -y
        `

        Она установит все объявленные в пакетах зависимости, которые в системе не установлены. 

    2. Установить дополнительные зависимости вызовом скрипта ~/arctic_standalone_ws/src/arctic_robot/arctic_config/scripts/extra_install.bash

6. Установить catkin tools для удобства сборки (это не встроенные в ROS catkin_make инструменты, а их Python альтернатива)

    `
    sudo pip install -U catkin_tools
    `
7. Инициализировать и собрать полученный catkin workspace:

    ```
    source /opt/ros/noetic/setup.bash
    cd ~/arctic_standalone_ws
    catkin build
    ```
8. Рекомендуется добавить в ~/.bashrc в конец строчку для автоматической инициализации при запуске консоли

    `
    source ~/arctic_standalone_ws/devel/setup.bash
    `

9. Добавить в ~/.bashrc после этого строчку настройки системы под ROS master, доступный для удалённого подключения (иначе распределённая ROS конфигурация не запустится), IP заменить на соответствующий данного компьютера:

    ```
    source `rospack find canonical_utilities`/scripts/connect_as_ros_master.bash 198.51.100.2
    ```
    ИЛИ

    ```
    source `rospack find canonical_utilities`/scripts/connect_as_ros_master_name.bash `hostname`.local
    ```
    если правильно настроена сеть для поддержки сетевых имён (на Linux как правило для этого работает avahi-daemon, может потребоваться настройка некоторых конфигурационных файлов системы), причём настройка должна быть корректной на всех используемых компьютерах (клиентском, который будет подключаться удалённо, тоже).

    На удалённом клиентском компьютере повторяется вся установка, но настраивается ROS slave (IP прописывается первый master, второй - slave):

    ```
    source `rospack find canonical_utilities`/scripts/connect_as_ros_slave.bash 198.51.100.2 198.51.100.3
    ```
    ИЛИ

    ```
    source `rospack find canonical_utilities`/scripts/connect_as_ros_slave_name.bash <master name>.local `hostname`.local
    ```

10. Добавить в ~/.bashrc строчку

    `
    export ROBOT=default_robot
    `

11. Установить Soar согласно инструкции в разделе "Soar installation".


## Soar installation

Здесь описывается процедура установки Soar с поддержкой Python на ROS Melodic и ROS Noetic.

1. Установить JDK8

    Загрузить и установить JDK:

    ```
    sudo apt install openjdk-8-jdk
    ```

    Указать, какую версию JDK использовать: после исполнения команды

    ```
    sudo update-alternatives --config java
    ```

    выбрать версию 8 в интерактивном режиме или явно указать версию командой:

    ```
    sudo update-alternatives --set java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java
    ```

    Текущую версию можно проверить командой:

    ```
    java -version
    ```

    Необходимо, чтобы отобразилась версия 1.8.
    Также следует установить swig:

    ```
    sudo apt install swig
    ```

2. Установить библиотеку Soar

    Клонировать библиотеку из репозитория:

    ```
    git clone https://github.com/SoarGroup/Soar.git
    alias python=python3
    cd Soar
    ```

    Собрать интерфейс Soar с python. При использовании ROS melodic (и Python2) следует использовать команду:

    ```
    python scons/scons.py sml_python --python=/usr/bin/python2
    ```

    При использовании ROS noetic (и Python3):

    ```
    python scons/scons.py sml_python
    ```

    Переместить Soar в директорию /opt:

    ```
    cd ..
    sudo mv Soar /opt
    ```

3. Проверка сборки

    Необходимо перейти в директорию `src/signproc/signproc_soar/scripts` и запустить тестовый скрипт командой

    ```
    python mini_test.py
    ```

    В норме скрипт должен отработать без ошибок (в т.ч. сообщений об исключениях интерпретатора Python), выведя сообщения о начале и завершении работы:

    ```
    python3 mini_test.py
    Processing started... Processing successfully finished. 
    ```

4. Дополнительные зависимости

    Кроме самой библиотеки Soar, для работы модели необходима установка дополнительных пакетов Python:

    ```
    pip3 install --user simple_pid scipy shapely descartes ply rdflib
    ```


## Video
To export video, software [rqt_bag_exporter](https://gitlab.com/InstitutMaupertuis/rqt_bag_exporter/tree/melodic/)
is required, follow the instructions there to install it.

# Running

## Short instructions
```
roslaunch arctic_model_gazebo arctic_semiotic.launch paused:=True robot:=default_robot
roslaunch arctic_config trajectory_navigation.launch gazebo:=True 
```
Then unpause the simulation using the Gazebo client GUI.

To record data, use 

```
roslaunch arctic_config record_data.launch
```

To export data to CSV, use 

```
rosrun arctic_config export_to_csv.sh
```
and for video files, the software rqt_bag_exporter:

```
roslaunch rqt_bag_exporter gui.launch
```

## Long instructions

Для запуска необходимо, чтобы catkin workspace был активирован командой

```
source <путь к catkin ws>/devel/setup.bash
```

Это обычно происходит автоматически при запуске консоли, если `.bashrc` был настроен.

Чтобы запустить симуляцию с роботом, нужно вызвать команду:

```
roslaunch arctic_model_gazebo arctic_semiotic.launch 
```

Система запущена (появится окно Gazebo с визуализацией мира и окно RViz с визуализацией того, что видит робот). 
При этом можно использовать параметры `gazebo_gui` и `rviz_gui` для включения или отключения интерфейсов Gazebo и RViz при запуске системы. Для этого необходимо либо поменять их значения в начале файла `arctic_landscape/src/arctic_model/arctic_model_gazebo/arctic_semiotic.launch` или запустить, указав их напрямую:

```
roslaunch arctic_model_gazebo arctic_semiotic.launch gazebo_gui:=False rviz_gui:=True
```

Для запуска rviz на удалённом компьютере, после корректной настройки его как slave машины (см. инструкцию по установке), достаточно вызвать команду

```
roslaunch arctic_config rviz.launch
```

Дальше можно отправлять команды на соответствующий HTTP сервер (JSON или RDF, в зависимости от версии). Это можно делать и вручную с помощью команды из директории `<путь к catkin_ws>/src/signproc/ros_ws/src/semiotic_interface/tests` и соответствующего файла с RDF командой (или JSON, соответственно):

```
./rdf_send_request.py data/поезжай_к_дереву.rdf
./json_send_request.py data/поезжай_к_дереву.json
```

# Troubleshooting

При первом запуске после установки лучше запускать с Gazebo GUI (флаг `gazebo_gui:=True`), т.к. некоторые модели будут подгружаться из интернета. При этом в окне Gazebo будет виден сплеш скрин с логотипом, но пока модели не загрузятся, он не исчезнет. Когда они загрузятся (обычно минут 5-10), в основном окне прогрузится мир с роботом и прочими объектами.

Всегда стоит перепроверить, был ли пересобран воркспейс

```
catkin build
```
и не забыть или перезапустить консольное приложение, или в каждой вкладке обновить среду командой

```
source <путь к catkin ws>/devel/setup.bash
```

Если какие-то зависимости не установятся, то необходимо устанавливать их по мере обнаружения командами:
sudo apt install ros-$ROS_DISTRO-<название>-<пакета> (нижние подчеркивания заменяются на дефисы)
pip install <название пакета> (именно под Python2, такие зависимости должны быть редкими)

## Сбор информации для отладки
Отладку лучше собирать с помощью tee, чтобы были полные логи вывода в консоль:

```
команда 2>&1 | tee имя_лога.log
```

Например:

```
roslaunch arctic_model_gazebo arctic_semiotic.launch rviz_gui:=false 2>&1 | tee textlog.log
```

## TF фреймы робота не видны в RViz / Контроллеры камеры или колёс не работают
 Обычно возникает из-за слишком медленной прогрузки Gazebo. Стоит подождать минут 5-10 до полной прогрузки, после чего загрузить контроллеры вручную командой

  ```
 roslaunch arctic_model_gazebo spawn_robot_controllers.launch
 ```

## Ветки репозиториев

При проблеме с установкой, можно попробовать проверить, выбраны ли в скачанных репозиториях правильные ветки. В текущей конфигурации, должны быть выбраны следующие ветки:

- arctic_landscape: devel-melodic
- arctic_robot: feat/diff_drive
- common_canonical: feat/artic_separation
- signproc: feat/soar_noetic
