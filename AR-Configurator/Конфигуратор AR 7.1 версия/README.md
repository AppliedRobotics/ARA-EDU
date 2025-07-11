<p align="center">
  <img style="
           display: block; 
           margin-left: auto;
           margin-right: auto;
           width: 50%;"
    src="../../logo/logo_black.png#gh-light-mode-only" alt="ara_logo"/>
</p>

<p align="center">
  <img style="
           display: block; 
           margin-left: auto;
           margin-right: auto;
           width: 50%;
  }"
    src="../../logo/logo_white.png#gh-dark-mode-only" alt="ara_logo"/>
</p>

<h1 style="text-align: center;">AR Configurator 7.1</h1>

## [Ссылка на скачивание](https://disk.360.yandex.ru/d/G2jAsrzb1h759g)

## Совместимость с операционными системами
- Windows 10/11
- Astra Linux
- Ubuntu
- Alter Linux
- RedOS

## Установка Конфигуратора
### Windows
- Скачайте файл конфигуратора AR-Configurator_win64_7.1.0.exe;
- Выполните установку (активируется двойным кликом по приложению);
- На рабочем столе появится иконка приложения;
- Конфигуратор запускается через иконку на рабочем столе.

### Astra Linux, Ubuntu (Debian-системы)
- Скачайте файл AR-Configurator_linux_x64_7_1_0.deb;
- Откройте терминал в папке с конфигуратором;
- Выполните команду (после ввода команды может потребоваться ввод пароля от системы);
```
sudo dpkg -i AR-Configurator_linux_x64_7_1_0.deb
``` 
- После первой установки deb-пакета введите 
команды (после ввода команды может потребоваться ввод пароля от системы);
```
sudo usermod -a -G tty $USER
sudo usermod -a -G dialout $USER
reboot
```
- После перезагрузки конфигуратор готов к работе.
Конфигуратор запускается командой:
```
ar-configurator
```
Для удаления конфигуратора введите команду (после ввода 
команды может потребоваться ввод пароля от системы):
```
sudo dpkg -r ar-configurator
```

### Red OS, Alter OS (RPM-системы)
- Скачайте AR-Configurator_linux_x64_7_1_0.rpm;
- Откройте терминал в папке с конфигуратором;
- Выполните команду (после ввода команды может потребоваться ввод пароля от системы);
```
sudo rpm -i AR-Configurator_linux_x64_7_1_0.rpm
```
- После первой установки deb-пакета введите 
команды (после ввода команды может потребоваться ввод пароля от системы);
```
sudo usermod -a -G tty $USER
sudo usermod -a -G dialout $USER
reboot
```
- После перезагрузки конфигуратор готов к работе.
- Конфигуратор запускается командой:
```
ar-configurator
```
Для удаления конфигуратора введите команду (после ввода команды может потребоваться ввод пароля от системы):
```
sudo rpm -e ar-configurator
```

### Calculate Linux
- Скачайте AR-Configurator_linux_CALCULATE_x64_7_1_0.zip;
- Разархивируйте архив;
- В появившейся папке откройте терминал и выполните команду 
(это сделает скрипт установщика исполняемым, после ввода команды может потребоваться ввод пароля от системы):
```
sudo chmod +x install_calculate_linux.sh
```
- Выполните установку, введя команду
```
sudo ./install_calculate_linux.sh
```
- Перед первым запуском конфигуратора выполните команду 
(это сделает команду запуска исполняемой, после ввода команды 
может потребоваться ввод пароля от системы):
```
sudo chmod +x run_calculate_linux.sh
```
- Для запуска конфигуратора выполните скрипт 
```
sudo ./run_calculate_linux.sh
```
