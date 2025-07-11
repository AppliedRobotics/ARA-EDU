<p align="center">
  <img style="
           display: block; 
           margin-left: auto;
           margin-right: auto;
           width: 50%;"
    src="../logo/logo_black.png#gh-light-mode-only" alt="ara_logo"/>
</p>

<p align="center">
  <img style="
           display: block; 
           margin-left: auto;
           margin-right: auto;
           width: 50%;
  }"
    src="../logo/logo_white.png#gh-dark-mode-only" alt="ara_logo"/>
</p>

<h1 style="text-align: center;">Обновление программируемого контроллера ESP на микроквадрокоптере ARA-Mini</h1>

### Скачайте 5 файлов, содержащихся в папке
Поместите данные файлы в одну удобную вам директорию
### Прошивка с ESPTool
- Перейдите на сайт https://espressif.github.io/esptool-js/
- Подключите провод microUSB в разъем для подключения к программируемому контроллеру
- Подключите батарею, убедитесь что красный светодиод над кнопкой питания загорается
- В программе ESP Tool выберите "Baudrate" = 115200, нажмите "Connect"
- Выберите порт, к которому подключен дрон и нажмите "Подключение"
- Нажмите на кнопку "Add File" 4 раза, так чтобы получилось 5 строк
- Введите адреса 0x1000, 0x8000, 0xe000, 0x10000, 0x310000 согласно таблице
- Добавьте файлы согласно таблице

| Flash Address | File           |
|---------------|----------------|
| 0x1000        | bootloader.bin |
| 0x8000        | partitions.bin |
| 0xe000        | boot_app0.bin  |
| 0x10000       | firmware.bin   |
| 0x310000      | spiffs.bin     |
- Нажмите "Program"
- После завершения всей загрузки отключите батарею, отсоедините провод
### Обучающие материалы
Обновление полетного и программируемого контроллеров микроквадрокоптера ARA MINI
- RUTUBE: https://rutube.ru/video/c854d824d0bcc2b605aa37a8a76e1d6d/
- YOUTUBE: https://youtu.be/3tQuwXXxphg