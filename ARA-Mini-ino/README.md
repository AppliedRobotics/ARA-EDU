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

<h1 style="text-align: center;">Программный код web-пульта для ARA-Mini</h1>
В данной папке представлен архив, содержащий библиотеки и программный код для самостоятельного
подключения и редактирования web-пульта для микроквадрокоптера ARA-Mini

- Скачайте архив
- Разархивируйте его в удобную для вас папку

### Подключение библиотек
В архиве содержаться библиотеки AsyncTCP_patched и ESPAsyncWebServer_patched
- Откройте Arduino IDE
- Выберите Sketch -> Include Library -> Add .ZIP Library...
- Выберите скачанную библиотеку
- Дождитесь установки, в командной строку появится вывод:
```
Library installed
```
### Открыть скетч
Файл с программным кодом ARA-mini-ESP32.ino содержится в папке ARA-Mini-ino\ARA-mini-ESP32
Файл откроется в среде Arduino IDE двойным нажатием.

Чтобы открыть файл вручную:
- Зайдите в среду разработки Arduino IDE
- Выберите File -> Open -> ARA-mini-ESP32.ino
