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

<h1 style="text-align: center;">Приложение для обновления ESC</h1>

### Как выбрать приложение?

Если вы используете операционную систему **Windows**, необходимо скачать приложение из папки "windows"

Если вы используете операционную систему на базе ядра **Linux**, такую как Ubuntu, Astra, Red OS, Alt и другие, необходимо проделать следующие шаги:
- откройте терминал и введите команду ```uname -i```
- если указанная выше команда выводит "i686" в консоль, необходимо скачать приложение из папки "linux32"
- если указанная выше команда выводит "x86_64" в консоль, необходимо скачать приложение из папки "linux64"

### Решение проблемы в Linux  

Если при запуске приложения под **Linux** возникает ошибка ``` cannot open shared object file: No such file or directory ```  
необходимо установить следующие библиотеки:  

- **Для Ubuntu, Astra, AltOS**:  
  ```sh
  sudo apt install libxcrypt-compat libnsl
  ```  
- **Для RED OS, AlterOS**:  
  ```sh
  sudo yum install libxcrypt-compat libnsl
  ```

Если при запуске приложения под **Linux** возникает ошибка ``` bash ./update_esc_linux*.run: отказано в доступе ```  
необходимо сделать следующие:
  ```sh
  sudo chmod +x ./update_esc*
  ```  

