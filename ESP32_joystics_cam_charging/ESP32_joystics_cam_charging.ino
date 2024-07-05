#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_camera.h"
#include "ARA_ESP.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char *ssid = "ARA-Mini_1"; //Имя точки доступа
const char *password = "12345678";  //Пароль от точки доступа

AsyncWebServer server(8775);  //порт подключения
AsyncWebSocket ws("/ws");     // Вебсокет

IPAddress local_ip(192, 168, 2, 113);  // Локальный IP-адрес
IPAddress gateway(192, 168, 2, 113);   // IP-адрес шлюза
IPAddress subnet(255, 255, 255, 0);    // Маска подсети

void startCameraServer(); //функция для запуска камера-сервера
void setupLedFlash(int pin); // функция для настройки мигания светодиода

//глобальные переменные для хранения значений крена, тангажа, рысканья и дросселя, которые будут получены через веб-сокет.
int Arm = 0;
int Flight_state = 0;
int Nav_state = 0;
float RollValue = 0;
float PitchValue = 0;
float YawValue = 0;
float ThrottleValue = 0;
int Programming = 0;

//Эта функция разделяет входную строку на массив значений с плавающей запятой с использованием указанного разделителя.
//Разделенные значения преобразуются в числа с плавающей запятой и сохраняются в предоставленном массиве.
void splitAndConvertToFloat(const String &inputString, float array[], int arraySize, char delimiter) {
  char temp[inputString.length() + 1];
  inputString.toCharArray(temp, inputString.length() + 1);

  char *token = strtok(temp, ";");
  int count = 0;

  while (token != NULL && count < arraySize) {
    array[count] = atof(token);
    token = strtok(NULL, ";");
    count++;
  }
}
//Эта функция обрабатывает входящие данные веб-сокета. Она извлекает данные в виде строки и разделяет их на массив значений с плавающей запятой с использованием точки с запятой в качестве разделителя.
//Затем разделенные значения присваиваются соответствующим глобальным переменным YawValue, ThrottleValue, RollValue и PitchValue.
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    data[len] = 0;  // Убедитесь, что строка завершается нулем

    String inputString = String((char *)data);
    float values[8];

    splitAndConvertToFloat(inputString, values, 8, ';');

    Arm = values[0];
    Nav_state = values[1];
    YawValue = values[2];
    ThrottleValue = values[3];
    RollValue = values[4];
    PitchValue = values[5];
    Flight_state = values[6];
    Programming = values[7];
  }
}

const char *html1 = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ARA-mini</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        html, body {
            height: 100dvh;
            width: 100dvw;
        }

        body {
            -webkit-user-select: none;
            -moz-user-select: none;
            -ms-user-select: none;
            user-select: none;
            touch-action: none;
            -ms-content-zooming: none;
            display: flex;
            flex-direction: column;
            min-height: 100dvh;
            align-items: center;
            justify-content: center;
            gap: 10px;
            font-family: Calibri, sans-serif;
        }

        #video-container-portrait, #video-container-landscape {
            border: 1px solid black;
            width: 45dvh;
            height: 45dvh;
            z-index: 1;
            text-align: center;
        }

        #control-panel {
            margin: 0 auto;
            padding: 20px;
            border: 1px solid #ccc;
            border-radius: 20px;
            background-color: #f9f9f9;
        }

        .joystick {
            width: 250px;
            height: 250px;
            background-color: #ddd;
            border-radius: 50%;
            display: inline-block;
            position: relative;
            overflow: hidden;
        }

        .joystick-handle {
            width: 50px;
            height: 50px;
            background-color: #4976c4;
            border-radius: 50%;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }

        .toggle-switch {
            height: 30px;
            width: 30px;
            appearance: none;
            background-color: #4976c4;
            border-radius: 50%;
            cursor: pointer;
            opacity: 0;
            margin: 0;
        }

        .tri-state-toggle {
            width: fit-content;
            display: flex;
            justify-content: center;
            border: 3px solid #ccc;
            border-radius: 50px;
        }

        .joystick-container {
            display: flex;
            gap: 10px;
            flex-direction: row;
            justify-content: space-evenly;
        }

        .toggles-container {
            margin-top: 20px;
            display: flex;
            flex-direction: row;
            justify-content: space-evenly;
        }

        .state-button {
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 5px;
            text-align: center;
            border: 3px solid #afaeae;
            border-radius: 10pt;
            font-size: 14pt;
            font-family: Calibri, sans-serif;
            cursor: pointer;
            flex: 1
        }

        .toggle-switch {
            height: 30px;
            width: 30px;
            appearance: none;
            background-color: #4976c4;
            border-radius: 50%;
            cursor: pointer;
            opacity: 0;
            margin: 0;
        }

        .tri-state-toggle {
            width: fit-content;
            display: flex;
            justify-content: center;
            border: 3px solid #afaeae;
            border-radius: 50px;
        }

        svg {
            height: 8dvw;
            position: absolute;
            top: 2%;
            left: 2%;
            width: 13.6dvw;
            max-width: 170px;
            max-height: 100px;
        }

        .toggles {
            width: 250px;
        }

        @media only screen and (max-width: 800px) {
            .joystick {
                width: 150px;
                height: 150px;
                border-radius: 50%;
            }

            .joystick-handle {
                width: 25px;
                height: 25px;
            }

            .toggles {
                width: 150px;
            }

            .toggle-switch {
                width: 20px;
                height: 20px;
            }

            .right-toggle {
                margin-left: 20px;
            }
        }

        @media (orientation: landscape) {
            body {
                background-color: #f9f9f9;
            }

            #video-container-portrait {
                display: none;
            }

            #video-container-landscape {
                flex: 1;
            }

            #control-panel{
                border: none;
                width: 100%;
                height: 100%;
                margin-top: 12dvh;
                display: flex;
                flex-direction: column;
                justify-content: space-evenly;
                background: none;
            }

            .joystick-container {
                align-items: end;
            }

            .toggles:last-child {
                display: none;
            }
        }
        @media (orientation: portrait) {
            #video-container-landscape {
                display: none;
            }
        }
    </style>
</head>
<body>
<svg xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
     xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd" xmlns="http://www.w3.org/2000/svg"
     xmlns:svg="http://www.w3.org/2000/svg" version="1.1" id="svg2" width="624.76001" height="367.76001"
     viewBox="0 0 624.76001 367.76001" sodipodi:docname="logo_AR_blue.eps">
    <defs id="defs6"/>
    <sodipodi:namedview id="namedview4" pagecolor="#ffffff" bordercolor="#000000" borderopacity="0.25"
                        inkscape:showpageshadow="2" inkscape:pageopacity="0.0" inkscape:pagecheckerboard="0"
                        inkscape:deskcolor="#d1d1d1"/>
    <g id="g8" inkscape:groupmode="layer" inkscape:label="ink_ext_XXXXXX"
       transform="matrix(1.3333333,0,0,-1.3333333,0,367.76)">
        <g id="g10" transform="scale(0.1)">
            <path d="m 391.063,1582.17 v 455.14 L 0,2263.09 v -906.7 l 391.063,225.78"
                  style="fill:#4879c6;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path12"/>
            <path d="m 821.863,2758.19 v -450.24 l 387.777,-223.62 389.95,225.14 -777.727,448.72"
                  style="fill:#4879c6;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path14"/>
            <path d="m 393.555,2084.42 386.261,222.74 v 449.88 L 3.91797,2309.38 393.555,2084.42"
                  style="fill:#4879c6;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path16"/>
            <path d="M 779.816,1310.23 393.375,1534.96 10.0156,1313.63 779.816,865.891 v 444.339"
                  style="fill:#4879c6;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path18"/>
            <path d="m 1211.97,2037.13 v -454.77 l 391.7,-226.15 v 907.07 l -391.7,-226.15"
                  style="fill:#4879c6;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path20"/>
            <path d="m 2032.73,1582.17 v 455.14 l -391.06,225.78 v -906.7 l 391.06,225.78"
                  style="fill:#4879c6;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path22"/>
            <path d="m 2463.53,2758.19 v -450.24 l 387.78,-223.62 389.95,225.14 -777.73,448.72"
                  style="fill:#4879c6;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path24"/>
            <path d="m 2035.22,2084.42 386.26,222.74 v 449.88 l -775.9,-447.66 389.64,-224.96"
                  style="fill:#4879c6;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path26"/>
            <path d="m 1540.53,746.512 -95.05,189.84 h -13.29 l -95.05,-189.84 z m -61.33,233.226 183.97,-362.277 h -58.76 l -41.4,80.398 h -249.38 l -40.87,-80.398 h -58.77 l 183.96,362.277 h 81.25"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path28"/>
            <path d="m 1755.15,797.27 h 230.97 c 14.99,0 26.92,5.339 35.78,16 8.85,10.64 13.28,27.539 13.28,50.628 0,23.122 -4.34,40.071 -13.03,50.922 -8.69,10.84 -20.87,16.27 -36.54,16.27 h -230.46 z m -52.13,-179.809 v 362.277 h 285.15 c 30.99,0 55.26,-9.578 72.82,-28.707 17.54,-19.133 26.32,-48.152 26.32,-87.133 0,-38.968 -8.95,-67.918 -26.83,-86.867 -17.89,-18.941 -42.67,-28.41 -74.36,-28.41 h -230.97 v -131.16 h -52.13"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path30"/>
            <path d="m 2200.76,797.27 h 230.97 c 15,0 26.92,5.339 35.78,16 8.85,10.64 13.28,27.539 13.28,50.628 0,23.122 -4.35,40.071 -13.03,50.922 -8.69,10.84 -20.87,16.27 -36.54,16.27 h -230.46 z m -52.13,-179.809 v 362.277 h 285.15 c 30.99,0 55.27,-9.578 72.82,-28.707 17.54,-19.133 26.32,-48.152 26.32,-87.133 0,-38.968 -8.95,-67.918 -26.84,-86.867 -17.88,-18.941 -42.67,-28.41 -74.35,-28.41 h -230.97 v -131.16 h -52.13"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path32"/>
            <path d="m 2591.16,979.738 h 52.13 V 666.109 h 267.76 v -48.648 h -319.89 v 362.277"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path34"/>
            <path d="m 3058.32,617.461 h -52.13 v 362.277 h 52.13 V 617.461"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path36"/>
            <path d="m 3151.33,979.738 h 334.72 V 931.09 H 3203.46 V 825.301 h 272.87 v -48.66 H 3203.46 V 666.109 h 282.59 v -48.648 h -334.72 v 362.277"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path38"/>
            <path d="m 3607.99,666.109 h 240.7 c 28.27,0 47.28,8.922 56.97,26.75 9.73,17.84 14.55,53.09 14.55,105.731 0,52.301 -4.95,87.43 -14.8,105.469 -9.9,18.011 -28.78,27.031 -56.72,27.031 h -240.7 z m -52.11,313.629 h 293.31 c 43.29,0 72.73,-8.308 88.4,-24.898 23.17,-24.731 34.77,-76.801 34.77,-156.25 0,-79.09 -11.6,-131 -34.77,-155.719 -15.67,-16.941 -45.11,-25.41 -88.4,-25.41 h -293.31 v 362.277"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path40"/>
            <path d="m 1290.64,185.109 h 240.28 c 19.04,0 32.98,3.36 41.83,10.09 13.27,10.27 19.9,29.18 19.9,56.821 0,27.609 -6.47,46.359 -19.39,56.269 -9.18,7.082 -23.3,10.633 -42.34,10.633 h -240.28 z m -52.12,182.469 h 313.76 c 29.64,0 52.46,-10.519 68.47,-31.527 16.01,-21 24.03,-48.969 24.03,-83.922 0,-30 -4.61,-52.41 -13.81,-67.231 -6.47,-10.589 -16.86,-19.066 -31.17,-25.418 12.6,-5.308 22.49,-12.199 29.65,-20.671 10.21,-12.02 15.33,-26.84 15.33,-44.5199 V 5.28906 h -52.13 V 94.5117 c 0,12.0273 -5.53,22.0193 -16.58,30.0003 -11.05,7.968 -24.4,11.937 -40.05,11.937 H 1290.64 V 5.28906 h -52.12 V 367.578"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path42"/>
            <path d="m 1924.63,48.6602 c 80.38,0 127.75,6.1718 142.06,18.539 14.3,12.3594 21.46,51.9298 21.46,118.7108 0,67.11 -7.07,106.84 -21.21,119.211 -14.15,12.359 -61.57,18.559 -142.31,18.559 -80.76,0 -128.19,-6.2 -142.32,-18.559 -14.14,-12.371 -21.21,-52.101 -21.21,-119.211 0,-66.43 7.15,-105.8983 21.46,-118.4491 14.31,-12.539 61.66,-18.8007 142.07,-18.8007 z M 1924.63,0 c -72.57,0 -123.34,4.85938 -152.29,14.5508 -28.96,9.7109 -47.36,28.0976 -55.19,55.2578 -5.46,18.6719 -8.18,57.3124 -8.18,115.8204 0,59.59 2.56,98.199 7.67,115.84 7.83,27.5 26.4,46.179 55.7,56.062 29.29,9.86 80.05,14.801 152.29,14.801 72.55,0 123.23,-4.863 152.02,-14.543 28.78,-9.711 47.27,-28.301 55.44,-55.801 5.45,-18.679 8.19,-57.476 8.19,-116.359 0,-59.231 -2.56,-97.6485 -7.68,-115.2774 -7.84,-27.5 -26.23,-46.0899 -55.19,-55.8008 C 2048.45,4.85938 1997.52,0 1924.63,0"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path44"/>
            <path d="M 2264.97,167.129 V 53.9492 h 227.92 c 19.75,0 34.66,5.211 44.71,15.6016 10.04,10.3984 15.07,24.0586 15.07,40.9882 0,16.582 -5.1,30.133 -15.33,40.723 -10.21,10.558 -25.03,15.867 -44.45,15.867 z m 274.93,97.562 c 0,16.649 -4.08,29.418 -12.24,38.27 -9.87,10.621 -25.51,15.961 -46.93,15.961 H 2264.97 V 215.781 h 215.76 c 22.11,0 37.49,3.981 46.16,11.957 8.67,7.973 13.01,20.293 13.01,36.953 z m 64.89,-153.632 c 0,-35.6098 -9.44,-62.1293 -28.36,-79.5785 C 2557.54,14.0313 2529.85,5.28906 2493.4,5.28906 H 2212.85 V 367.578 h 268.28 c 35.77,0 63.18,-8.84 82.27,-26.539 19.09,-17.68 28.62,-43.141 28.62,-76.391 0,-36.41 -13.12,-59.757 -39.35,-70.019 14.99,-5.297 26.92,-14.109 35.77,-26.449 10.9,-15.16 16.35,-34.2 16.35,-57.121"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path46"/>
            <path d="m 2886.35,48.6602 c 80.39,0 127.76,6.1718 142.06,18.539 14.31,12.3594 21.46,51.9298 21.46,118.7108 0,67.11 -7.07,106.84 -21.2,119.211 -14.14,12.359 -61.57,18.559 -142.32,18.559 -80.75,0 -128.18,-6.2 -142.31,-18.559 -14.14,-12.371 -21.21,-52.101 -21.21,-119.211 0,-66.43 7.15,-105.8983 21.45,-118.4491 14.32,-12.539 61.66,-18.8007 142.07,-18.8007 z M 2886.35,0 c -72.56,0 -123.33,4.85938 -152.28,14.5508 -28.96,9.7109 -47.36,28.0976 -55.19,55.2578 -5.46,18.6719 -8.18,57.3124 -8.18,115.8204 0,59.59 2.56,98.199 7.68,115.84 7.82,27.5 26.39,46.179 55.69,56.062 29.3,9.86 80.05,14.801 152.28,14.801 72.56,0 123.24,-4.863 152.03,-14.543 28.78,-9.711 47.27,-28.301 55.44,-55.801 5.44,-18.679 8.18,-57.476 8.18,-116.359 0,-59.231 -2.55,-97.6485 -7.67,-115.2774 -7.84,-27.5 -26.23,-46.0899 -55.18,-55.8008 C 3010.18,4.85938 2959.24,0 2886.35,0"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path48"/>
            <path d="m 3158.1,367.578 h 385.38 V 318.922 H 3377.11 V 5.28906 h -52.12 V 318.922 H 3158.1 v 48.656"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path50"/>
            <path d="m 3667.88,5.28906 h -52.13 V 367.578 h 52.13 V 5.28906"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path52"/>
            <path d="m 3823.46,185.91 c 0,-66.07 7.26,-105.4686 21.78,-118.1717 14.47,-12.707 58.91,-19.0781 133.22,-19.0781 69.27,0 111.5,3.6992 126.72,11.1015 17.8,8.8203 26.69,32.6172 26.69,71.3983 h 55.45 c 0,-57.4803 -12.26,-93.6912 -36.68,-108.6717 C 4126.16,7.48828 4068.76,0 3978.46,0 3886.75,0 3828.73,10.582 3804.43,31.7383 3780.13,52.8984 3768,104.199 3768,185.629 c 0,81.801 12.05,133.359 36.17,154.711 24.12,21.32 82.18,31.992 174.29,31.992 90.65,0 148.15,-7.332 172.44,-21.973 24.24,-14.648 36.42,-50.871 36.42,-108.668 h -55.45 c 0,38.797 -8.19,62.149 -24.47,70.079 -16.34,7.921 -59.32,11.91 -128.94,11.91 -74.66,0 -119.19,-6.282 -133.48,-18.828 -14.35,-12.551 -21.52,-52.192 -21.52,-118.942"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path54"/>
            <path d="m 4481.97,207.859 c 85.86,0 139.9,-4.949 162.18,-14.828 27.66,-12.332 41.5,-41.941 41.5,-88.832 0,-44.4373 -16.71,-73.5193 -50.12,-87.2693 C 4608.24,5.62891 4552.92,0 4469.58,0 4379.4,0 4321.39,9.69922 4295.54,29.0898 4275.77,43.8984 4265.9,76.1602 4265.9,125.871 h 54.97 c 0,-34.1913 7.71,-55.6093 23.17,-64.2616 15.43,-8.6289 57.46,-12.9492 126.06,-12.9492 67.18,0 110.75,3.1796 130.68,9.5312 19.93,6.3477 29.91,21.668 29.91,46.0076 0,24.653 -7.46,40.012 -22.37,46 -14.91,5.992 -57.02,9.012 -126.35,9.012 -84.79,0 -141.7,10.051 -170.82,30.121 -22.28,15.168 -33.39,42.496 -33.39,81.996 0,42.641 13.37,70.231 40.15,82.762 26.74,12.5 81.42,18.762 164.06,18.762 82.26,0 136.33,-9.68 162.18,-29.071 20.48,-15.543 30.71,-45.84 30.71,-90.972 h -53.87 c 0,31.25 -8.08,50.972 -24.25,59.14 -16.18,8.153 -54.41,12.25 -114.77,12.25 -51.72,0 -83.52,-0.719 -95.37,-2.109 -35.92,-4.258 -53.9,-21.102 -53.9,-50.488 0,-24.793 9.72,-41.622 29.12,-50.461 19.38,-8.871 59.44,-13.282 120.15,-13.282"
                  style="fill:#0d0906;fill-opacity:1;fill-rule:nonzero;stroke:none" id="path56"/>
        </g>
    </g>
</svg>
<div id="video-container-portrait">
    <img id="stream-portrait" crossorigin style="width: 100%; height: 100%; z-index: 1" alt="Нет соединения">
</div>
<div id="control-panel">
    <div class="joystick-container">
        <div class="joystick" id="left-joystick">
            <div class="joystick-handle" id="left-handle"></div>
        </div>
        <div id="video-container-landscape">
            <img id="stream-landscape" crossorigin style="width: 100%; height: 100%; z-index: 1" alt="Нет соединения">
        </div>
        <div class="joystick" id="right-joystick">
            <div class="joystick-handle" id="right-handle"></div>
        </div>
    </div>
    <div>
        <div class="joystick-container">
            <div class="toggles">
                <div style="display: flex; align-items: center;">
                    <div style="position: relative; top: -10px; margin-right: 5px">
                        Режим навигации
                    </div>
                    <div>
                        <div class="toggles-container">
                            <div class="tri-state-toggle">
                                <input class="button toggle-switch" type="radio" name="toggleNavSwitch" data-id="0"/>
                                <input class="button toggle-switch" type="radio" name="toggleNavSwitch" data-id="1"/>
                                <input class="button toggle-switch" type="radio" name="toggleNavSwitch" data-id="2"/>
                            </div>
                        </div>
                        <div class="toggles-container"
                             style="width: 96px; padding-left: 10px; margin: 0 auto 20px; font-size: 9pt">
                            <div style="flex: 1"></div>
                            <div style="flex: 1">ALT</div>
                            <div style="flex: 1">POS</div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="toggles">
                <div style="display: flex; align-items: center; ">
                    <div>
                    </div>
                </div>
            </div>
        </div>
        <div style="display: flex; flex-direction: row; gap: 10px; justify-content: center">
            <div class="state-button" id="armButton">
                ARM
            </div>
        </div>
    </div>
</div>

<script>
    // CONFIG
    const host = document.location.hostname

    // JOYSTICKS
    const leftHandle = document.getElementById('left-handle');
    const rightHandle = document.getElementById('right-handle');
    const leftJoystick = document.getElementById('left-joystick');
    const rightJoystick = document.getElementById('right-joystick');

    let allReadings = [0, 1, 0, 0, 0, 0, 1, 0] // [armedButton, navState, LX, LY, RX, RY, flightState, programmingButton]

    function updateJoystickPosition(mouseX, mouseY, joystick, handle, shift, isDown) {
        const handleSize = handle.clientWidth;
        const joystickRect = joystick.getBoundingClientRect();

        const centerX = joystickRect.left + joystickRect.width / 2;
        const centerY = joystickRect.top + joystickRect.height / 2;
        const radius = joystickRect.width / 2 - handleSize / 2;

        let dx = mouseX - centerX;
        let dy = mouseY - centerY;
        let distance = Math.sqrt(dx * dx + dy * dy);
        let angle = Math.atan2(dy, dx);

        if (distance > radius) {
            dx = radius * Math.cos(angle);
            dy = radius * Math.sin(angle);
        }

        let currentX = centerX + dx - joystickRect.left - handleSize / 2;
        let currentY = centerY + dy - joystickRect.top - handleSize / 2;

        let actualReadingX = (dx / radius).toFixed(4);
        let actualReadingY;
        if (isDown) {
            actualReadingY = ((radius - dy) / (2 * radius)).toFixed(4);
        } else {
            actualReadingY = (-dy / radius).toFixed(4);
        }

        console.log(actualReadingX, actualReadingY);

        handle.style.left = `${currentX + handleSize / 2}px`;
        handle.style.top = `${currentY + handleSize / 2}px`;

        allReadings[shift] = parseFloat(actualReadingX);
        allReadings[shift + 1] = parseFloat(actualReadingY);
    }

    function hydrateJoystickLeft() {
        const shift = 2;
        leftHandle.style.left = "50%";
        leftHandle.style.top = `calc(100% - ${leftHandle.clientWidth / 2}px)`;

        leftJoystick.addEventListener('mousedown', function (event) {
            updateJoystickPosition(event.clientX, event.clientY, leftJoystick, leftHandle, shift, true);
            const handleMouseMove = function (event) {
                updateJoystickPosition(event.clientX, event.clientY, leftJoystick, leftHandle, shift, true);
            };
            document.addEventListener('mousemove', handleMouseMove);
            document.addEventListener('mouseup', function () {
                allReadings[shift] = 0;
                document.removeEventListener('mousemove', handleMouseMove);
                leftHandle.style.left = "50%";
            }, {once: true});
        });

        leftJoystick.addEventListener('touchstart', function (event) {
            var indexL = event.touches.length - 1;
            updateJoystickPosition(event.touches[indexL].clientX, event.touches[indexL].clientY, leftJoystick, leftHandle, shift, true);
            const handleMouseMove = function (event) {
                updateJoystickPosition(event.touches[indexL].clientX, event.touches[indexL].clientY, leftJoystick, leftHandle, shift, true);
            };
            leftJoystick.addEventListener('touchmove', handleMouseMove);
            leftJoystick.addEventListener('touchend', function () {
                allReadings[shift] = 0;
                leftHandle.style.left = "50%";
            }, {once: true});
        });
    }

    function hydrateJoystickRight() {
        const shift = 4;
        rightHandle.style.left = "50%";
        rightHandle.style.top = "50%";

        rightJoystick.addEventListener('mousedown', function (event) {
            updateJoystickPosition(event.clientX, event.clientY, rightJoystick, rightHandle, shift, false);
            const handleMouseMove = function (event) {
                updateJoystickPosition(event.clientX, event.clientY, rightJoystick, rightHandle, shift, false);
            };
            document.addEventListener('mousemove', handleMouseMove);
            document.addEventListener('mouseup', function () {
                allReadings[shift] = 0
                allReadings[shift + 1] = 0
                document.removeEventListener('mousemove', handleMouseMove);
                rightHandle.style.left = "50%";
                rightHandle.style.top = "50%";
            }, {once: true});
        });

        rightJoystick.addEventListener('touchstart', function (event) {
            var indexR = event.touches.length - 1;
            updateJoystickPosition(event.touches[indexR].clientX, event.touches[indexR].clientY, rightJoystick, rightHandle, shift, false);
            const handleMouseMove = function (event) {
                updateJoystickPosition(event.touches[indexR].clientX, event.touches[indexR].clientY, rightJoystick, rightHandle, shift, false);
            };
            rightJoystick.addEventListener('touchmove', handleMouseMove);
            rightJoystick.addEventListener('touchend', function () {
                allReadings[shift] = 0
                allReadings[shift + 1] = 0
                rightHandle.style.left = "50%";
                rightHandle.style.top = "50%";
            }, {once: true});
        });
    }

    hydrateJoystickLeft()
    hydrateJoystickRight()
    window.addEventListener('scroll-fix', function (e) {
        e.preventDefault();
    });

    //TOGGLES
    const toggleNavSwitchStates = document.querySelectorAll('[name=toggleNavSwitch]')
    toggleNavSwitchStates.forEach((el) => {
        if (+el.dataset.id === allReadings[1]) el.style.opacity = "1"
        el.addEventListener("click", (ev) => {
            allReadings[1] = +el.dataset.id
            toggleNavSwitchStates.forEach((elem) => {
                elem.style.opacity = "0"
            })
            el.style.opacity = "1"
        })
    })


    // ARM
    const armButton = document.querySelector('#armButton')
    armButton.addEventListener('click', (ev) => {
        if (allReadings[0]) {
            ev.target.innerText = 'ARM'
            ev.target.style.background = 'white'
            ev.target.style.color = 'black'
            allReadings[0] = 0
            allReadings[1] = 1
            toggleNavSwitchStates.forEach((el) => {
                el.style.opacity = "0"
                if (+el.dataset.id === allReadings[1]) el.style.opacity = "1"
            })
        } else {
            ev.target.innerText = 'DISARM'
            ev.target.style.background = '#4976c4'
            ev.target.style.color = 'white'
            allReadings[0] = 1
            allReadings[1] = 2
            toggleNavSwitchStates.forEach((el) => {
                el.style.opacity = "0"
                if (+el.dataset.id === allReadings[1]) el.style.opacity = "1"
            })
        }
    })

    // NETWORK
    websocket_url = `ws://${host}:8775/ws`;
    sendTimeout = 100;
    const websocket = new WebSocket(websocket_url);

    const sendLoop = setInterval(() => {
        if (websocket.readyState === websocket.OPEN)
            websocket.send(allReadings.join(';'));
        else {
            console.log(websocket.readyState);
            console.log("Socket not working!!!");
            clearInterval(sendLoop);
        }
    }, sendTimeout)

    // STREAM
    const streamSrc = `http://${host}:81/stream`
    const streamLandscape = document.querySelector('#stream-landscape')
    const streamPortrait = document.querySelector('#stream-portrait')
    let currentOrientation = null

    window.onresize = function (event) {
        applyOrientation();
    }

    function applyOrientation() {
        if (window.innerHeight > window.innerWidth) {
            if (currentOrientation === "portrait") {
                return
            }
            currentOrientation = "portrait"
            streamLandscape.src = ""
            streamPortrait.src = streamSrc
        } else {
            if (currentOrientation === "landscape") {
                return
            }
            currentOrientation = "landscape"
            streamPortrait.src = ""
            streamLandscape.src = streamSrc
        }
    }
    applyOrientation()
</script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  
  camera_config_t config;// Настройка конфигурации камеры
  pinMode(XCLK_GPIO_NUM, OUTPUT);// Установка пина XCLK (тактовый сигнал) в режим вывода
  digitalWrite(XCLK_GPIO_NUM, 0);// Установка пина XCLK в низкий уровень (0)

  // Настройка конфигурации камеры
  config.ledc_channel = LEDC_CHANNEL_0;// Канал ШИМ
  config.ledc_timer = LEDC_TIMER_0;// Таймер ШИМ
  config.pin_d0 = Y2_GPIO_NUM; // Пины данных (D0-D7)
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; // Пины управления камерой
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;// Пины питания и сброса
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;// Частота тактового сигнала
  config.frame_size = FRAMESIZE_UXGA; // Разрешение кадра
  config.pixel_format = PIXFORMAT_JPEG;// Формат пикселей (JPEG для потоковой передачи)
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;// Режим захвата
  config.fb_location = CAMERA_FB_IN_PSRAM;// Расположение буфера кадра
  config.jpeg_quality = 12;// Качество JPEG
  config.fb_count = 1;// Количество буферов кадров

   // Настройка параметров камеры в зависимости от наличия PSRAM
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      // Если есть PSRAM, используем более высокое качество JPEG и больше буферов кадров
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Если PSRAM нет, ограничиваем разрешение и используем DRAM
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
   }

  // Инициализация камеры с заданными параметрами
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Вывод ошибки в последовательный порт, если инициализация камеры не удалась
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Получение объекта сенсора камеры
  sensor_t *s = esp_camera_sensor_get();
  // Настройка сенсора (отражение по вертикали, яркость, насыщенность) 
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // Изменение размера кадра для повышения частоты кадров
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

// Дополнительная настройка для конкретных моделей камер
#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);  // Создать точку доступа с именем ssid и паролем password
  WiFi.softAPConfig(local_ip, gateway, subnet);  // Настроить конфигурацию точки доступа (локальный IP, шлюз, маска подсети)

  Serial.print("AP IP Address: ");  // Вывести IP-адрес точки доступа в последовательный порт
  Serial.println(WiFi.softAPIP());

  startCameraServer();

  ws.onEvent(onWsEvent);   // Настроить веб-сокет
  server.addHandler(&ws);  // Добавить обработчик веб-сокета на сервер

  // Обработчик запросов GET по адресу "/"
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", html1);
  });

  server.begin();     // Запустить сервер
  esp.begin(Serial);  // Запустить сервис по отправки данных в полетный контроллер
}


void loop() {
  esp.arm(Arm);// Установить арм
  delay(20);

  esp.nav_mode(Nav_state);// Установить навигационный режим
  delay(20);//50

  esp.flight_mode(Flight_state);
  delay(20);

  esp.roll(RollValue);// Установить крен
  delay(20);//50

  esp.pitch(PitchValue);// Установить тангаж
  delay(20);

  esp.yaw(YawValue);// Установить рысканье
  delay(20);

  esp.throttle(ThrottleValue);// Установить дроссель
  delay(20);
    
  
    
  }
