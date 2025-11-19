from machine import Pin
import time

LED_PIN = 32

led = Pin(LED_PIN, Pin.OUT)

while True:
    led.value(1)
    print("Led ON!")
    time.sleep(1)
    led.value(0)
    print("Led OFF!")
    time.sleep(1)
