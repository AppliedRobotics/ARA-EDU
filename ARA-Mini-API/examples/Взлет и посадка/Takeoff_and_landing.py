from ara_api.api_core import ARALinkManager
import time

manager = ARALinkManager() # создаём объект для управления квадрокоптером

def main():
    manager.takeoff(1.5)  # взлетаем на высоту 1.5 метра
    time.sleep(5)  # ждём 5 секунд
    manager.land()  # приземляемся

if __name__ == "__main__":
    main()
