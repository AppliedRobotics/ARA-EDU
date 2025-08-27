from ara_api.api_core import ARALinkManager
import time

manager = ARALinkManager() # создаём объект для управления квадрокоптером

def main():
    manager.takeoff(1.5)  # взлетаем на высоту 1.5 метра
    time.sleep(5)  # ждём 5 секунд
    manager.move_by_point(1, 0) # движемся в точку (1,0)
    time.sleep(2) # ждём 2 секунды для стабилизации
    manager.move_by_point(1, 1) # движемся в точку (1,1)
    time.sleep(2) # ждём 2 секунды для стабилизации
    manager.move_by_point(0, 1) # движемся в точку (0,1)
    time.sleep(2) # ждём 2 секунды для стабилизации
    manager.move_by_point(0, 0) # движемся в точку (0,0)
    time.sleep(2) # ждём 2 секунды для стабилизации
    manager.land()  # приземляемся

if __name__ == "__main__":
    main()

