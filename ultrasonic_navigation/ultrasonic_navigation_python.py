import cv2
import numpy as np
import socket
from collections import deque
import asyncio
import aiohttp

# Настройки для TCP порта
tcp_port = 80
points_count = 50
packet_window = 100  # Окно пакетов

# Параметры для графиков
max_x = 2500  # Максимальное значение по оси X
max_y = 3000  # Максимальное значение по оси Y
max_distance = 6000  # Максимальное расстояние для графика расстояний
img_size = 600  # Размер окна изображения

# Адрес ESP32 сервера
esp32_address = 'http://192.168.4.1'

# Создаем массивы для хранения координат
x_coords = deque(maxlen=points_count)
y_coords = deque(maxlen=points_count)
z_coords = deque(maxlen=packet_window)  # Для графика Z по времени

# Для второго окна
distances = [deque(maxlen=packet_window) for _ in range(4)]
packet_counter = deque(maxlen=packet_window)

# Создаем изображения для графиков
graph_img1 = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255
graph_img2 = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255
graph_img3 = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

# Функция для обновления первого изображения (график XY)
def update_image1():
    global graph_img1
    graph_img1 = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

    # Рисуем оси
    cv2.line(graph_img1, (50, img_size - 50), (img_size - 50, img_size - 50), (0, 0, 0), 2)  # X axis
    cv2.line(graph_img1, (50, img_size - 50), (50, 50), (0, 0, 0), 2)  # Y axis

    # Добавляем метки на оси
    for i in range(0, max_x + 1, max_x // 5):
        x_pos = 50 + int(i * (img_size - 100) / max_x)
        cv2.putText(graph_img1, str(i), (x_pos, img_size - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    for i in range(0, max_y + 1, max_y // 5):
        y_pos = img_size - 50 - int(i * (img_size - 100) / max_y)
        cv2.putText(graph_img1, str(i), (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Рисуем траекторию x и y
    if len(x_coords) > 1:
        for i in range(1, len(x_coords)):
            pt1 = (50 + int(x_coords[i-1] * (img_size - 100) / max_x), img_size - 50 - int(y_coords[i-1] * (img_size - 100) / max_y))
            pt2 = (50 + int(x_coords[i] * (img_size - 100) / max_x), img_size - 50 - int(y_coords[i] * (img_size - 100) / max_y))
            cv2.line(graph_img1, pt1, pt2, (255, 0, 0), 2)

# Функция для обновления второго изображения (график расстояний)
def update_image2():
    global graph_img2
    graph_img2 = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

    # Рисуем оси
    cv2.line(graph_img2, (50, img_size - 50), (img_size - 50, img_size - 50), (0, 0, 0), 2)  # X axis
    cv2.line(graph_img2, (50, img_size - 50), (50, 50), (0, 0, 0), 2)  # Y axis

    # Добавляем метки на оси
    for i in range(0, packet_window + 1, packet_window // 10):
        x_pos = 50 + int(i * (img_size - 100) / packet_window)
        cv2.putText(graph_img2, str(i), (x_pos, img_size - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    for i in range(0, max_distance + 1, max_distance // 6):
        y_pos = img_size - 50 - int(i * (img_size - 100) / max_distance)
        cv2.putText(graph_img2, str(i), (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Цвета для разных приемников
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)]

    # Рисуем линии для каждого приемника
    for i in range(4):
        if len(distances[i]) > 1:
            for j in range(1, len(distances[i])):
                pt1 = (50 + int((packet_counter[j-1] - start_packet_count) * (img_size - 100) / packet_window), 
                       img_size - 50 - int(distances[i][j-1] * (img_size - 100) / max_distance))
                pt2 = (50 + int((packet_counter[j] - start_packet_count) * (img_size - 100) / packet_window), 
                       img_size - 50 - int(distances[i][j] * (img_size - 100) / max_distance))
                cv2.line(graph_img2, pt1, pt2, colors[i], 2)

# Функция для обновления третьего изображения (график Z координаты)
def update_image3():
    global graph_img3
    graph_img3 = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

    # Рисуем оси
    cv2.line(graph_img3, (50, img_size - 50), (img_size - 50, img_size - 50), (0, 0, 0), 2)  # X axis
    cv2.line(graph_img3, (50, img_size - 50), (50, 50), (0, 0, 0), 2)  # Y axis

    # Добавляем метки на оси
    for i in range(0, packet_window + 1, packet_window // 10):
        x_pos = 50 + int(i * (img_size - 100) / packet_window)
        cv2.putText(graph_img3, str(i), (x_pos, img_size - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    for i in range(0, max_y + 1, max_y // 5):
        y_pos = img_size - 50 - int(i * (img_size - 100) / max_y)
        cv2.putText(graph_img3, str(i), (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Рисуем линии для Z координаты
    if len(z_coords) > 1:
        for i in range(1, len(z_coords)):
            pt1 = (50 + int((packet_counter[i-1] - start_packet_count) * (img_size - 100) / packet_window), 
                   img_size - 50 - int(z_coords[i-1] * (img_size - 100) / max_y))
            pt2 = (50 + int((packet_counter[i] - start_packet_count) * (img_size - 100) / packet_window), 
                   img_size - 50 - int(z_coords[i] * (img_size - 100) / max_y))
            cv2.line(graph_img3, pt1, pt2, (0, 0, 255), 2)


async def fetch_data(session):
    async with session.get(f'{esp32_address}/') as response:
        return await response.text()
    
async def receive_data():
    global packet_count, start_packet_count
    packet_count = 0
    start_packet_count = 0    

    async with aiohttp.ClientSession() as session:
        while True:
            try:
                data = await fetch_data(session)
                if data:
                    serial_data = data.strip().split(' ')

                    if len(serial_data) == 6:
                        x = float(serial_data[0])
                        y = float(serial_data[1])
                        z = float(serial_data[2])
                        receiver_id = int(serial_data[3])
                        distance = float(serial_data[4])
                        counter = int(serial_data[5])  # Счетчик можно не использовать

                        x_coords.append(x)
                        y_coords.append(y)
                        z_coords.append(z)

                        packet_count += 1
                        packet_counter.append(packet_count)
                        if (packet_count - start_packet_count > packet_window):
                            start_packet_count += 1
                        if 0 <= receiver_id < 4:
                            distances[receiver_id].append(distance)

                        # Обновляем изображения при получении новых данных
                        update_image1()
                        update_image2()
                        update_image3()
                        cv2.imshow('Trajectory Graph', graph_img1)
                        cv2.imshow('Distance Graph', graph_img2)
                        cv2.imshow('Z Coordinate Graph', graph_img3)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                        print(f"x: {x}, y: {y}, z: {z}, receiver_id: {receiver_id}, distance: {distance}, packet_count: {counter}")

            except aiohttp.ClientError as e:
                print(f"HTTP error: {e}")

try:
    asyncio.run(receive_data())
except KeyboardInterrupt:
    print("Interrupted by user, closing session.")
finally:
    cv2.destroyAllWindows()
