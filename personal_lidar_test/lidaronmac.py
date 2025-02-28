import serial
import numpy as np
import pygame
import math

ser = serial.Serial('/dev/cu.usbserial-0001', 115200)  # Check with `ls /dev/tty.*`

MAP_SIZE = 500
GRID_RESOLUTION = 0.05
grid = np.zeros((MAP_SIZE, MAP_SIZE))

robot_x, robot_y = MAP_SIZE // 2, MAP_SIZE // 2

pygame.init()
screen = pygame.display.set_mode((MAP_SIZE, MAP_SIZE))
clock = pygame.time.Clock()

def update_map(angle, distance):
    theta = np.radians(angle)
    x = robot_x + (distance * np.cos(theta)) / GRID_RESOLUTION
    y = robot_y + (distance * np.sin(theta)) / GRID_RESOLUTION
    if 0 <= int(x) < MAP_SIZE and 0 <= int(y) < MAP_SIZE:
        grid[int(y)][int(x)] = 255

while True:
    data = ser.read(1024)
    for i in range(0, len(data) - 10, 10):
        if data[i] == 0x54:
            angle = (data[i+2] | (data[i+3] << 8)) / 100.0
            distance = (data[i+4] | (data[i+5] << 8)) / 1000.0
            if 0.1 < distance < 3:
                update_map(angle, distance)

    screen.fill((0, 0, 0))
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            if grid[y][x] > 0:
                pygame.draw.rect(screen, (255, 255, 255), (x, y, 1, 1))
    pygame.display.flip()
    clock.tick(10)