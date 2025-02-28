#using a ld06 lidar sensor and a raspberry pi 3b
#in this version i will try to get a map of the room using the lidar sensor in stationary mode
#i will use the lidar sensor to get the distance to the walls and then plot them in a 2d grid
#i will use the pygame library to plot the map in real time
#i am using a esp32 to run the lidar sensor and send the data to the raspberry pi using websockets
#i will use the websockets library to get the data from the esp32
# i will use the numpy library to store the data in a 2d grid

# import the necessary libraries
import socket
import numpy as np
import pygame
import websocket
import math

#initialize the wifi connection to the esp32
Esp32_IP = "Add this later"
Port =  8888
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((Esp32_IP, Port))

#occupancy grid
# set the grid to be 500 to start
Map_size= 500
#size of the grid ie resolution, prob start with 0.05 meters
Resolution = 0.05
#initialize the grid
Grid = np.zeros((Map_size, Map_size))


#origin of the grid, x and y
Origin_x = Map_size/2
Origin_y = Map_size/2

#initialize the pygame window
pygame.init()
screen = pygame.display.set_mode((Map_size, Map_size))
pygame.display.set_caption('Lidar Map')

clock = pygame.time.Clock()

# update the grid with the new data
def update_grid(data):
    """Convert LiDAR data to a 2D occupancy grid."""
    global Origin_x, Origin_y

    #cartesian coordinates
    theta = np.radians(angle)
    x = Origin_x + (distance * np.cos(theta)) / Grid
    y = Origin_y + (distance * np.sin(theta)) / Grid

    #update the grid
    if 0 <= int(x) < Map_size and 0 <= int(y) < Map_size:
        Grid[int(y)][int(x)] = 255  # White pixel for obstacles
while True:
    # Read LiDAR data from ESP32
    data = sock.recv(1024)
    if not data:
        continue

    # Parse LiDAR packets
    for i in range(0, len(data) - 10, 10):
        if data[i] == 0x54:  # LiDAR packet header
            angle = (data[i+2] | (data[i+3] << 8)) / 100.0
            distance = (data[i+4] | (data[i+5] << 8)) / 1000.0  # Convert mm to meters

            if 0.1 < distance < 3:  # Ignore outliers
                update_grid(angle, distance)

    # Display occupancy grid
    screen.fill((0, 0, 0))
    for y in range(Map_size):
        for x in range(Map_size):
            if Grid[y][x] > 0:
                pygame.draw.rect(screen, (255, 255, 255), (x, y, 1, 1))
    pygame.display.flip()
    clock.tick(10)


#close the socket
sock.close()


