import pygame
import math
import time

screen = pygame.display.set_mode((1000, 500))

def wave(x):
    return (math.sin(2 * x * math.pi / 180) + math.sin(6 * x * math.pi / 180) + math.cos(5 * x * math.pi / 180)) / 3
    # return math.sin(math.radians(2 * x))

check_for_frequency = 0
fourier_list = [0] * 700

while 1:
    screen.fill((0, 0, 0))

    pygame.draw.line(screen, (255, 255, 255), (280, 150), (1000, 150))

    check_for_frequency = round(check_for_frequency + 0.01, 2)

    for i in range(1, 720):
        point_height_last = 150 - wave(i - 1) * 100
        point_height_current = 150 - wave(i) * 100
        pygame.draw.line(screen, (255, 255, 255), (280 + i - 1, point_height_last), (280 + i, point_height_current))

    for i in range(0, 720, int(360 / check_for_frequency)):
        pygame.draw.line(screen, (255, 255, 255), (280 + i, 50), (280 + i, 250))

    y_integrated = 0
    x_integrated = 0

    for i in range(1, 10 * 360):
        amplitude_last = wave(i - 1) * 100
        amplitude_current = wave(i) * 100
        y_pos = math.sin(math.radians((i - 1) * check_for_frequency)) * amplitude_last
        y_integrated += y_pos
        x_pos = math.cos(math.radians((i - 1) * check_for_frequency)) * amplitude_last
        x_integrated += x_pos
        pygame.draw.line(screen, (255, 255, 255), (100 + x_pos, 100 + y_pos), (100 + math.cos(math.radians(i * check_for_frequency)) * amplitude_current, 100 + math.sin(math.radians(i * check_for_frequency)) * amplitude_current))

    fourier_list[int(check_for_frequency * 100)] = - 3 * (math.fabs(x_integrated / (10 * 360)) + math.fabs(y_integrated / (10 * 360)))

    pygame.draw.line(screen, (255, 255, 255), (290, 350), (1000, 350))

    for i in range(0, 800, 100):
        pygame.draw.line(screen, (255, 255, 255), (290 + i, 250), (290 + i, 450))

    for i in range(1, 700):
        pygame.draw.line(screen, (255, 255, 255), (290 + i - 1, 350 + fourier_list[i - 1]), (290 + i, 350 + fourier_list[i]))

    pygame.draw.circle(screen, (255, 255, 255), (100 , 100), 3)
    pygame.draw.circle(screen, (255, 0, 0), (100 + x_integrated / (10 * 360), 100 + y_integrated / (10 * 360)), 6)
    print(y_integrated / (10 * 360), x_integrated / (10 * 360), check_for_frequency, int(check_for_frequency * 100))
    pygame.display.flip()