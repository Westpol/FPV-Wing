from time import sleep

import pygame
import time


class Screen:
    def __init__(self):
        pygame.init()
        self.screen_height = 1000
        self.screen_width = 2000
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.attitude = (0.0, 0.0, 0.0)
        self.gps_speed = 0.0
        self.baro_altitude = 0.0


    def start(self):
        pass

    def update_values(self, index, value):
        pass

    def update_display(self):
        PFD_SKY_BLUE = (5, 150, 255)
        PFD_EARTH_BROWN = (154, 71, 16)
        PFD_GREEN = (0, 255, 3)
        PFD_INDICATOR_BACKGROUND_GRAY = (118, 118, 122)
        PFD_FOREGROUND_BLACK = (3, 3, 9)
        PFD_YELLOW = (254, 254, 3)
        PFD_WHITE = (255, 255, 255)

        self.screen.fill((0, 0, 0))
        pygame.draw.rect(self.screen, PFD_EARTH_BROWN, (0, self.screen_height - 840, 820, 840))
        pygame.display.flip()


if __name__ == "__main__":
    screen = Screen()

    screen.start()

    screen.update_display()

    time.sleep(5)
