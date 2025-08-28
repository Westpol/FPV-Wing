import pygame
import time
import math


class Screen:
    def __init__(self):
        pygame.init()
        self.screen_height = 1000
        self.screen_width = 1500
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.attitude = (0, 0, 0.0)     # (pitch, roll, yaw - use GPS heading for yaw)
        self.gps_speed = 0.0
        self.gps_heading = 0
        self.baro_altitude = 0.0
        self.running = True
        self.clock = pygame.time.Clock()
        self.delta_t = 0


    def start(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.update_display()

            self.delta_t = self.clock.tick(10) / 1000
        pygame.quit()

    def update_values(self, index, value):
        pass

    def update_display(self):
        self.screen.fill((0, 0, 0))
        PFD_SKY_BLUE = (5, 150, 255)
        PFD_EARTH_BROWN = (154, 71, 16)
        PFD_GREEN = (0, 255, 3)
        PFD_INDICATOR_BACKGROUND_GRAY = (118, 118, 122)
        PFD_FOREGROUND_BLACK = (3, 3, 9)
        PFD_YELLOW = (254, 254, 3)
        PFD_WHITE = (255, 255, 255)
        PFD_HEIGHT = 830
        PFD_WIDTH = 800
        PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE = 100
        PFD_ATTITUDE_PIXELS_PER_DEGREE = 10
        PFD_PITCH_MARKING_2_5_DEGREE_WIDTH = 30
        PFD_PITCH_MARKING_5_DEGREE_WIDTH = 50
        PFD_PITCH_MARKING_10_DEGREE_WIDTH = 100

        PFD_CENTER = (400, 600)

        ARTIFICIAL_HORIZON_DIMENSIONS = 1200
        artificial_horizon = pygame.Surface((ARTIFICIAL_HORIZON_DIMENSIONS, ARTIFICIAL_HORIZON_DIMENSIONS), pygame.SRCALPHA)

        #test_surface.fill(PFD_SKY_BLUE)

        #pygame.draw.line(test_surface, (0, 0, 0), (0, 25), (50, 25))

        pygame.draw.rect(artificial_horizon, PFD_SKY_BLUE, (0, 0, ARTIFICIAL_HORIZON_DIMENSIONS, ARTIFICIAL_HORIZON_DIMENSIONS / 2))
        pygame.draw.rect(artificial_horizon, PFD_EARTH_BROWN, (0, ARTIFICIAL_HORIZON_DIMENSIONS / 2, ARTIFICIAL_HORIZON_DIMENSIONS, ARTIFICIAL_HORIZON_DIMENSIONS / 2))
        pygame.draw.line(artificial_horizon, PFD_WHITE, (0, ARTIFICIAL_HORIZON_DIMENSIONS / 2), (ARTIFICIAL_HORIZON_DIMENSIONS, ARTIFICIAL_HORIZON_DIMENSIONS / 2), 4)
        offset = (self.gps_heading % 10) / 10
        for i in range(int(-(ARTIFICIAL_HORIZON_DIMENSIONS / 80 + 2) / 2), int((ARTIFICIAL_HORIZON_DIMENSIONS / 80 + 2) / 2)):
            x_pos = (i * 80) + (offset * 80) + 600
            pygame.draw.line(artificial_horizon, PFD_WHITE, (x_pos, ARTIFICIAL_HORIZON_DIMENSIONS / 2), (x_pos, ARTIFICIAL_HORIZON_DIMENSIONS / 2 + 15), 4)

        for i in range(-4, 5):      # 10 degree lines
            if i == 0:
                continue
            y_pos = i * PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE + 600
            pygame.draw.line(artificial_horizon, PFD_WHITE, (ARTIFICIAL_HORIZON_DIMENSIONS / 2 - PFD_PITCH_MARKING_10_DEGREE_WIDTH / 2, y_pos), (ARTIFICIAL_HORIZON_DIMENSIONS / 2 + PFD_PITCH_MARKING_10_DEGREE_WIDTH / 2, y_pos), 4)

        for i in range(-4, 4):      # 5 degree lines
            y_pos = i * PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE + 650
            pygame.draw.line(artificial_horizon, PFD_WHITE, (ARTIFICIAL_HORIZON_DIMENSIONS / 2 - PFD_PITCH_MARKING_5_DEGREE_WIDTH / 2, y_pos), (ARTIFICIAL_HORIZON_DIMENSIONS / 2 + PFD_PITCH_MARKING_5_DEGREE_WIDTH / 2, y_pos), 4)

        for i in range(-8, 8):  # 2.5 degree lines
            y_pos = i * (PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE / 2) + 625
            pygame.draw.line(artificial_horizon, PFD_WHITE,
                             (ARTIFICIAL_HORIZON_DIMENSIONS / 2 - PFD_PITCH_MARKING_2_5_DEGREE_WIDTH / 2, y_pos),
                             (ARTIFICIAL_HORIZON_DIMENSIONS / 2 + PFD_PITCH_MARKING_2_5_DEGREE_WIDTH / 2, y_pos), 4)

        font = pygame.font.SysFont("Arial", 35)
        pitch_values = list(range(-40, 50, 10))

        for pitch in pitch_values:
            if pitch == 0:
                continue

            text = font.render(str(abs(pitch)), True, PFD_WHITE)
            artificial_horizon.blit(text, (600 + 70, 582 - ((pitch / 10) * PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE)))
            artificial_horizon.blit(text, (600 - 105, 582 - ((pitch / 10) * PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE)))

        rotated_horizon = pygame.transform.rotozoom(artificial_horizon, self.attitude[1], 1.0)

        rotated_rect = rotated_horizon.get_rect()
        pitch_pixel = self.attitude[0] * 10
        rotated_rect.center = (PFD_CENTER[0] + (math.sin(math.radians(self.attitude[1])) * (pitch_pixel)), PFD_CENTER[1] + (math.cos(math.radians(self.attitude[1])) * (pitch_pixel)))

        self.screen.blit(rotated_horizon, rotated_rect.topleft)

        artificial_horizon_mask = pygame.Surface((4000, 4000))
        pygame.draw.rect(artificial_horizon_mask, (255, 255, 255), (180, 450, 427, 300))
        pygame.draw.circle(artificial_horizon_mask, (255, 255, 255), (393, 580), 250)
        pygame.draw.circle(artificial_horizon_mask, (255, 255, 255), (393, 620), 250)
        pygame.draw.rect(artificial_horizon_mask, (0, 0, 0), (180 - 300, 0, 300, 1000))
        pygame.draw.rect(artificial_horizon_mask, (0, 0, 0), (180 + 427, 0, 1000, 1000))
        artificial_horizon_mask.set_colorkey((255, 255, 255))
        self.screen.blit(artificial_horizon_mask, (0,0))

        top_mask_static_things = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)

        center_rect = pygame.Rect(592, 592, 18, 18)
        #pygame.draw.rect(self.screen, PFD_FOREGROUND_BLACK, center_rect)
        pygame.draw.rect(top_mask_static_things, PFD_YELLOW, center_rect, width=4)
        right_wing_points = [(592 + 100, 592), (592 + 200, 592), (592 + 200, 608), (592 + 120, 608), (592 + 120, 620), (592 + 100, 620)]
        left_wing_points = [(592 - 100, 592), (592 - 200, 592), (592 - 200, 608), (592 - 120, 608), (592 - 120, 620), (592 - 100, 620)]
        pygame.draw.polygon(top_mask_static_things, PFD_FOREGROUND_BLACK, right_wing_points)
        pygame.draw.polygon(top_mask_static_things, PFD_YELLOW, right_wing_points, width=3)
        pygame.draw.polygon(top_mask_static_things, PFD_FOREGROUND_BLACK, left_wing_points)
        pygame.draw.polygon(top_mask_static_things, PFD_YELLOW, left_wing_points, width=3)

        top_mask_static_things = pygame.transform.rotozoom(top_mask_static_things, 0, 1.0)
        self.screen.blit(top_mask_static_things, (PFD_CENTER[0] - 600, PFD_CENTER[1] - 600))

        pygame.display.flip()


if __name__ == "__main__":
    screen = Screen()

    screen.start()
