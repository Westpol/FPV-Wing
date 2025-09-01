import pygame
import time
import math
import socket
import struct

class Screen:
    def __init__(self):
        pygame.init()
        self.screen_height = 1050
        self.screen_width = 1500
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.attitude = (0, 0, 0.0)     # (pitch, roll, yaw - use GPS heading for yaw)
        self.gps_speed = 0
        self.gps_heading = 0
        self.baro_altitude = 150.0
        self.running = True
        self.clock = pygame.time.Clock()
        self.delta_t = 0
        self.telemetry = Telemetry()

        # color and size defines
        self.PFD_SKY_BLUE = (5, 150, 255)
        self.PFD_EARTH_BROWN = (154, 71, 16)
        self.PFD_GREEN = (0, 255, 3)
        self.PFD_INDICATOR_BACKGROUND_GRAY = (118, 118, 122)
        self.PFD_FOREGROUND_BLACK = (3, 3, 9)
        self.PFD_YELLOW = (254, 254, 3)
        self.PFD_WHITE = (255, 255, 255)
        self.PFD_HEIGHT = 830
        self.PFD_WIDTH = 800
        self.PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE = 100
        self.PFD_ATTITUDE_PIXELS_PER_DEGREE = 10
        self.PFD_PITCH_MARKING_2_5_DEGREE_WIDTH = 30
        self.PFD_PITCH_MARKING_5_DEGREE_WIDTH = 50
        self.PFD_PITCH_MARKING_10_DEGREE_WIDTH = 100

        self.PFD_CENTER = (400, 600)

        self.ARTIFICIAL_HORIZON_DIMENSIONS = 1200


    def start(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.update_display()

            self.telemetry.wait_for_package()
            self.attitude = (self.telemetry.pitch, self.telemetry.roll, self.telemetry.yaw)

        pygame.quit()

    def update_values(self, index, value):
        pass

    def draw_artificial_horizon(self):

        artificial_horizon = pygame.Surface((self.ARTIFICIAL_HORIZON_DIMENSIONS, self.ARTIFICIAL_HORIZON_DIMENSIONS),
                                            pygame.SRCALPHA)

        # test_surface.fill(self.PFD_SKY_BLUE)

        # pygame.draw.line(test_surface, (0, 0, 0), (0, 25), (50, 25))

        pygame.draw.rect(artificial_horizon, self.PFD_SKY_BLUE,
                         (0, 0, self.ARTIFICIAL_HORIZON_DIMENSIONS, self.ARTIFICIAL_HORIZON_DIMENSIONS / 2))
        pygame.draw.rect(artificial_horizon, self.PFD_EARTH_BROWN, (
        0, self.ARTIFICIAL_HORIZON_DIMENSIONS / 2, self.ARTIFICIAL_HORIZON_DIMENSIONS, self.ARTIFICIAL_HORIZON_DIMENSIONS / 2))
        pygame.draw.line(artificial_horizon, self.PFD_WHITE, (0, self.ARTIFICIAL_HORIZON_DIMENSIONS / 2),
                         (self.ARTIFICIAL_HORIZON_DIMENSIONS, self.ARTIFICIAL_HORIZON_DIMENSIONS / 2), 4)
        offset = -((self.gps_heading % 10) / 10)
        for i in range(int(-(self.ARTIFICIAL_HORIZON_DIMENSIONS / 80 + 2) / 2),
                       int((self.ARTIFICIAL_HORIZON_DIMENSIONS / 80 + 2) / 2)):
            x_pos = (i * 80) + (offset * 80) + 600
            pygame.draw.line(artificial_horizon, self.PFD_WHITE, (x_pos, self.ARTIFICIAL_HORIZON_DIMENSIONS / 2),
                             (x_pos, self.ARTIFICIAL_HORIZON_DIMENSIONS / 2 + 15), 4)

        for i in range(-4, 5):  # 10 degree lines
            if i == 0:
                continue
            y_pos = i * self.PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE + 600
            pygame.draw.line(artificial_horizon, self.PFD_WHITE,
                             (self.ARTIFICIAL_HORIZON_DIMENSIONS / 2 - self.PFD_PITCH_MARKING_10_DEGREE_WIDTH / 2, y_pos),
                             (self.ARTIFICIAL_HORIZON_DIMENSIONS / 2 + self.PFD_PITCH_MARKING_10_DEGREE_WIDTH / 2, y_pos), 4)

        for i in range(-4, 4):  # 5 degree lines
            y_pos = i * self.PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE + 650
            pygame.draw.line(artificial_horizon, self.PFD_WHITE,
                             (self.ARTIFICIAL_HORIZON_DIMENSIONS / 2 - self.PFD_PITCH_MARKING_5_DEGREE_WIDTH / 2, y_pos),
                             (self.ARTIFICIAL_HORIZON_DIMENSIONS / 2 + self.PFD_PITCH_MARKING_5_DEGREE_WIDTH / 2, y_pos), 4)

        for i in range(-8, 8):  # 2.5 degree lines
            y_pos = i * (self.PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE / 2) + 625
            pygame.draw.line(artificial_horizon, self.PFD_WHITE,
                             (self.ARTIFICIAL_HORIZON_DIMENSIONS / 2 - self.PFD_PITCH_MARKING_2_5_DEGREE_WIDTH / 2, y_pos),
                             (self.ARTIFICIAL_HORIZON_DIMENSIONS / 2 + self.PFD_PITCH_MARKING_2_5_DEGREE_WIDTH / 2, y_pos), 4)

        font = pygame.font.SysFont("Arial", 35)
        pitch_values = list(range(-40, 50, 10))

        for pitch in pitch_values:
            if pitch == 0:
                continue

            text = font.render(str(abs(pitch)), True, self.PFD_WHITE)
            artificial_horizon.blit(text, (600 + 70, 582 - ((pitch / 10) * self.PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE)))
            artificial_horizon.blit(text, (600 - 105, 582 - ((pitch / 10) * self.PFD_ATTITUDE_10_DEGREES_PIXEL_DISTANCE)))

        rotated_horizon = pygame.transform.rotozoom(artificial_horizon, self.attitude[1], 1.0)

        rotated_rect = rotated_horizon.get_rect()
        pitch_pixel = self.attitude[0] * 10
        rotated_rect.center = (self.PFD_CENTER[0] + (math.sin(math.radians(self.attitude[1])) * (pitch_pixel)),
                               self.PFD_CENTER[1] + (math.cos(math.radians(self.attitude[1])) * (pitch_pixel)))

        self.screen.blit(rotated_horizon, rotated_rect.topleft)

        artificial_horizon_mask = pygame.Surface((4000, 4000))
        pygame.draw.rect(artificial_horizon_mask, (255, 255, 255), (180, 450, 427, 300))
        pygame.draw.circle(artificial_horizon_mask, (255, 255, 255), (393, 580), 250)
        pygame.draw.circle(artificial_horizon_mask, (255, 255, 255), (393, 620), 250)
        pygame.draw.rect(artificial_horizon_mask, (0, 0, 0), (180 - 300, 0, 300, 1000))
        pygame.draw.rect(artificial_horizon_mask, (0, 0, 0), (180 + 427, 0, 1000, 1000))
        artificial_horizon_mask.set_colorkey((255, 255, 255))
        self.screen.blit(artificial_horizon_mask, (0, 0))

    def draw_heading(self):

        heading_surface = pygame.Surface((430, 65), pygame.SRCALPHA)

        pygame.draw.rect(heading_surface, self.PFD_INDICATOR_BACKGROUND_GRAY, (0, 0, 430, 60))
        pygame.draw.lines(heading_surface, self.PFD_WHITE, False, [(1, 65), (1, 1), (426, 1), (426, 65)], 4)

        heading_offset = -((self.attitude[2] % 10) * 10)
        equalized_heading = int((self.gps_heading % 360) / 10)
        font2 = pygame.font.SysFont("Arial", 40)
        for i in range(-4, 5):
            pygame.draw.line(heading_surface, self.PFD_WHITE, (214 + (100 * i) + 50 + heading_offset, 0),
                             (214 + (100 * i) + 50 + heading_offset, 13), 4)
            pygame.draw.line(heading_surface, self.PFD_WHITE, (214 + (100 * i) + heading_offset, 0),
                             (214 + (100 * i) + heading_offset, 28), 4)
            if 10 <= equalized_heading + i < 36:
                text = font2.render(str(abs(equalized_heading + i)), True, self.PFD_WHITE)
                heading_surface.blit(text, (195 + i * 100 + heading_offset, 25))
            elif 0 <= equalized_heading + i < 36:
                text = font2.render(str(abs(equalized_heading + i)), True, self.PFD_WHITE)
                heading_surface.blit(text, (205 + i * 100 + heading_offset, 25))
            elif equalized_heading + i < 0:
                text = font2.render(str(abs(36 + equalized_heading + i)), True, self.PFD_WHITE)
                heading_surface.blit(text, (195 + i * 100 + heading_offset, 25))
            elif equalized_heading + i > 35:
                text = font2.render(str(abs(0 + ((equalized_heading + i) % 36))), True, self.PFD_WHITE)
                heading_surface.blit(text, (205 + i * 100 + heading_offset, 25))

        heading_surface = pygame.transform.rotozoom(heading_surface, 0, 1.0)

        self.screen.blit(heading_surface, (180, 925))

    def draw_top_mask(self):

        top_mask_static_things = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)

        center_rect = pygame.Rect(592, 592, 18, 18)
        # pygame.draw.rect(self.screen, self.PFD_FOREGROUND_BLACK, center_rect)
        pygame.draw.rect(top_mask_static_things, self.PFD_YELLOW, center_rect, width=4)
        right_wing_points = [(592 + 100, 592), (592 + 200, 592), (592 + 200, 608), (592 + 120, 608), (592 + 120, 620),
                             (592 + 100, 620)]
        left_wing_points = [(592 - 100, 592), (592 - 200, 592), (592 - 200, 608), (592 - 120, 608), (592 - 120, 620),
                            (592 - 100, 620)]
        pygame.draw.polygon(top_mask_static_things, self.PFD_FOREGROUND_BLACK, right_wing_points)
        pygame.draw.polygon(top_mask_static_things, self.PFD_YELLOW, right_wing_points, width=3)
        pygame.draw.polygon(top_mask_static_things, self.PFD_FOREGROUND_BLACK, left_wing_points)
        pygame.draw.polygon(top_mask_static_things, self.PFD_YELLOW, left_wing_points, width=3)
        pygame.draw.rect(top_mask_static_things, self.PFD_YELLOW, (180 + 412, 905, 6, 35))

        pygame.draw.line(top_mask_static_things, self.PFD_YELLOW, (100, 602), (216, 602), 5)
        pygame.draw.line(top_mask_static_things, self.PFD_YELLOW, (275, 602), (310, 602), 5)
        pygame.draw.polygon(top_mask_static_things, self.PFD_YELLOW, ((300, 602), (330, 592), (330, 612)))

        top_mask_static_things = pygame.transform.rotozoom(top_mask_static_things, 0, 1.0)
        self.screen.blit(top_mask_static_things, (self.PFD_CENTER[0] - 600, self.PFD_CENTER[1] - 600))

    def draw_speed(self):
        speed_mask = pygame.Surface((120, 425), pygame.SRCALPHA)

        pygame.draw.rect(speed_mask, self.PFD_INDICATOR_BACKGROUND_GRAY, (0, 0, 90, 425))
        pygame.draw.line(speed_mask, self.PFD_WHITE, (0, 1), (120, 1), 3)
        pygame.draw.line(speed_mask, self.PFD_WHITE, (0, 422), (120, 422), 3)
        pygame.draw.line(speed_mask, self.PFD_WHITE, (90, 0), (90, 500), 3)

        pixels_per_knot = 10  # 10 px per knot
        base_speed = int(self.gps_speed // 10) * 10
        offset = (self.gps_speed - base_speed) * pixels_per_knot  # smooth scroll

        font2 = pygame.font.SysFont("Arial", 35)

        for i in range(-4, 5):
            tick_value = base_speed + i * 10
            if tick_value >= 0:
                y_pos = (425 // 2) + offset - (i * 10 * pixels_per_knot)
                mid_y = y_pos + (5 * pixels_per_knot)
                pygame.draw.line(speed_mask, self.PFD_WHITE, (70, y_pos), (90, y_pos), 3)
                if tick_value > 0:
                    pygame.draw.line(speed_mask, self.PFD_WHITE, (80, mid_y), (90, mid_y), 3)

                # label
                text = font2.render(str(tick_value), True, self.PFD_WHITE)
                speed_mask.blit(text, (5, y_pos - 20))

        speed_mask = pygame.transform.rotozoom(speed_mask, 0, 1.0)

        self.screen.blit(speed_mask, (15, 390))

    def draw_height(self):
        height_mask = pygame.Surface((120, 425), pygame.SRCALPHA)

        pygame.draw.rect(height_mask, self.PFD_INDICATOR_BACKGROUND_GRAY, (10, 0, 75, 425))
        pygame.draw.line(height_mask, self.PFD_WHITE, (10, 1), (115, 1), 3)
        pygame.draw.line(height_mask, self.PFD_WHITE, (10, 422), (115, 422), 3)
        pygame.draw.line(height_mask, self.PFD_WHITE, (85, 0), (85, 500), 3)

        pixels_per_knot = 10  # 10 px per knot
        base_speed = int(self.baro_altitude // 10) * 10
        offset = (self.baro_altitude - base_speed) * pixels_per_knot  # smooth scroll

        font2 = pygame.font.SysFont("Arial", 35)

        for i in range(-4, 5):
            tick_value = base_speed + i * 10
            if tick_value >= 0:
                y_pos = (425 // 2) + offset - (i * 10 * pixels_per_knot)
                mid_y = y_pos + (5 * pixels_per_knot)
                pygame.draw.line(height_mask, self.PFD_WHITE, (75, y_pos), (85, y_pos), 3)
                pygame.draw.lines(height_mask, self.PFD_WHITE, False, [(0, y_pos + 5), (10, y_pos), (0, y_pos - 5)], 3)
                if tick_value > 0:
                    pygame.draw.line(height_mask, self.PFD_WHITE, (75, mid_y), (85, mid_y), 3)

                # label
                text = font2.render(str(tick_value), True, self.PFD_WHITE)
                height_mask.blit(text, (15, y_pos - 20))

        height_mask = pygame.transform.rotozoom(height_mask, 0, 1.0)

        self.screen.blit(height_mask, (660, 390))

    def update_display(self):
        self.screen.fill((0, 0, 0))

        self.draw_artificial_horizon()

        self.draw_heading()

        self.draw_speed()

        self.draw_height()

        self.draw_top_mask()

        pygame.display.flip()

class Telemetry:
    def __init__(self):
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        UDP_IP = "0.0.0.0"
        UDP_PORT = 8888
        self.last_attitude_update = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))

        print(f"Listening on UDP {UDP_IP}:{UDP_PORT}")

    def wait_for_package(self):
        self.sock.settimeout(0.1)
        try:
            packet, addr = self.sock.recvfrom(1024)  # buffer size 1024 bytes
        except:
            return
        #print(f"From {addr}: {packet.hex(' ')}")
        if packet[0] == 0xC8:
            if packet[2] == 0x1E:  # Attitude
                payload = packet[3:9]
                pitch_raw, roll_raw, yaw_raw = struct.unpack('>hhh', payload)
                scale = 0.0001  # 100 µrad = 0.0001 rad
                pitch_rad = pitch_raw * scale
                roll_rad = roll_raw * scale
                yaw_rad = yaw_raw * scale

                rad2deg = 180.0 / math.pi
                self.pitch = pitch_rad * rad2deg
                self.roll = roll_rad * rad2deg
                self.yaw = yaw_rad * rad2deg
                print(1.0 / (time.time() - self.last_attitude_update), " Hz")
                self.last_attitude_update = time.time()
            else:
                self.wait_for_package()




if __name__ == "__main__":
    screen = Screen()

    screen.start()
