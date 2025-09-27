import pygame
import math
from math import sin, cos, pi
import sys

pygame.init()

WIDTH = 1000
HEIGHT = 800

BASE_SIZE = 8
BLACK = (10, 10, 20)
WHITE = (230, 230, 230)
RED = (255, 80, 80)
BLUE = (80, 150, 255)
GREEN = (80, 255, 150)
GRAY = (128, 128, 128)
TRAIL_RED = (200, 50, 50)
TRAIL_BLUE = (50, 100, 200)

m1 = 1.0
m2 = 1.0
l0 = 1.25
R = 1.5
g = 9.81
k = 100.0
dt = 0.01

r = 2.0
theta = pi / 3
phi = pi / 2
r_dot = 0.0
theta_dot = 0.0
phi_dot = 0.0
global_time = 0

def invert(m):
    a = m
    det = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
           a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
           a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))

    if abs(det) < 1e-12:
        print("VERY BAD DET")
        det = 1e-12

    inv_det = 1.0 / det
    
    inv_m = [
        [
            (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det,
            (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det,
            (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det
        ],
        [
            (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det,
            (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det,
            (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det
        ],
        [
            (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det,
            (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det,
            (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det
        ]
    ]
    return inv_m

def apply(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]
    ]


def derivatives(state):
    # state: [r, theta, phi, r_dot, theta_dot, phi_dot]
    r, theta, phi, r_dot, theta_dot, phi_dot = state

    M = [
        [m1 + m2, 0, m2 * R * sin(theta - phi)],
        [0, (m1 + m2) * r**2, m2 * R * r * cos(theta - phi)],
        [m2 * R * sin(theta - phi), m2 * R * r * cos(theta - phi), m2 * R**2]
    ]

    a_r = (
        (m1 + m2) * r * theta_dot**2 +
        (m1 + m2) * g * math.cos(theta) -
        k * (r - l0) +
        m2 * R * phi_dot**2 * math.cos(theta - phi)
    )

    a_theta = (
        -2 * (m1 + m2) * r * r_dot * theta_dot -
        m2 * R * r * phi_dot**2 * math.sin(theta - phi) -
        (m1 + m2) * g * r * math.sin(theta)
    )

    a_phi = (
        -2 * m2 * R * r_dot * theta_dot * math.cos(theta - phi) +
        m2 * R * r * theta_dot**2 * math.sin(theta - phi) -
        m2 * g * R * math.sin(phi)
    )

    A = [a_r, a_theta, a_phi]


    r_ddot, theta_ddot, phi_ddot = apply(invert(M), A)

    return [r_dot, theta_dot, phi_dot, r_ddot, theta_ddot, phi_ddot]

def rk4_step(state, dt):
    # run-of-the-mill runge-kutta impl
    k1 = derivatives(state)
    s2 = [state[i] + 0.5 * dt * k1[i] for i in range(6)]
    k2 = derivatives(s2)
    s3 = [state[i] + 0.5 * dt * k2[i] for i in range(6)]
    k3 = derivatives(s3)
    s4 = [state[i] + dt * k3[i] for i in range(6)]
    k4 = derivatives(s4)
    new_state = [state[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) for i in range(6)]
    return new_state

def update_physics():
    global r, theta, phi, r_dot, theta_dot, phi_dot, global_time
    state = [r, theta, phi, r_dot, theta_dot, phi_dot]
    new_state = rk4_step(state, dt)
    r, theta, phi, r_dot, theta_dot, phi_dot = new_state
    global_time += dt

def draw_spring(screen, start_pos, end_pos, coils=20, width=5):
    start_x, start_y = start_pos
    end_x, end_y = end_pos
    
    dx = end_x - start_x
    dy = end_y - start_y
    length = math.hypot(dx, dy)
    
    if length < 1: return

    ux, uy = dx / length, dy / length
    px, py = -uy * width, ux * width
    
    points = [start_pos]
    segment_len = length / (coils * 2)

    for i in range(1, coils * 2):
        pos_on_line = (start_x + i * segment_len * ux, start_y + i * segment_len * uy)
        side = 1 if i % 2 == 1 else -1
        points.append((pos_on_line[0] + side * px, pos_on_line[1] + side * py))
    
    points.append(end_pos)
    
    pygame.draw.lines(screen, GRAY, False, points, 2)

def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("super duper cool thing")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 30)
    
    trail1, trail2 = [], []
    max_trail_length = 200
    SCALE = 100
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
        
        update_physics()

        screen.fill(BLACK)
        
        center_x, center_y = WIDTH // 2, 250

        x1 = center_x + r * sin(theta) * SCALE
        y1 = center_y + r * cos(theta) * SCALE
        
        x2 = x1 + R * sin(phi) * SCALE
        y2 = y1 + R * cos(phi) * SCALE

        trail1.append((x1, y1))
        trail2.append((x2, y2))
        if len(trail1) > max_trail_length: trail1.pop(0)
        if len(trail2) > max_trail_length: trail2.pop(0)

        if len(trail1) > 1:
            pygame.draw.aalines(screen, TRAIL_RED, False, trail1)
        if len(trail2) > 1:
            pygame.draw.aalines(screen, TRAIL_BLUE, False, trail2)

        pygame.draw.circle(screen, WHITE, (center_x, center_y), 5)
        draw_spring(screen, (center_x, center_y), (x1, y1))
        pygame.draw.line(screen, WHITE, (x1, y1), (x2, y2), 4)

        radius_m1 = int(BASE_SIZE * (m1 / (m1 + m2)) * 2)  # Scale based on mass
        radius_m2 = int(BASE_SIZE * (m2 / (m1 + m2)) * 2)  # Scale based on mass

        pygame.draw.circle(screen, RED, (int(x1), int(y1)), radius_m1)
        pygame.draw.circle(screen, BLUE, (int(x2), int(y2)), radius_m2)

        time_text = font.render(f"time: {global_time:.2f}s", True, WHITE)
        energy = 0.5 * (m1 + m2) * (r_dot**2 + r**2 * theta_dot**2) + 0.5 * m2 * R**2 * phi_dot**2 + \
                 m2 * R * r_dot * phi_dot * sin(theta - phi) + m2 * R * r * theta_dot * phi_dot * cos(theta - phi) - \
                 (m1 + m2) * g * r * cos(theta) - m2 * g * R * cos(phi) + 0.5 * k * (r - l0)**2
        energy_text = font.render(f"energ: {energy:.2f} J", True, WHITE)

        screen.blit(time_text, (10, 10))
        screen.blit(energy_text, (10, 40))
        
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
