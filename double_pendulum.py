import pygame
from math import sin, cos, pi
import sys

pygame.init()

WIDTH = 600
HEIGHT = 600

BASE_SIZE = 8
BLACK = (10, 10, 20)
WHITE = (230, 230, 230)
RED = (255, 80, 80)
BLUE = (80, 150, 255)
TRAIL_RED = (200, 50, 50)
TRAIL_BLUE = (50, 100, 200)


m1 = 1
m2 = 1
l1 = 0.75
l2 = 1.25
g = 10
dt = 0.01


theta = 5*pi/6
phi = pi
theta_dot = 0.0
phi_dot = 0.0
global_time = 0.0

def invert_2x2(m):

    a, b = m[0][0], m[0][1]
    c, d = m[1][0], m[1][1]
    det = a * d - b * c
    if abs(det) < 1e-12:
        det = 1e-12
    inv_det = 1.0 / det
    return [
        [ d * inv_det, -b * inv_det],
        [-c * inv_det,  a * inv_det]
    ]

def mat_apply(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1],
        m[1][0] * v[0] + m[1][1] * v[1],
    ]

def derivatives(state):
    theta, phi, theta_dot, phi_dot = state

    M11 = (m1 + m2) * l1**2
    M12 = m2 * l1 * l2 * cos(theta - phi)
    M21 = M12
    M22 = m2 * l2**2

    M = [[M11, M12],
         [M21, M22]]

    A1 = -m2 * l1 * l2 * (phi_dot**2) * sin(theta - phi) - (m1 + m2) * g * l1 * sin(theta)
    A2 =  m2 * l1 * l2 * (theta_dot**2) * sin(theta - phi) - m2 * g * l2 * sin(phi)

    A = [A1, A2]

    invM = invert_2x2(M)
    theta_ddot, phi_ddot = mat_apply(invM, A)

    return [theta_dot, phi_dot, theta_ddot, phi_ddot]

def rk4_step(state, dt):
    k1 = derivatives(state)
    s2 = [state[i] + 0.5 * dt * k1[i] for i in range(4)]
    k2 = derivatives(s2)
    s3 = [state[i] + 0.5 * dt * k2[i] for i in range(4)]
    k3 = derivatives(s3)
    s4 = [state[i] + dt * k3[i] for i in range(4)]
    k4 = derivatives(s4)
    new_state = [state[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) for i in range(4)]
    return new_state

def update_physics():
    global theta, phi, theta_dot, phi_dot, global_time
    state = [theta, phi, theta_dot, phi_dot]
    new_state = rk4_step(state, dt)
    theta, phi, theta_dot, phi_dot = new_state
    global_time += dt

def total_energy(theta, phi, theta_dot, phi_dot):
    KE = 0.5 * (m1 + m2) * (l1**2) * (theta_dot**2) \
         + 0.5 * m2 * (l2**2) * (phi_dot**2) \
         + m2 * l1 * l2 * theta_dot * phi_dot * cos(theta - phi)
    PE = -(m1 + m2) * g * l1 * cos(theta) - m2 * g * l2 * cos(phi)
    return KE + PE


def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("super duper other cool thing")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 30)
    
    trail1, trail2 = [], []
    max_trail_length = 400
    SCALE = 140 
    
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

        x1 = center_x + l1 * sin(theta) * SCALE
        y1 = center_y + l1 * cos(theta) * SCALE
        
        x2 = x1 + l2 * sin(phi) * SCALE
        y2 = y1 + l2 * cos(phi) * SCALE

        trail1.append((int(x1), int(y1)))
        trail2.append((int(x2), int(y2)))
        if len(trail1) > max_trail_length: trail1.pop(0)
        if len(trail2) > max_trail_length: trail2.pop(0)

        if len(trail1) > 1:
            pygame.draw.aalines(screen, TRAIL_RED, False, trail1)
        if len(trail2) > 1:
            pygame.draw.aalines(screen, TRAIL_BLUE, False, trail2)

        p0 = (int(center_x), int(center_y))
        p1 = (int(x1), int(y1))
        p2 = (int(x2), int(y2))

        pygame.draw.circle(screen, WHITE, p0, 5)
        pygame.draw.line(screen, WHITE, p0, p1, 2)
        pygame.draw.line(screen, WHITE, p1, p2, 4)

        radius_m1 = max(2, int(BASE_SIZE * (m1 / (m1 + m2)) * 2))
        radius_m2 = max(2, int(BASE_SIZE * (m2 / (m1 + m2)) * 2))

        pygame.draw.circle(screen, RED, p1, radius_m1)
        pygame.draw.circle(screen, BLUE, p2, radius_m2)

        time_text = font.render(f"time: {global_time:.2f}s", True, WHITE)
        screen.blit(time_text, (10, 10))
        screen.blit(font.render(f"energy: {total_energy(theta, phi, theta_dot, phi_dot):.2f}J", True, WHITE), (10, 40))

        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
