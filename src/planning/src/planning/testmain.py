import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import quad
from scipy.optimize import minimize_scalar

ac = 30      # Aceleração lateral
at = -20   # Aceleração de travagem

# Função para gerar a pista de corrida com curvas variadas
def race_track_point(t):
    # Combinação de funções trigonométricas para criar uma pista com variação
    x = 10 * np.sin(t / 5) + 20 * np.cos(t / 10)
    y = 15 * np.sin(t / 8) - 10 * np.cos(t / 4)
    return x, y

def find_circle_center(point1, point2, point3):
    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3
    mid1 = ((x1 + x2) / 2, (y1 + y2) / 2)
    mid2 = ((x2 + x3) / 2, (y2 + y3) / 2)
    slope1 = (y2 - y1) / (x2 - x1) if x2 != x1 else None
    slope2 = (y3 - y2) / (x3 - x2) if x3 != x2 else None
    perp_slope1 = -1 / slope1 if slope1 is not None else 0
    perp_slope2 = -1 / slope2 if slope2 is not None else 0
    if perp_slope1 is not None and perp_slope2 is not None:
        center_x = (perp_slope1 * mid1[0] - perp_slope2 * mid2[0] + mid2[1] - mid1[1]) / (perp_slope1 - perp_slope2)
        center_y = perp_slope1 * (center_x - mid1[0]) + mid1[1]
    elif perp_slope1 is None:
        center_x = mid1[0]
        center_y = perp_slope2 * (center_x - mid2[0]) + mid2[1]
    elif perp_slope2 is None:
        center_x = mid2[0]
        center_y = perp_slope1 * (center_x - mid1[0]) + mid1[1]
    radius = np.sqrt((center_x - x1)**2 + (center_y - y1)**2)
    return (center_x, center_y), radius

def speed_1(radius, lat_acell):
    return math.sqrt(lat_acell * radius)

def speed_2(points, speeds, brake_acell):
    real_speeds = [speeds[-1]]
    for i in range(len(speeds) - 2, -1, -1):
        dist = math.sqrt((points[i][0] - points[i + 1][0])**2 + (points[i][1] - points[i + 1][1])**2)
        max_speed = math.sqrt(real_speeds[-1]**2 + 2 * abs(brake_acell) * dist)
        real_speeds.append(min(max_speed, speeds[i]))
    return real_speeds[::-1]

# Geração de pontos espaçados para a pista
num_points = 5
points = [race_track_point(i) for i in range(num_points)]
radiuses = [0]
for i in range(1, len(points) - 1):
    center, radius = find_circle_center(points[i - 1], points[i], points[i + 1])
    radiuses.append(radius)
radiuses[0] = radiuses[1]
radiuses.append(radiuses[-1])

# Cálculo das velocidades
speeds = [speed_1(r, ac) for r in radiuses]
speeds2 = speed_2(points, speeds, at)

# Cálculo de tempo para o gráfico v(t)
times = [0]
for i in range(1, len(points)):
    dist = math.sqrt((points[i][0] - points[i - 1][0])**2 + (points[i][1] - points[i - 1][1])**2)
    avg_speed = (speeds2[i] + speeds2[i - 1]) / 2
    delta_time = dist / avg_speed if avg_speed != 0 else 0
    times.append(times[-1] + delta_time)

# Plotando o gráfico v(t) e a forma da pista
plt.figure(figsize=(14, 6))

# Gráfico de v(t)
plt.subplot(1, 2, 1)
plt.plot(times, speeds2, label='Velocidade Ajustada (v(t))', color='blue')
plt.plot(times, speeds, label='Velocidade Inicial', color='orange', linestyle='--')
plt.xlabel('Tempo (s)')
plt.ylabel('Velocidade (m/s)')
plt.title('Gráfico de Velocidade em função do Tempo')
plt.legend()
plt.grid(True)

# Desenho da pista
plt.subplot(1, 2, 2)
track_x, track_y = zip(*points)
plt.plot(track_x, track_y, label='Pista de Corrida')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Forma da Pista de Corrida')
plt.axis('equal')
plt.grid(True)
plt.legend()

plt.show()
