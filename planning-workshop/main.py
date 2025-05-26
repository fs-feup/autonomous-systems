from operator import le
import os
import re
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Vehicle:
    x: float
    y: float
    angle: float


@dataclass
class TestCase:
    path_points: List[Tuple[float, float]]
    cones: List[Tuple[float, float]]
    vehicle: Vehicle
    def __str__(self):
        return (f"TestCase with {len(self.path_points)} path points, "
                f"{len(self.cones)} cones, "
                f"vehicle at ({self.vehicle.x}, {self.vehicle.y}) with angle {self.vehicle.angle}")


def read_test_file(file_path):
    path_points = []
    cones = []
    vehicle = None
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if parts[0] == 'P' and len(parts) >= 3:
                x, y = float(parts[1]), float(parts[2])
                path_points.append((x, y))
            elif parts[0] == 'C' and len(parts) >= 3:
                x, y = float(parts[1]), float(parts[2])
                cones.append((x, y))
            elif parts[0] == 'V' and len(parts) >= 4:
                x, y, angle = float(parts[1]), float(parts[2]), float(parts[3])
                vehicle = Vehicle(x, y, angle)
    if vehicle is None:
        print(f"Warning: No vehicle data in {file_path}, using default (0,0,0)")
        vehicle = Vehicle(0.0, 0.0, 0.0)
    return TestCase(path_points, cones, vehicle)


def process_folder(folder_path):
    test_cases = []
    if not os.path.isdir(folder_path):
        print(f"Error: Folder '{folder_path}' does not exist.")
        return test_cases
    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    for file_name in files:
        input_path = os.path.join(folder_path, file_name)
        try:
            test_case = read_test_file(input_path)
            test_cases.append(test_case)
            print(f"Processed '{file_name}': {test_case}")
        except Exception as e:
            print(f"Error processing '{file_name}': {e}")
    return test_cases


#####################################################################
###################### IMPLEMENT THIS FUNCTION ######################
#####################################################################

def border_estimation(test_case):
    """
    IMPLEMENT YOUR BORDER ESTIMATION ALGORITHM HERE
    
    This function should separate the cones into left and right borders.
    
    Input:
        test_case: A TestCase object containing path points, cones, and vehicle state
    
    Output:
        left_cones: List of (x,y) tuples representing the left border cones
        right_cones: List of (x,y) tuples representing the right border cones
    
    Notes:
        - Classify each cone as either left or right border
        - The visualization code will automatically display:
            - Left cones in blue
            - Right cones in yellow
        - No need to modify any other part of the code
    """
    
    if not test_case.cones:
        return [], []
    
    cones = test_case.cones
    path_points = test_case.path_points
    
    left_cones, right_cones = classify_by_simple_path_geometry(cones, path_points)

    return left_cones, right_cones


def classify_by_simple_path_geometry(cones, path_points):
    """
    Classifica cones em esquerda e direita baseado na geometria de um caminho.
    
    Esta função utiliza produto vetorial para determinar em que lado de um caminho
    cada cone está localizado. Para cada cone, encontra o ponto mais próximo no 
    caminho e usa a direção do segmento para classificar o cone como esquerdo ou direito.
    
    Args:
        cones: Lista de cones, onde cada cone é uma tupla/lista [x, y, ...]
        path_points: Lista de pontos que definem o caminho, onde cada ponto é [x, y]
    
    Returns:
        tuple: (left_cones, right_cones) - duas listas contendo os cones classificados
    
    Algoritmo:
        1. Para cada cone, encontra o segmento de caminho mais próximo
        2. Calcula o ponto mais próximo nesse segmento
        3. Usa produto vetorial para determinar o lado (positivo = esquerda, negativo = direita)
    """
    left_cones = []
    right_cones = []
    
    # Processa cada cone individualmente
    for cone in cones:
        # Extrai as coordenadas x, y do cone
        cone_pos = np.array([cone[0], cone[1]])
        
        # Inicializa variáveis para encontrar o segmento de caminho mais próximo
        min_distance = float('inf')
        best_segment_dir = None  # Direção normalizada do melhor segmento
        best_closest_point = None  # Ponto mais próximo encontrado
        
        # Itera através de todos os segmentos do caminho
        for i in range(len(path_points) - 1):
            # Define os pontos inicial e final do segmento atual
            p1 = np.array(path_points[i])
            p2 = np.array(path_points[i + 1])
            
            # Encontra o ponto mais próximo no segmento atual
            closest_point, distance = closest_point_on_segment(p1, p2, cone_pos)
            
            # Verifica se este é o segmento mais próximo encontrado até agora
            if distance < min_distance:
                min_distance = distance
                best_closest_point = closest_point
                
                # Calcula e normaliza a direção do segmento
                segment_vec = p2 - p1
                segment_length = np.linalg.norm(segment_vec)
                
                # Evita divisão por zero para segmentos degenerados
                if segment_length > 1e-10:
                    best_segment_dir = segment_vec / segment_length
        
        # Classifica o cone usando o melhor segmento encontrado
        if best_segment_dir is not None and best_closest_point is not None:
            # Calcula o vetor do ponto no caminho para o cone
            path_to_cone = cone_pos - best_closest_point
            
            # Produto vetorial 2D: determina o lado do cone em relação ao caminho
            # Produto vetorial positivo = cone à esquerda do caminho
            # Produto vetorial negativo = cone à direita do caminho
            # Fórmula 2D: cross(a, b) = a[0]*b[1] - a[1]*b[0]
            cross_product = best_segment_dir[0] * path_to_cone[1] - best_segment_dir[1] * path_to_cone[0]
            
            # Classifica baseado no sinal do produto vetorial
            if cross_product > 0:
                left_cones.append(cone)  # Cone à esquerda
            else:
                right_cones.append(cone)  # Cone à direita
        else:
            # Fallback: se não conseguir determinar a direção, coloca à direita
            # Isso pode acontecer com caminhos degenerados ou dados inválidos
            right_cones.append(cone)
    
    return left_cones, right_cones


def closest_point_on_segment(p1, p2, point):
    """
    Encontra o ponto mais próximo em um segmento de linha para um ponto dado.
    
    Esta função projeta um ponto sobre um segmento de linha definido por dois pontos.
    O resultado é sempre um ponto que está dentro do segmento (não na extensão da linha).
    
    Args:
        p1 (np.array): Ponto inicial do segmento [x, y]
        p2 (np.array): Ponto final do segmento [x, y]
        point (np.array): Ponto para o qual encontrar o mais próximo [x, y]
    
    Returns:
        tuple: (closest_point, distance)
            - closest_point (np.array): Coordenadas do ponto mais próximo no segmento
            - distance (float): Distância euclidiana entre o ponto e o ponto mais próximo
    
    Algoritmo:
        1. Calcula vetores do segmento e do ponto inicial ao ponto dado
        2. Projeta o ponto sobre a linha usando produto escalar
        3. Limita a projeção ao segmento (parâmetro t entre 0 e 1)
        4. Calcula o ponto final e a distância
    """
    # Calcula o vetor que define o segmento de linha
    segment_vec = p2 - p1
    
    # Calcula o vetor do ponto inicial do segmento ao ponto dado
    point_vec = point - p1
    
    # Calcula o comprimento ao quadrado do segmento (evita sqrt desnecessário)
    segment_length_sq = np.dot(segment_vec, segment_vec)
    
    # Verifica se o segmento é degenerado (pontos p1 e p2 são muito próximos)
    if segment_length_sq < 1e-10:
        # Para segmento degenerado, o ponto mais próximo é o próprio p1
        closest_point = p1
    else:
        # Projeta o ponto sobre a linha usando produto escalar
        # t representa a posição ao longo do segmento (0 = p1, 1 = p2)
        t = np.dot(point_vec, segment_vec) / segment_length_sq
        
        # Limita t ao intervalo [0, 1] para garantir que o ponto esteja no segmento
        # t < 0: ponto mais próximo é p1
        # t > 1: ponto mais próximo é p2
        # 0 <= t <= 1: ponto está na projeção dentro do segmento
        t = max(0, min(1, t))
        
        # Calcula o ponto mais próximo usando interpolação linear
        closest_point = p1 + t * segment_vec
    
    # Calcula a distância euclidiana entre o ponto original e o ponto mais próximo
    distance = np.linalg.norm(point - closest_point)
    
    return closest_point, distance


def visualize_track(test_case, left_cones, right_cones):
    plt.figure(figsize=(12, 8))
    if test_case.path_points:
        path_x, path_y = zip(*test_case.path_points)
        plt.plot(path_x, path_y, 'g-', label='Path')
    if left_cones:
        left_x, left_y = zip(*left_cones)
        plt.scatter(left_x, left_y, color='blue', s=100, marker='^', label='Left cones')
    if right_cones:
        right_x, right_y = zip(*right_cones)
        plt.scatter(right_x, right_y, color='yellow', s=100, marker='^', edgecolors='black', label='Right cones')
    vehicle = test_case.vehicle
    plt.scatter(vehicle.x, vehicle.y, color='red', s=150, marker='o', label='Vehicle')
    arrow_length = 2.0
    dx = arrow_length * np.cos(vehicle.angle)
    dy = arrow_length * np.sin(vehicle.angle)
    plt.arrow(vehicle.x, vehicle.y, dx, dy, head_width=0.5, head_length=0.7, fc='red', ec='red')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Track Visualization')
    plt.legend()
    return plt


def main(folder_path):
    test_cases = process_folder(folder_path)
    for i, test_case in enumerate(test_cases):
        left_cones, right_cones = border_estimation(test_case)
        plot = visualize_track(test_case, left_cones, right_cones)
        plot.savefig(f"./planning-workshop/output/track_visualization_{i+1}.png")
        print(f"Visualization saved as track_visualization_{i+1}.png")
        plot.close()


if __name__ == "__main__":
    folder_path = "./planning-workshop/input"
    main(folder_path)