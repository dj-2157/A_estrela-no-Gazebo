import heapq
import os
import json
from matplotlib import pyplot as plt
import numpy as np

def load_costmap(filename="base_maze.txt"):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, filename)
    print(f"Buscando o arquivo em: {file_path}")
    with open(file_path, "r") as f:
        return np.array(json.load(f))

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(costmap, start, goal):
    rows, cols = costmap.shape
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, cost, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dy, current[1] + dx)
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            if costmap[neighbor[0]][neighbor[1]] == 1:
                continue  # obstáculo

            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                came_from[neighbor] = current

    return None  # Caminho não encontrado

def matriz_to_world_coords(path, size, resolution=1.0):
    world_path = []
    offset = size // 2
    for row, col in path:
        x = (col - offset) * resolution
        y = (row - offset) * resolution
        world_path.append((x, y))
    return world_path

def plot_base_maze(costmap, path, start, goal):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(costmap, cmap='Greys', origin='upper')

    if path:
        path_y, path_x = zip(*path)
        ax.plot(path_x, path_y, color='red', linewidth=2, label="Caminho A*")
        ax.scatter(start[1], start[0], c='green', s=100, label='Início')
        ax.scatter(goal[1], goal[0], c='blue', s=100, label='Objetivo')

    ax.set_title("Caminho gerado pelo A* no labirinto")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.legend()
    plt.tight_layout()
    plt.show()

def convert_path_to_world_coords_in_expanded_maze(cell_size=3.0):
    """
    Converte o caminho da matriz 16x16 para coordenadas reais do mundo Gazebo.

    O carrinho é spawnado em <pose>-3 0 0 0 0 0</pose> no SDF,
    mas o odômetro sempre começa em (0, 0).
    O offset de +3 no X compensa essa diferença.

    Fórmula: x = (col - 8) * cell_size + spawn_offset_x
             y = (8 - row) * cell_size
    """
    costmap = load_costmap("base_maze.txt")

    start = (8, 7)   # origem do odômetro (0, 0)
    goal  = (15, 2)  # destino final

    SPAWN_OFFSET_X = 3.0  # carrinho spawnado em x=-3, odom começa em 0

    path = astar(costmap, start, goal)
    if not path:
        print("❌ Caminho não encontrado.")
        return []

    print(f"Caminho na matriz: {path}")

    world_path = []
    for row, col in path:
        x = (col - 8) * cell_size + SPAWN_OFFSET_X
        y = (8 - row) * cell_size
        world_path.append((float(x), float(y)))

    return world_path