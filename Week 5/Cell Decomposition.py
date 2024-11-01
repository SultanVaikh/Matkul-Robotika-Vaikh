import numpy as np

def cell_decomposition(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    cells = []

    # Buat cell dari setiap blok kosong (0)
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 0:
                cells.append((i, j))

    # Jalur dari start ke goal menggunakan cell bebas
    path = []
    current = start
    while current != goal:
        path.append(current)
        x, y = current
        neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        
        # Temukan tetangga yang tersedia dalam cell yang tidak ada rintangan
        valid_neighbors = [n for n in neighbors if n in cells]
        
        if valid_neighbors:
            current = min(valid_neighbors, key=lambda n: abs(n[0] - goal[0]) + abs(n[1] - goal[1]))
        else:
            print("Tidak ada jalur yang ditemukan")
            return None
    
    path.append(goal)
    print("Jalur yang Ditempuh:", path)
    return path

# Contoh grid (0 = jalan, 1 = rintangan)
grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0]
]
start = (0, 0)
goal = (4, 4)
cell_decomposition(grid, start, goal)