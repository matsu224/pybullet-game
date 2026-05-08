from collections import deque

def find_furthest_point(maze, start):
    rows, cols = len(maze), len(maze[0])
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 上下左右の移動
    visited = [[False] * cols for _ in range(rows)]
    queue = deque([(start, 0)])
    visited[start[0]][start[1]] = True
    furthest_point = start
    max_distance = 0

    while queue:
        (x, y), dist = queue.popleft()
        if dist > max_distance:
            max_distance = dist
            furthest_point = (x, y)
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and not visited[nx][ny] and maze[nx][ny] == 0:
                visited[nx][ny] = True
                queue.append(((nx, ny), dist + 1))
    
    return furthest_point, max_distance

def main():
    # 例として、0が通路で1が壁の迷路
    maze = [
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 1, 1, 1, 0]
    ]

    start = (0, 0)
    furthest_point, distance = find_furthest_point(maze, start)
    print(f"最も遠い点は{furthest_point}で、距離は{distance}です")

if __name__ == '__main__':
     main()