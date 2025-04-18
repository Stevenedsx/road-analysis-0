import numpy as np
import matplotlib.pyplot as plt
import math

class AStar:
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        self.grid = np.zeros((height, width))
        for x, y in obstacles:
            self.grid[y][x] = 1  # 1表示障碍物

    def is_valid_position(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == 0

    def get_neighbors(self, x, y):
        # 只考虑上下左右四个方向
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 上、右、下、左
        neighbors = []
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if self.is_valid_position(new_x, new_y):
                neighbors.append((new_x, new_y))
        return neighbors

    def heuristic(self, a, b):
        # 使用曼哈顿距离作为启发式函数
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def find_path(self, start, goal):
        start = tuple(start)
        goal = tuple(goal)
        
        if not self.is_valid_position(*start) or not self.is_valid_position(*goal):
            print("未找到可行路径！")
            return None

        open_set = {start}
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                print(f"找到路径，共经过 {len(path)} 个格子")
                return path

            open_set.remove(current)
            for neighbor in self.get_neighbors(*current):
                tentative_g_score = g_score[current] + 1  # 每个移动的代价都是1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

        print("未找到可行路径！")
        return None  # 没有找到路径

    def visualize_path(self, path, save_path=None):
        plt.figure(figsize=(10, 10))
        
        # 绘制网格
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x] == 1:
                    plt.fill([x, x+1, x+1, x], [y, y, y+1, y+1], 'gray')
                else:
                    plt.plot([x, x+1, x+1, x, x], [y, y, y+1, y+1, y], 'k-', alpha=0.2)
        
        # 绘制路径
        if path:
            path_x = [p[0] + 0.5 for p in path]
            path_y = [p[1] + 0.5 for p in path]
            plt.plot(path_x, path_y, 'r-', linewidth=2)
            plt.plot(path_x[0], path_y[0], 'go', markersize=10)  # 起点
            plt.plot(path_x[-1], path_y[-1], 'ro', markersize=10)  # 终点
        
        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True)
        
        if save_path:
            plt.savefig(save_path)
        plt.close() 