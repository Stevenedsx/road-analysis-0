import pygame
import yaml
import math
import sys
import os

# 添加项目根目录到 Python 路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from planners.astar import AStar
from simulation.arg_parser import parse_args, create_config_from_args

class PygameSimulator:
    def __init__(self, config):
        """初始化模拟器
        
        Args:
            config: 配置字典，可以来自配置文件或命令行参数
        """
        self.map_width = config['map']['width']
        self.map_height = config['map']['height']
        self.start = tuple(config['start'])
        self.goal = tuple(config['goal'])
        self.obstacles = [tuple(pos) for pos in config['static_obstacles']]
        self.algorithm = config.get('algorithm', 'astar')
        
        # 车辆属性
        self.car_speed = config['car']['speed']
        self.car_length = config['car']['length']
        self.car_width = config['car']['width']
        self.car_position = list(self.start)  # 车辆当前位置
        self.car_orientation = 0  # 车辆朝向（弧度）
        
        # 初始化动态障碍物
        self.dynamic_obstacles = []
        if 'dynamic_obstacles' in config:
            for obs in config['dynamic_obstacles']:
                if obs['type'] == 'circular':
                    self.dynamic_obstacles.append({
                        'type': 'circular',
                        'center': tuple(obs['center']),
                        'radius': obs['radius'],
                        'speed': obs['speed'],
                        'angle': obs['initial_angle'],
                        'position': list(obs['position'])
                    })
        
        # 初始化路径规划器
        if self.algorithm == 'astar':
            self.planner = AStar(self.map_width, self.map_height, self.obstacles)
            self.path = None
            self.current_path_index = 0
            self.path_update_interval = 100  # 每300帧更新一次路径，值越大更新越慢
            self.frame_count = 0
        
        # Pygame 初始化
        pygame.init()
        self.grid_size = 20  # 每格像素大小
        self.window_width = self.map_width * self.grid_size
        self.window_height = self.map_height * self.grid_size
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("2D 网格地图")

        # 颜色
        self.colors = {
            'background': (255, 255, 255),
            'grid': (200, 200, 200),
            'start': (0, 255, 0),
            'goal': (255, 0, 0),
            'obstacle': (100, 100, 100),
            'dynamic_obstacle': (255, 165, 0),  # 橙色
            'path': (255, 0, 0),  # 红色
            'current_path': (255, 0, 0),  # 红色
            'car': (0, 0, 255)  # 蓝色
        }
        
        # 暂停状态
        self.paused = False

    def draw_grid(self):
        for x in range(0, self.window_width, self.grid_size):
            pygame.draw.line(self.screen, self.colors['grid'], (x, 0), (x, self.window_height))
        for y in range(0, self.window_height, self.grid_size):
            pygame.draw.line(self.screen, self.colors['grid'], (0, y), (self.window_width, y))

    def draw_cell(self, x, y, color):
        rect = pygame.Rect(x * self.grid_size, y * self.grid_size, self.grid_size, self.grid_size)
        pygame.draw.rect(self.screen, color, rect)
        pygame.draw.rect(self.screen, (0, 0, 0), rect, 1)  # 黑色边框

    def draw_car(self):
        """绘制车辆"""
        x, y = self.car_position
        # 计算车辆在屏幕上的位置
        screen_x = x * self.grid_size
        screen_y = y * self.grid_size
        
        # 绘制车辆（蓝色方块）
        car_rect = pygame.Rect(screen_x, screen_y, self.grid_size, self.grid_size)
        pygame.draw.rect(self.screen, self.colors['car'], car_rect)
        pygame.draw.rect(self.screen, (0, 0, 0), car_rect, 1)  # 黑色边框
        
        # 绘制车辆朝向指示器
        center_x = screen_x + self.grid_size // 2
        center_y = screen_y + self.grid_size // 2
        end_x = center_x + math.cos(self.car_orientation) * self.grid_size // 2
        end_y = center_y + math.sin(self.car_orientation) * self.grid_size // 2
        pygame.draw.line(self.screen, (255, 255, 255), (center_x, center_y), (end_x, end_y), 2)

    def draw_path(self):
        """绘制路径"""
        if self.path and len(self.path) > 1:
            # 只绘制小车已经走过的路径
            if self.current_path_index > 0:
                # 绘制已走过的路径（红色）
                for i in range(self.current_path_index):
                    start_pos = self.path[i]
                    end_pos = self.path[i + 1]
                    pygame.draw.line(self.screen, self.colors['path'],
                                   (start_pos[0] * self.grid_size + self.grid_size // 2,
                                    start_pos[1] * self.grid_size + self.grid_size // 2),
                                   (end_pos[0] * self.grid_size + self.grid_size // 2,
                                    end_pos[1] * self.grid_size + self.grid_size // 2),
                                   3)
            
            # 绘制当前路径段（灰色）
            if self.current_path_index < len(self.path) - 1:
                current_pos = self.path[self.current_path_index]
                next_pos = self.path[self.current_path_index + 1]
                pygame.draw.line(self.screen, (200, 200, 200),  # 灰色
                               (current_pos[0] * self.grid_size + self.grid_size // 2,
                                current_pos[1] * self.grid_size + self.grid_size // 2),
                               (next_pos[0] * self.grid_size + self.grid_size // 2,
                                next_pos[1] * self.grid_size + self.grid_size // 2),
                               3)

    def update_dynamic_obstacles(self):
        """更新动态障碍物的位置"""
        for obs in self.dynamic_obstacles:
            if obs['type'] == 'circular':
                # 更新角度
                obs['angle'] += obs['speed']
                # 计算新位置
                center_x, center_y = obs['center']
                obs['position'][0] = center_x + obs['radius'] * math.cos(obs['angle'])
                obs['position'][1] = center_y + obs['radius'] * math.sin(obs['angle'])

    def update_car_position(self):
        """更新车辆位置"""
        if self.path and not self.paused:
            if self.current_path_index < len(self.path) - 1:
                current_pos = self.path[self.current_path_index]
                next_pos = self.path[self.current_path_index + 1]
                
                # 计算到下一个点的方向
                dx = next_pos[0] - current_pos[0]
                dy = next_pos[1] - current_pos[1]
                self.car_orientation = math.atan2(dy, dx)
                
                # 更新车辆位置
                self.car_position = list(next_pos)
                self.current_path_index += 1
                
                # 检查是否到达终点
                if self.current_path_index >= len(self.path) - 1:
                    self.car_position = list(self.goal)
                    self.paused = True  # 到达终点后自动暂停
                    print("已到达终点，模拟器暂停")

    def update_path(self):
        """更新路径"""
        if self.algorithm == 'astar' and not self.paused:
            self.frame_count += 1
            if self.frame_count >= self.path_update_interval:
                self.frame_count = 0
                # 重新规划路径
                self.path = self.planner.find_path(self.start, self.goal)
                if self.path:
                    self.current_path_index = 0
                    # 在屏幕上显示路径长度
                    path_length = len(self.path)
                    print(f"当前路径长度：{path_length} 个格子")
                else:
                    print("未找到可行路径！")

    def toggle_pause(self):
        """切换暂停状态"""
        self.paused = not self.paused
        print(f"模拟器已{'暂停' if self.paused else '继续'}")

    def run(self):
        running = True
        clock = pygame.time.Clock()

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:  # 空格键切换暂停状态
                        self.toggle_pause()

            if not self.paused:
                self.update_dynamic_obstacles()
                self.update_path()
                self.update_car_position()

            self.screen.fill(self.colors['background'])
            self.draw_grid()

            # 画起点
            self.draw_cell(*self.start, self.colors['start'])
            # 画终点
            self.draw_cell(*self.goal, self.colors['goal'])
            # 画静态障碍物
            for ox, oy in self.obstacles:
                self.draw_cell(ox, oy, self.colors['obstacle'])
            # 画动态障碍物
            for obs in self.dynamic_obstacles:
                x, y = map(int, obs['position'])  # 转换为整数坐标
                self.draw_cell(x, y, self.colors['dynamic_obstacle'])
            
            # 画路径
            if self.algorithm == 'astar':
                self.draw_path()
            
            # 画车辆
            self.draw_car()

            pygame.display.flip()
            clock.tick(60)  # 保持60FPS的帧率

        pygame.quit()

if __name__ == '__main__':
    # 解析命令行参数
    args = parse_args()
    
    if args.use_args:
        # 使用命令行参数创建配置
        config = create_config_from_args(args)
    else:
        # 从配置文件加载
        with open(args.config, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
    
    # 创建并运行模拟器
    sim = PygameSimulator(config)
    sim.run()
