import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from dwa import DWAPlanner
import math

def load_config(config_path: str) -> dict:
    """加载配置文件"""
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def save_path_image(map_width: int, map_height: int, 
                   obstacles: list, path: list, 
                   start: tuple, goal: tuple,
                   output_path: str):
    """保存路径图像
    
    Args:
        map_width: 地图宽度
        map_height: 地图高度
        obstacles: 障碍物列表
        path: 路径点列表
        start: 起点坐标
        goal: 终点坐标
        output_path: 输出图片路径
    """
    # 创建图形
    plt.figure(figsize=(10, 10))
    
    # 绘制网格
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(np.arange(0, map_width + 1, 1))
    plt.yticks(np.arange(0, map_height + 1, 1))
    
    # 绘制障碍物
    if obstacles:
        obs_x, obs_y = zip(*obstacles)
        plt.scatter(obs_x, obs_y, color='black', s=100, marker='s', label='障碍物')
    
    # 绘制路径
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='路径')
        plt.scatter(path_x, path_y, color='blue', s=50, marker='o')
    
    # 绘制起点和终点
    plt.scatter(start[0], start[1], color='green', s=200, marker='*', label='起点')
    plt.scatter(goal[0], goal[1], color='red', s=200, marker='*', label='终点')
    
    # 设置坐标轴
    plt.xlim(-1, map_width)
    plt.ylim(-1, map_height)
    plt.gca().set_aspect('equal', adjustable='box')
    
    # 添加图例
    plt.legend()
    
    # 添加标题
    plt.title('DWA算法路径规划结果')
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # 保存图片
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()

def main():
    # 加载配置
    config = load_config('data/maps/default_config.yaml')
    
    # 提取配置参数
    map_width = config['map']['width']
    map_height = config['map']['height']
    start = tuple(config['start'])
    goal = tuple(config['goal'])
    obstacles = [tuple(pos) for pos in config['static_obstacles']]
    
    # 创建DWA规划器实例
    dwa = DWAPlanner(map_width, map_height, obstacles)
    
    # 设置起点方向（假设初始朝向目标点）
    start_yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
    start_with_yaw = (start[0], start[1], start_yaw)
    
    # 寻找路径
    path = dwa.find_path(start_with_yaw, goal)
    
    if path:
        print("成功找到路径！")
        # 保存路径图像
        output_path = 'results/dwa_path.png'
        save_path_image(map_width, map_height, obstacles, path, start, goal, output_path)
        print(f"路径图像已保存到: {output_path}")
    else:
        print("未找到可行路径！")

if __name__ == '__main__':
    main() 