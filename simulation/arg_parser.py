import argparse
import yaml

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='2D网格地图模拟器')
    
    # 按顺序添加位置参数，设置nargs='?'表示参数是可选的
    parser.add_argument('width', type=int, nargs='?', help='地图宽度（格子数）')
    parser.add_argument('height', type=int, nargs='?', help='地图高度（格子数）')
    parser.add_argument('start_x', type=int, nargs='?', help='起点X坐标')
    parser.add_argument('start_y', type=int, nargs='?', help='起点Y坐标')
    parser.add_argument('goal_x', type=int, nargs='?', help='终点X坐标')
    parser.add_argument('goal_y', type=int, nargs='?', help='终点Y坐标')
    
    # 可选参数
    parser.add_argument('--config', type=str, default='data/maps/default_config.yaml',
                        help='地图配置文件路径')
    parser.add_argument('--use-args', action='store_true',
                        help='是否使用命令行参数覆盖配置文件')
    parser.add_argument('--algorithm', type=str, choices=['astar', 'rrt', 'dwa'], default='astar',
                        help='路径规划算法选择')
    
    return parser.parse_args()

def create_config_from_args(args):
    """根据命令行参数创建配置字典，其他配置从默认配置文件读取"""
    # 首先从默认配置文件读取完整配置
    with open('data/maps/default_config.yaml', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # 只有在提供了参数时才覆盖配置
    if args.width is not None:
        config['map']['width'] = args.width
    if args.height is not None:
        config['map']['height'] = args.height
    if args.start_x is not None and args.start_y is not None:
        config['start'] = [args.start_x, args.start_y]
    if args.goal_x is not None and args.goal_y is not None:
        config['goal'] = [args.goal_x, args.goal_y]
    
    # 添加算法选择
    config['algorithm'] = args.algorithm
    
    return config 