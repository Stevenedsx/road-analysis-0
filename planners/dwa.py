import numpy as np
from typing import List, Tuple, Optional
import math

class DWAPlanner:
    def __init__(self, map_width: int, map_height: int, obstacles: List[Tuple[int, int]]):
        """初始化DWA算法
        
        Args:
            map_width: 地图宽度
            map_height: 地图高度
            obstacles: 障碍物列表，每个障碍物是一个(x, y)元组
        """
        self.map_width = map_width
        self.map_height = map_height
        self.obstacles = set(obstacles)
        
        # 机器人参数
        self.max_speed = 1.0  # 最大速度
        self.min_speed = -0.5  # 最小速度
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # 最大角速度
        self.max_accel = 0.2  # 最大加速度
        self.max_dyaw_rate = 40.0 * math.pi / 180.0  # 最大角加速度
        self.v_resolution = 0.01  # 速度分辨率
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # 角速度分辨率
        self.dt = 0.1  # 时间间隔
        self.predict_time = 3.0  # 预测时间
        self.to_goal_cost_gain = 0.15  # 目标代价增益
        self.speed_cost_gain = 1.0  # 速度代价增益
        self.obstacle_cost_gain = 1.0  # 障碍物代价增益
        
    def is_valid_position(self, pos: Tuple[float, float]) -> bool:
        """检查位置是否有效（不超出地图范围且不在障碍物上）"""
        x, y = pos
        return (0 <= x < self.map_width and 
                0 <= y < self.map_height and 
                (int(x), int(y)) not in self.obstacles)
    
    def motion(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """运动模型
        
        Args:
            x: 状态 [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            u: 控制输入 [v(m/s), omega(rad/s)]
            dt: 时间间隔
            
        Returns:
            新状态
        """
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
        return x
    
    def calc_dynamic_window(self, x: np.ndarray) -> Tuple[float, float, float, float]:
        """计算动态窗口
        
        Args:
            x: 当前状态 [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            
        Returns:
            动态窗口 [min_v, max_v, min_yaw_rate, max_yaw_rate]
        """
        # 速度窗口
        vs = [self.min_speed, self.max_speed,
              x[3] - self.max_accel * self.dt,
              x[3] + self.max_accel * self.dt]
        
        # 角速度窗口
        yaw_rates = [-self.max_yaw_rate, self.max_yaw_rate,
                     x[4] - self.max_dyaw_rate * self.dt,
                     x[4] + self.max_dyaw_rate * self.dt]
        
        return [max(vs[0], vs[2]), min(vs[1], vs[3]),
                max(yaw_rates[0], yaw_rates[2]), min(yaw_rates[1], yaw_rates[3])]
    
    def calc_trajectory(self, x: np.ndarray, v: float, yaw_rate: float) -> List[Tuple[float, float]]:
        """计算轨迹
        
        Args:
            x: 初始状态
            v: 速度
            yaw_rate: 角速度
            
        Returns:
            轨迹点列表
        """
        trajectory = []
        time = 0
        while time <= self.predict_time:
            x = self.motion(x, [v, yaw_rate], self.dt)
            trajectory.append((x[0], x[1]))
            time += self.dt
        return trajectory
    
    def calc_to_goal_cost(self, trajectory: List[Tuple[float, float]], goal: Tuple[float, float]) -> float:
        """计算到目标点的代价"""
        dx = goal[0] - trajectory[-1][0]
        dy = goal[1] - trajectory[-1][1]
        return math.sqrt(dx**2 + dy**2)
    
    def calc_obstacle_cost(self, trajectory: List[Tuple[float, float]]) -> float:
        """计算障碍物代价"""
        min_dist = float("inf")
        for pos in trajectory:
            for obs in self.obstacles:
                dist = math.sqrt((pos[0] - obs[0])**2 + (pos[1] - obs[1])**2)
                if dist < min_dist:
                    min_dist = dist
        return 1.0 / min_dist if min_dist != float("inf") else float("inf")
    
    def calc_final_input(self, x: np.ndarray, goal: Tuple[float, float]) -> Tuple[float, float, List[Tuple[float, float]]]:
        """计算最终控制输入
        
        Args:
            x: 当前状态
            goal: 目标点
            
        Returns:
            (最佳速度, 最佳角速度, 最佳轨迹)
        """
        dw = self.calc_dynamic_window(x)
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = []
        
        # 遍历所有可能的控制输入
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for yaw_rate in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                trajectory = self.calc_trajectory(x.copy(), v, yaw_rate)
                
                # 计算代价
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.max_speed - v
                obstacle_cost = self.calc_obstacle_cost(trajectory)
                
                final_cost = (self.to_goal_cost_gain * to_goal_cost +
                            self.speed_cost_gain * speed_cost +
                            self.obstacle_cost_gain * obstacle_cost)
                
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, yaw_rate]
                    best_trajectory = trajectory
        
        return best_u[0], best_u[1], best_trajectory
    
    def find_path(self, start: Tuple[float, float, float], goal: Tuple[float, float],
                 max_iterations: int = 1000) -> Optional[List[Tuple[float, float]]]:
        """寻找路径
        
        Args:
            start: 起点 (x, y, yaw)
            goal: 终点 (x, y)
            max_iterations: 最大迭代次数
            
        Returns:
            路径点列表
        """
        # 初始化状态
        x = np.array([start[0], start[1], start[2], 0.0, 0.0])
        path = [(x[0], x[1])]
        
        for _ in range(max_iterations):
            # 计算控制输入
            v, yaw_rate, trajectory = self.calc_final_input(x, goal)
            
            # 更新状态
            x = self.motion(x, [v, yaw_rate], self.dt)
            path.append((x[0], x[1]))
            
            # 检查是否到达目标
            if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= 0.5:
                return path
        
        return None 