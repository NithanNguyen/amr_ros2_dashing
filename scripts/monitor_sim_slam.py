#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import psutil
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import time
import argparse
import sys
import threading
from datetime import datetime

# --- ROS 2 LIBRARIES ---
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- BIẾN TOÀN CỤC ---
current_stage = "Stage 1: Idle"
stage_markers = [] 
is_moving = False
start_time_global = 0

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('slam_performance_monitor')
        # Subscribe cmd_vel để phát hiện robot di chuyển
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global is_moving, current_stage, stage_markers, start_time_global
        # Logic phát hiện chuyển động (bỏ qua nhiễu nhỏ)
        if not is_moving and (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01):
            is_moving = True
            switch_time = time.time() - start_time_global
            current_stage = "Stage 2: Mapping"
            stage_markers.append((switch_time, "Start Mapping"))
            print(f"\n\n[AUTO] Phát hiện chuyển động! -> Stage 2 ({switch_time:.1f}s)")

def manual_input_listener():
    global current_stage, stage_markers, start_time_global
    try:
        input() # Chờ nhấn Enter
        switch_time = time.time() - start_time_global
        current_stage = "Stage 3: Loop Closure"
        stage_markers.append((switch_time, "Loop Closure"))
        print(f"\n\n[MANUAL] Đã nhấn Enter! -> Stage 3 ({switch_time:.1f}s)")
    except EOFError: pass

def get_process_stats(node_names):
    stats = {}
    # Duyệt qua tất cả các process
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            p_name = proc.info['name']
            p_cmd = proc.info['cmdline']
            cmd_str = ' '.join(p_cmd) if p_cmd else ""

            for target in node_names:
                # Logic match process
                is_match = (target == p_name) or (target in cmd_str)
                
                # Loại trừ chính script này
                if is_match and "monitor_sim" not in cmd_str:
                    cpu = proc.cpu_percent(interval=None) 
                    mem = proc.memory_info().rss / (1024 * 1024) # MB
                    
                    if target not in stats: stats[target] = {'cpu': 0, 'ram': 0}
                    stats[target]['cpu'] += cpu
                    stats[target]['ram'] += mem
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return stats

def main():
    global start_time_global, current_stage
    
    # Init ROS 2
    rclpy.init()
    monitor_node = PerformanceMonitor()
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(monitor_node,), daemon=True)
    ros_thread.start()

    # Cấu hình tham số đầu vào
    parser = argparse.ArgumentParser()
    
    # --- ĐÃ SỬA: Loại bỏ 'gzserver' khỏi danh sách mặc định ---
    default_nodes = ['async_slam_toolbox_node', 'rviz2', 'robot_state_publisher']
    
    parser.add_argument('--nodes', nargs='+', default=default_nodes, 
                        help="List of nodes/processes to monitor")
    args = parser.parse_args()

    print(f"--- ROS 2 SLAM Performance Monitor (No Gazebo) ---")
    print(f"Monitoring Nodes: {args.nodes}")
    print("Status: Waiting... (Di chuyển robot để bắt đầu Stage 2, Nhấn ENTER để đánh dấu Stage 3)")

    input_thread = threading.Thread(target=manual_input_listener)
    input_thread.daemon = True
    input_thread.start()

    data_log = []
    start_time_global = time.time()
    stage_markers.append((0, "Start"))

    try:
        while rclpy.ok():
            current_time = time.time() - start_time_global
            node_stats = get_process_stats(args.nodes)
            
            row = {'Time': current_time}
            total_cpu = 0
            total_ram = 0

            for node in args.nodes:
                val = node_stats.get(node, {'cpu': 0, 'ram': 0})
                row[f'{node}_CPU'] = val['cpu']
                row[f'{node}_RAM'] = val['ram']
                total_cpu += val['cpu']
                total_ram += val['ram']

            row['Total_CPU'] = total_cpu
            row['Total_RAM'] = total_ram
            
            data_log.append(row)
            
            # In thông tin vắn tắt (Ưu tiên hiển thị SLAM CPU)
            slam_cpu = node_stats.get('async_slam_toolbox_node', {'cpu':0})['cpu']
            msg = f"\rT={current_time:03.0f}s | {current_stage} | SLAM CPU: {slam_cpu:4.1f}% | Total RAM: {total_ram:5.0f} MB      "
            sys.stdout.write(msg)
            sys.stdout.flush()
            
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopping & Saving data...")
    finally:
        monitor_node.destroy_node()
        rclpy.shutdown()

    if not data_log: return

    # --- SAVE CSV ---
    df = pd.DataFrame(data_log)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"slam_benchmark_no_sim_{timestamp}.csv"
    df.to_csv(csv_filename, index=False)
    
    # --- PLOTTING ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    tick_spacing = 10
    ax1.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
    ax2.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))

    def draw_markers(ax):
        trans = ax.get_xaxis_transform() 
        colors = ['#2ca02c', '#d62728', '#1f77b4'] 
        for i, (t, label) in enumerate(stage_markers):
            if t > df['Time'].max(): continue
            c = colors[i % len(colors)]
            ax.axvline(x=t, color=c, linestyle='--', alpha=0.7, linewidth=1.5)
            ax.text(t, -0.12, label, transform=trans, color=c, 
                    ha='center', va='top', fontweight='bold', fontsize=9, 
                    bbox=dict(facecolor='white', alpha=0.9, edgecolor='none'))

    # Plot 1: CPU
    for node in args.nodes:
        # Nếu là rviz2 thì vẽ nét đứt để dễ phân biệt với thuật toán chính
        linestyle = '--' if 'rviz' in node else '-'
        ax1.plot(df['Time'], df[f'{node}_CPU'], label=node, alpha=0.8, linewidth=1.5, linestyle=linestyle)
    
    draw_markers(ax1)
    ax1.set_ylabel('CPU Usage (%)')
    ax1.set_title(f'CPU Performance (SLAM Core Components)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Plot 2: RAM
    for node in args.nodes:
        ax2.plot(df['Time'], df[f'{node}_RAM'], label=node, linewidth=2)
    
    draw_markers(ax2)
    ax2.set_ylabel('RAM (MB)')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_title('Memory Usage')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    plt.subplots_adjust(bottom=0.15)
    plt.savefig(f"slam_benchmark_no_sim_{timestamp}.png")
    print(f"\nDone! Saved data to: {csv_filename}")
    print(f"Chart saved: slam_benchmark_no_sim_{timestamp}.png")
    plt.show()

if __name__ == "__main__":
    main()