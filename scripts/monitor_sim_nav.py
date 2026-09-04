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

# --- ROS 2 Imports ---
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SystemMonitorNode(Node):
    def __init__(self, target_nodes):
        super().__init__('performance_monitor_v7')
        
        # Parameters & State
        self.target_nodes = target_nodes
        self.current_stage = "Stage 1: Idle"
        self.stage_markers = []
        self.is_moving = False
        self.start_time_global = time.time()
        self.data_log = []
        
        # Initial Marker
        self.stage_markers.append((0, "Start"))

        # Subscribers
        # QoS profile 10 is standard for standard topics
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer for Monitoring Loop (1Hz)
        self.create_timer(1.0, self.monitor_callback)
        
        print(f"--- ROS 2 System Monitor V7 (Full Axis & Solid Lines) ---")
        print(f"Monitoring Nodes: {self.target_nodes}")
        print("Status: Waiting... (Press ENTER to finish Stage 3)")

    def cmd_vel_callback(self, msg):
        # Detect motion
        if not self.is_moving and (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01):
            self.is_moving = True
            switch_time = time.time() - self.start_time_global
            self.current_stage = "Stage 2: Moving"
            self.stage_markers.append((switch_time, "Start Moving"))
            # Print new line to not mess up the \r status line
            print(f"\n\n[AUTO] Motion Detected! -> Stage 2 ({switch_time:.1f}s)")

    def monitor_callback(self):
        current_time = time.time() - self.start_time_global
        node_stats = self.get_process_stats(self.target_nodes)
        
        row = {'Time': current_time}
        total_system_cpu = 0
        total_system_ram = 0

        for node in self.target_nodes:
            val = node_stats.get(node, {'cpu': 0, 'ram': 0})
            row[f'{node}_CPU'] = val['cpu']
            row[f'{node}_RAM'] = val['ram']
            total_system_cpu += val['cpu']
            total_system_ram += val['ram']

        row['Total_CPU'] = total_system_cpu
        row['Total_RAM'] = total_system_ram
        
        self.data_log.append(row)
        
        msg = f"\rT={current_time:03.0f}s | {self.current_stage} | TOTAL CPU: {total_system_cpu:5.1f}% | TOTAL RAM: {total_system_ram:5.0f} MB     "
        sys.stdout.write(msg)
        sys.stdout.flush()

    def trigger_stage_3(self):
        switch_time = time.time() - self.start_time_global
        self.current_stage = "Stage 3: Return"
        self.stage_markers.append((switch_time, "Stage 3"))
        print(f"\n\n[MANUAL] Type Enter! -> Stage 3 ({switch_time:.1f}s)")

    def get_process_stats(self, node_names):
        stats = {}
        # Iterate over all running processes
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                pinfo = proc.info
                cmdline = pinfo['cmdline']
                
                if cmdline:
                    # Join cmdline to string for searching
                    cmd_str = ' '.join(cmdline)
                    
                    for target_node in node_names:
                        # Logic tìm kiếm Node trong ROS 2:
                        # 1. Tìm thấy __node:=<name> (khi dùng launch remapping)
                        # 2. Hoặc tên node nằm trong tên executable (ví dụ: ros2 run pkg <node>)
                        # 3. Hoặc tên node nằm trong cmdline arguments
                        
                        is_match = False
                        if f"__node:={target_node}" in cmd_str:
                            is_match = True
                        elif target_node in pinfo['name']: # Check process name matches
                            is_match = True
                        elif target_node in cmd_str and "python" not in pinfo['name']: # Binary executable match
                             is_match = True
                        
                        if is_match:
                            cpu = proc.cpu_percent(interval=None) 
                            mem = proc.memory_info().rss / (1024 * 1024) # MB
                            
                            if target_node not in stats: stats[target_node] = {'cpu': 0, 'ram': 0}
                            stats[target_node]['cpu'] += cpu
                            stats[target_node]['ram'] += mem
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return stats

def manual_input_listener(monitor_node):
    """Thread lắng nghe phím Enter"""
    try:
        if sys.version_info[0] < 3: raw_input()
        else: input()
        
        # Gọi hàm update stage của node
        monitor_node.trigger_stage_3()
    except EOFError: pass

def main():
    # Initialize ROS 2
    rclpy.init()
    
    # Parse Arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--nodes', nargs='+', required=True, help="List of nodes to monitor")
    args = parser.parse_args()

    # Create Node
    monitor_node = SystemMonitorNode(args.nodes)

    # Start Input Thread
    input_thread = threading.Thread(target=manual_input_listener, args=(monitor_node,))
    input_thread.daemon = True
    input_thread.start()

    try:
        # Spin để xử lý callbacks (Timer & Subscription)
        rclpy.spin(monitor_node)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Clean up
        monitor_node.destroy_node()
        rclpy.shutdown()
        process_data_and_plot(monitor_node.data_log, monitor_node.stage_markers, args.nodes)

def process_data_and_plot(data_log, stage_markers, nodes):
    if not data_log:
        print("No data collected.")
        return

    # --- SAVE DATA ---
    df = pd.DataFrame(data_log)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"benchmark_nav_{timestamp}.csv"
    df.to_csv(csv_filename, index=False)
    print(f"Data saved to {csv_filename}")

    # --- SMOOTHING ---
    window_size = 4
    for node in nodes:
        if f'{node}_CPU' in df.columns:
            df[f'{node}_CPU_Smooth'] = df[f'{node}_CPU'].rolling(window=window_size, min_periods=1).mean()
            df[f'{node}_RAM_Smooth'] = df[f'{node}_RAM'].rolling(window=window_size, min_periods=1).mean()
    
    # Calculate Smooth Total CPU & RAM
    df['Total_CPU_Smooth'] = df['Total_CPU'].rolling(window=window_size, min_periods=1).mean()
    
    # --- [MODIFIED] Added Total RAM Smoothing ---
    if 'Total_RAM' in df.columns:
        df['Total_RAM_Smooth'] = df['Total_RAM'].rolling(window=window_size, min_periods=1).mean()

    # --- PLOTTING ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    # X-Axis Settings
    tick_spacing = 10
    ax1.tick_params(labelbottom=True) 
    ax2.tick_params(labelbottom=True)
    ax1.xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
    ax1.yaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
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

    # --- Plot 1: CPU ---
    for node in nodes:
        if f'{node}_CPU_Smooth' in df.columns:
            ax1.plot(df['Time'], df[f'{node}_CPU_Smooth'], label=node, alpha=0.7, linewidth=1.5, linestyle='-')
    
    ax1.plot(df['Time'], df['Total_CPU_Smooth'], label='TOTAL', color='black', linewidth=2.5)
    
    draw_markers(ax1)
    # Handle Y-limit safely
    max_cpu_val = df['Total_CPU_Smooth'].max() if not df.empty else 0
    top_ylim = max(105, max_cpu_val + 10)
    ax1.set_ylim(0, top_ylim)
    
    ax1.set_ylabel('CPU Usage (%)')
    ax1.set_title(f'CPU Performance (ROS 2)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3, which='both')

    # --- Plot 2: RAM ---
    for node in nodes:
        if f'{node}_RAM_Smooth' in df.columns:
            ax2.plot(df['Time'], df[f'{node}_RAM_Smooth'], label=node, linewidth=2, linestyle='-')
    
    # --- [MODIFIED] Plot Total RAM ---
    if 'Total_RAM_Smooth' in df.columns:
        ax2.plot(df['Time'], df['Total_RAM_Smooth'], label='TOTAL', color='black', linewidth=2.5, linestyle='-')
    
    draw_markers(ax2)
    ax2.set_ylim(bottom=0)
    
    ax2.set_ylabel('RAM (MB)')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_title('Memory Usage (ROS 2)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3, which='both')
    plt.subplots_adjust(left=0.06, right=0.95, top=0.95, bottom=0.12, hspace=0.25)
    
    png_filename = f"benchmark_nav_{timestamp}.png"
    plt.savefig(png_filename)
    print(f"\nDone! Chart saved at: {png_filename}")
    # plt.show() # Uncomment if you have a display connected to Jetson, otherwise keep commented

if __name__ == "__main__":
    main()