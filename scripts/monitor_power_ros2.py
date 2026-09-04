#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Energy Monitoring Node for ROS 2 Dashing
Robot: 4-wheel Differential Drive with Jetson Nano, RPLiDAR S2E, IMU
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import pandas as pd
import matplotlib.pyplot as plt
import time
import sys
import os
import numpy as np

# =============================================================================
# 1. CẤU HÌNH PHẦN CỨNG (HARDWARE SPECIFICATION)
# Dựa trên datasheet thực tế và robot của bạn
# =============================================================================
HARDWARE = {
    # --- NHÓM TẢI TĨNH (STATIC LOAD - LUÔN TIÊU THỤ) ---
    'Jetson_Nano': 5.0,         # 10W Mode (Thực tế thường 8-10W khi chạy navigation)
    'RPLiDAR_S2E': 2.2,        # 5V @ 450mA (Scanning mode)
    'IMU_BNO055': 0.05,        # 3.3V @ 15mA
    'Camera': 2.5,             # Camera từ URDF (nếu không dùng thì set = 0)
    'STM32_MCU': 0.6,          # MCU điều khiển motor driver
    
    # --- NHÓM ĐỘNG HỌC (DYNAMICS) ---
    # Thông số này cần ĐIỀU CHỈNH dựa trên motor thực tế của bạn
    # Công thức: P_motion = k_v*|v| + k_w*|w| + k_a*|a|
    
    'k_v': 16.0,   # W/(m/s) - Ma sát lăn + tải động cơ khi di chuyển thẳng
    'k_w': 9.0,    # W/(rad/s) - Ma sát khi quay (angular velocity)
    'k_a': 14.0,   # W/(m/s²) - Quán tính khi gia tốc/phanh
    
    # Thông số bánh xe (từ URDF của bạn)
    'wheel_radius': 0.032,     # m (radius="0.032" trong URDF)
    'wheel_separation': 0.191, # m (separation="0.191" trong diff_drive)
}

# Tính tổng tải tĩnh
STATIC_LOAD = (HARDWARE['Jetson_Nano'] + 
               HARDWARE['RPLiDAR_S2E'] + 
               HARDWARE['IMU_BNO055'] + 
               HARDWARE['Camera'] + 
               HARDWARE['STM32_MCU'])

# =============================================================================
# 2. CLASS MONITOR - ROS 2 DASHING
# =============================================================================
class EnergyMonitorROS2(Node):
    def __init__(self):
        super().__init__('energy_monitor_ros2')
        
        # Time tracking
        self.start_time = time.time()
        self.last_time = self.start_time
        self.last_v = 0.0
        self.last_w = 0.0
        
        # Energy tracking
        self.total_energy = 0.0
        self.max_power = 0.0
        self.min_power = STATIC_LOAD
        self.is_moving = False
        
        # Data storage
        self.data_log = []
        self.stage_markers = []
        
        # Statistics
        self.distance_traveled = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.first_odom = True
        
        # ROS 2 Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/filtered',  # Dùng filtered odometry từ EKF
            self.odom_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info("="*60)
        self.get_logger().info("ENERGY MONITORING SYSTEM INITIALIZED")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Static Load: {STATIC_LOAD:.2f} W")
        self.get_logger().info(f"  - Jetson Nano: {HARDWARE['Jetson_Nano']:.1f} W")
        self.get_logger().info(f"  - RPLiDAR S2E: {HARDWARE['RPLiDAR_S2E']:.1f} W")
        self.get_logger().info(f"  - IMU: {HARDWARE['IMU_BNO055']:.2f} W")
        self.get_logger().info(f"  - Camera: {HARDWARE['Camera']:.1f} W")
        self.get_logger().info(f"  - MCU: {HARDWARE['STM32_MCU']:.1f} W")
        self.get_logger().info("="*60)
        self.get_logger().info("Press Ctrl+C to stop and generate report...")

    def cmd_vel_callback(self, msg):
        """Detect movement start"""
        threshold_linear = 0.01  # m/s
        threshold_angular = 0.01  # rad/s
        
        if not self.is_moving and (abs(msg.linear.x) > threshold_linear or 
                                   abs(msg.angular.z) > threshold_angular):
            self.is_moving = True
            t = time.time() - self.start_time
            self.stage_markers.append((t, "Movement Start"))
            self.get_logger().info(f"[MOTION DETECTED] at t={t:.1f}s")

    def odom_callback(self, msg):
        """Process odometry and calculate power consumption"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.001:  # Skip too-fast updates
            return

        # Extract velocities
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        
        # Calculate acceleration
        a_linear = abs(v - self.last_v) / dt
        a_angular = abs(w - self.last_w) / dt
        
        # Calculate distance traveled
        if not self.first_odom:
            dx = msg.pose.pose.position.x - self.last_x
            dy = msg.pose.pose.position.y - self.last_y
            self.distance_traveled += np.sqrt(dx**2 + dy**2)
        else:
            self.first_odom = False
        
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        
        # POWER CALCULATION
        # P_motion = k_v*|v| + k_w*|w| + k_a*|a_linear|
        p_motion = (abs(v) * HARDWARE['k_v'] + 
                   abs(w) * HARDWARE['k_w'] + 
                   a_linear * HARDWARE['k_a'])
        
        # Total power
        current_power = STATIC_LOAD + p_motion
        
        # Update statistics
        if current_power > self.max_power:
            self.max_power = current_power
        if current_power < self.min_power:
            self.min_power = current_power
        
        # Energy integration (trapezoidal rule)
        self.total_energy += current_power * dt
        
        # Log data
        self.data_log.append({
            'Time': current_time - self.start_time,
            'Power': current_power,
            'Power_Static': STATIC_LOAD,
            'Power_Motion': p_motion,
            'Energy': self.total_energy,
            'Velocity': v,
            'Angular_Vel': w,
            'Acceleration': a_linear,
            'Distance': self.distance_traveled
        })
        
        # Update last values
        self.last_v = v
        self.last_w = w
        self.last_time = current_time

    def generate_report(self):
        """Generate comprehensive energy report"""
        if not self.data_log:
            self.get_logger().warning("No data collected!")
            return

        df = pd.DataFrame(self.data_log)
        
        # Smooth data for better visualization
        window_size = min(20, len(df) // 10)
        if window_size < 2:
            window_size = 2
        df['Power_Smooth'] = df['Power'].rolling(window=window_size, min_periods=1).mean()
        
        # Setup output directory
        out_dir = os.path.expanduser('~/energy_reports_ros2')
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # Calculate statistics
        total_time = df['Time'].iloc[-1]
        avg_power = df['Power'].mean()
        energy_kwh = self.total_energy / 3600000  # Convert J to kWh
        
        # Create figure with 3 subplots
        fig = plt.figure(figsize=(14, 12))
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        
        # --- PLOT 1: Power vs Time ---
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(df['Time'], df['Power'], color='#cccccc', linewidth=0.5, 
                label='Raw Power', alpha=0.5)
        ax1.plot(df['Time'], df['Power_Smooth'], color='#d62728', linewidth=2, 
                label='Smoothed Power')
        ax1.axhline(y=STATIC_LOAD, color='blue', linestyle='--', alpha=0.7, 
                   label=f'Static Load ({STATIC_LOAD:.1f}W)')
        ax1.fill_between(df['Time'], STATIC_LOAD, df['Power_Smooth'], 
                        alpha=0.2, color='orange', label='Dynamic Power')
        
        # Add markers
        self.stage_markers.append((df['Time'].iloc[-1], "End"))
        for t, label in self.stage_markers:
            ax1.axvline(x=t, color='green', linestyle='--', alpha=0.5)
            ax1.text(t, ax1.get_ylim()[1]*0.95, f" {label}", 
                    color='green', fontweight='bold', fontsize=9)
        
        ax1.set_ylabel('Power (Watt)', fontweight='bold')
        ax1.set_title('Power Consumption Profile - ROS 2 Differential Drive Robot', 
                     fontweight='bold', fontsize=12)
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        
        # --- PLOT 2: Energy vs Time ---
        ax2 = fig.add_subplot(gs[1, :])
        ax2.fill_between(df['Time'], df['Energy'], color='#1f77b4', alpha=0.3)
        ax2.plot(df['Time'], df['Energy'], color='#1f77b4', linewidth=2)
        ax2.set_ylabel('Cumulative Energy (Joules)', fontweight='bold')
        ax2.set_xlabel('Time (seconds)', fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.set_title(f'Total Energy: {self.total_energy:.1f} J = {energy_kwh:.6f} kWh', 
                     fontweight='bold')
        
        # --- PLOT 3: Velocity Profile ---
        # ax3 = fig.add_subplot(gs[2, 0])
        # ax3.plot(df['Time'], df['Velocity'], color='green', linewidth=1.5)
        # ax3.set_ylabel('Linear Vel (m/s)', fontweight='bold')
        # ax3.set_xlabel('Time (s)', fontweight='bold')
        # ax3.grid(True, alpha=0.3)
        # ax3.set_title('Linear Velocity Profile')
        
        # --- PLOT 4: Power Distribution ---
        # ax4 = fig.add_subplot(gs[2, 1])
        # avg_static = STATIC_LOAD
        # avg_dynamic = df['Power_Motion'].mean()
        # labels = ['Static Load', 'Dynamic Load']
        # sizes = [avg_static, avg_dynamic]
        # colors = ['#ff9999', '#66b3ff']
        # ax4.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', 
        #        startangle=90)
        # ax4.set_title('Average Power Distribution')
        
        # Info text box
        info_text = (
            f"SYSTEM CONFIGURATION\n"
            f"{'='*40}\n"
            f"Static Load: {STATIC_LOAD:.1f} W\n"
            f"  • Jetson Nano: {HARDWARE['Jetson_Nano']:.1f} W\n"
            f"  • RPLiDAR S2E: {HARDWARE['RPLiDAR_S2E']:.1f} W\n"
            f"  • IMU: {HARDWARE['IMU_BNO055']:.2f} W\n"
            f"  • Camera: {HARDWARE['Camera']:.1f} W\n"
            f"\nPERFORMANCE METRICS\n"
            f"{'='*40}\n"
            f"Total Time: {total_time:.1f} s\n"
            f"Distance: {self.distance_traveled:.2f} m\n"
            f"Total Energy: {self.total_energy:.1f} J\n"
            f"Average Power: {avg_power:.1f} W\n"
            f"Peak Power: {self.max_power:.1f} W\n"
            f"Energy/Distance: {self.total_energy/max(self.distance_traveled, 0.01):.1f} J/m\n"
            f"\nDYNAMIC COEFFICIENTS\n"
            f"{'='*40}\n"
            f"k_v (velocity): {HARDWARE['k_v']:.1f} W/(m/s)\n"
            f"k_w (angular): {HARDWARE['k_w']:.1f} W/(rad/s)\n"
            f"k_a (accel): {HARDWARE['k_a']:.1f} W/(m/s²)"
        )
        
        plt.figtext(0.02, 0.02, info_text, fontsize=8, family='monospace',
                   bbox=dict(facecolor='white', alpha=0.95, boxstyle='round,pad=1'))
        
        # Save files
        img_path = os.path.join(out_dir, f"energy_report_{timestamp}.png")
        csv_path = os.path.join(out_dir, f"energy_data_{timestamp}.csv")
        
        plt.savefig(img_path, dpi=150, bbox_inches='tight')
        df.to_csv(csv_path, index=False)
        
        # Print summary
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("ENERGY REPORT GENERATED")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Report saved: {img_path}")
        self.get_logger().info(f"Data saved: {csv_path}")
        self.get_logger().info(f"\nTotal Energy: {self.total_energy:.1f} J ({energy_kwh:.6f} kWh)")
        self.get_logger().info(f"Total Distance: {self.distance_traveled:.2f} m")
        self.get_logger().info(f"Energy Efficiency: {self.total_energy/max(self.distance_traveled, 0.01):.1f} J/m")
        self.get_logger().info(f"Average Power: {avg_power:.1f} W")
        self.get_logger().info(f"Peak Power: {self.max_power:.1f} W")
        self.get_logger().info("="*60)
        
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = EnergyMonitorROS2()
    
    try:
        print("\n" + "="*60)
        print("ENERGY MONITORING STARTED")
        print("Press Ctrl+C to stop and generate report")
        print("="*60 + "\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nStopping monitoring and generating report...")
    finally:
        node.generate_report()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()