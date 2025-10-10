#!/usr/bin/env python3
"""
Robot Network Control Client
Runs on x86 computers, sends network commands to Raspberry Pi via keyboard control

Supported platforms: Windows, Linux, macOS
Control keys:
W - Forward
S - Backward
A - Turn Left
D - Turn Right
Space - Stop
Q - Quit program

Author: AI Assistant
Date: January 2025
"""

import socket
import json
import time
import sys
import os
import threading
from pynput import keyboard
import tkinter as tk
from tkinter import ttk, messagebox
import queue
import yaml


class RobotControlClient:
    """Robot Network Control Client"""

    def __init__(self, network_config_file='network_control/network_config.yaml'):
        """
        Initialize the client

        Args:
            network_config_file: Network configuration file path
        """
        # Load client configuration from network config file
        self.network_config_file = network_config_file
        self.load_network_config()
        self.socket = None
        self.is_connected = False
        self.is_running = False

        # Control command mapping
        self.commands = {
            'w': 'forward',
            's': 'backward',
            'a': 'left',
            'd': 'right',
            'space': 'stop'
        }

        # Current status
        self.current_command = 'stop'
        self.last_command_time = 0

        # GUI related
        self.root = None
        self.status_label = None
        self.connection_label = None
        self.command_label = None
        self.log_text = None

        # Message queue
        self.message_queue = queue.Queue()

        # Keyboard listener
        self.keyboard_listener = None

    def load_network_config(self):
        """Load network configuration file"""
        try:
            with open(self.network_config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)

            client_config = config.get('client', {})
            self.server_ip = client_config.get('server_ip', '127.0.0.1')
            self.server_port = client_config.get('server_port', 8888)

            print(f"✓ Network config loaded successfully [{self.network_config_file}]: server_ip={self.server_ip}, server_port={self.server_port}")

        except FileNotFoundError as e:
            print(f"⚠️ Network config file not found [{self.network_config_file}], using default config: {e}")
            # Use default config
            self.server_ip = '127.0.0.1'
            self.server_port = 8888
        except yaml.YAMLError as e:
            print(f"⚠️ Network config file format error [{self.network_config_file}], using default config: {e}")
            # Use default config
            self.server_ip = '127.0.0.1'
            self.server_port = 8888
        except Exception as e:
            print(f"⚠️ Unknown error loading network config file [{self.network_config_file}], using default config: {e}")
            # Use default config
            self.server_ip = '127.0.0.1'
            self.server_port = 8888

    def connect_to_server(self):
        """Connect to server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # 5 second timeout
            self.socket.connect((self.server_ip, self.server_port))
            self.socket.settimeout(None)  # Cancel timeout after connection
            self.is_connected = True
            self.log_message("✓ Successfully connected to server")
            return True
        except socket.error as e:
            self.log_message(f"✗ Failed to connect to server: {e}")
            return False

    def disconnect_from_server(self):
        """Disconnect from server"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        self.is_connected = False
        self.log_message("Disconnected from server")

    def send_command(self, command_key):
        """Send control command to server"""
        if not self.is_connected:
            self.log_message("✗ Not connected to server")
            return False

        if command_key not in self.commands:
            self.log_message(f"✗ Invalid command: {command_key}")
            return False

        command = self.commands[command_key]

        try:
            # Construct message
            message = {
                'type': 'control',
                'command': command,
                'timestamp': time.time()
            }

            # Send message
            data = json.dumps(message).encode('utf-8')
            self.socket.sendall(data)

            # Update status
            self.current_command = command
            self.last_command_time = time.time()

            self.log_message(f"→ Sent command: {command}")
            return True

        except Exception as e:
            self.log_message(f"✗ Failed to send command: {e}")
            self.is_connected = False
            return False

    def on_key_press(self, key):
        """Handle key press events"""
        try:
            # Get key character
            if hasattr(key, 'char') and key.char:
                key_char = key.char.lower()
            elif key == keyboard.Key.space:
                key_char = 'space'
            elif key == keyboard.Key.esc:
                key_char = 'q'  # ESC key also exits
            else:
                return

            # Handle special keys
            if key_char == 'q':
                self.log_message("Exiting program...")
                self.is_running = False
                return False  # Stop listening

            # Send control command
            if key_char in self.commands:
                self.send_command(key_char)

        except Exception as e:
            self.log_message(f"Keyboard input error: {e}")

    def start_keyboard_listener(self):
        """Start keyboard listener"""
        try:
            self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
            self.keyboard_listener.start()
            self.log_message("✓ Keyboard listener started")
        except Exception as e:
            self.log_message(f"✗ Failed to start keyboard listener: {e}")
            return False
        return True

    def stop_keyboard_listener(self):
        """Stop keyboard listener"""
        if self.keyboard_listener:
            self.keyboard_listener.stop()
            self.keyboard_listener = None
            self.log_message("Keyboard listener stopped")

    def log_message(self, message):
        """Log message"""
        timestamp = time.strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}"

        # Add to queue, processed by GUI thread
        self.message_queue.put(full_message)

        # Also print to console
        print(full_message)

    def update_gui(self):
        """Update GUI status"""
        try:
            # Update connection status
            if self.connection_label:
                status = "Connected" if self.is_connected else "Disconnected"
                color = "green" if self.is_connected else "red"
                self.connection_label.config(text=f"Connection Status: {status}", foreground=color)

            # Update current command
            if self.command_label:
                self.command_label.config(text=f"Current Command: {self.current_command}")

            # Update status label
            if self.status_label:
                if self.is_running:
                    self.status_label.config(text="Running", foreground="green")
                else:
                    self.status_label.config(text="Stopped", foreground="red")

            # Process message queue
            while not self.message_queue.empty():
                message = self.message_queue.get_nowait()
                if self.log_text:
                    self.log_text.insert(tk.END, message + "\n")
                    self.log_text.see(tk.END)  # Auto scroll to bottom

        except Exception as e:
            print(f"GUI update error: {e}")

        # Update every 100ms
        if self.root and self.is_running:
            self.root.after(100, self.update_gui)

    def create_gui(self):
        """Create GUI interface"""
        self.root = tk.Tk()
        self.root.title("Robot Network Control Client")
        self.root.geometry("600x500")
        self.root.resizable(True, True)

        # Create main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(4, weight=1)

        # Title
        title_label = ttk.Label(main_frame, text="Robot Network Control Client",
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))

        # Server information
        server_info = f"Server: {self.server_ip}:{self.server_port}"
        server_label = ttk.Label(main_frame, text=server_info)
        server_label.grid(row=1, column=0, columnspan=2, pady=(0, 10))

        # Status display
        status_frame = ttk.LabelFrame(main_frame, text="Status Information", padding="5")
        status_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        status_frame.columnconfigure(1, weight=1)

        # Connection status
        ttk.Label(status_frame, text="Connection Status:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.connection_label = ttk.Label(status_frame, text="Disconnected", foreground="red")
        self.connection_label.grid(row=0, column=1, sticky=tk.W)

        # Running status
        ttk.Label(status_frame, text="Running Status:").grid(row=1, column=0, sticky=tk.W, padx=(0, 5))
        self.status_label = ttk.Label(status_frame, text="Not Started", foreground="red")
        self.status_label.grid(row=1, column=1, sticky=tk.W)

        # Current command
        ttk.Label(status_frame, text="Current Command:").grid(row=2, column=0, sticky=tk.W, padx=(0, 5))
        self.command_label = ttk.Label(status_frame, text="None")
        self.command_label.grid(row=2, column=1, sticky=tk.W)

        # Control instructions
        control_frame = ttk.LabelFrame(main_frame, text="Control Instructions", padding="5")
        control_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))

        control_text = """
Control Keys:
  W - Forward      S - Backward
  A - Turn Left    D - Turn Right
  Space - Stop     Q - Quit Program

Note: Robot will continue moving after each key press until next key press
        """
        control_label = ttk.Label(control_frame, text=control_text, justify=tk.LEFT)
        control_label.grid(row=0, column=0, sticky=tk.W)

        # Log display
        log_frame = ttk.LabelFrame(main_frame, text="Runtime Log", padding="5")
        log_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Create scrollbar
        scrollbar = ttk.Scrollbar(log_frame)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

        # Create text box
        self.log_text = tk.Text(log_frame, height=15, yscrollcommand=scrollbar.set,
                               font=("Consolas", 9))
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure scrollbar
        scrollbar.config(command=self.log_text.yview)

        # Configure text box weights
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

    def show_connection_dialog(self):
        """显示连接配置对话框"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Connection Configuration")
        dialog.geometry("300x150")
        dialog.transient(self.root)
        dialog.grab_set()

        # IP地址
        ttk.Label(dialog, text="Server IP:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        ip_entry = ttk.Entry(dialog)
        ip_entry.insert(0, self.server_ip)
        ip_entry.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))

        # 端口
        ttk.Label(dialog, text="Port:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        port_entry = ttk.Entry(dialog)
        port_entry.insert(0, str(self.server_port))
        port_entry.grid(row=1, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))

        def apply_settings():
            try:
                self.server_ip = ip_entry.get()
                self.server_port = int(port_entry.get())
                dialog.destroy()
                self.log_message(f"Connection configuration updated: {self.server_ip}:{self.server_port}")
            except ValueError:
                messagebox.showerror("Error", "Port must be a number")

        # 按钮
        button_frame = ttk.Frame(dialog)
        button_frame.grid(row=2, column=0, columnspan=2, pady=10)

        ttk.Button(button_frame, text="OK", command=apply_settings).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(side=tk.LEFT, padx=5)

        # 配置对话框
        dialog.columnconfigure(1, weight=1)

        # 等待对话框关闭
        self.root.wait_window(dialog)

    def start_client(self):
        """Start the client"""
        self.log_message("Robot network control client starting...")

        # 创建GUI
        self.create_gui()

        # 连接服务器
        if not self.connect_to_server():
            messagebox.showwarning("Connection Failed", "Unable to connect to server, please check network configuration")

        # 启动键盘监听
        if not self.start_keyboard_listener():
            messagebox.showerror("Error", "Unable to start keyboard listener")
            return

        self.is_running = True
        self.log_message("Client started, please use keyboard to control robot")

        # 启动GUI更新
        self.update_gui()

        # 运行GUI主循环
        self.root.mainloop()

    def close(self):
        """Close the client"""
        self.log_message("Closing client...")
        self.is_running = False
        self.stop_keyboard_listener()
        self.disconnect_from_server()

        if self.root:
            self.root.quit()


def main():
    """Main function"""
    print("Robot Network Control Client")
    print("=" * 40)

    # 创建客户端实例
    client = RobotControlClient()

    try:
        # 启动客户端
        client.start_client()

    except Exception as e:
        print(f"Program runtime error: {e}")
    finally:
        client.close()
        print("Client exited")


if __name__ == "__main__":
    main()
