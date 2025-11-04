# ekf_dashboard.py
# Web-based dashboard for EKF sensor fusion visualization
# Shows real-time state estimation, fusion outputs, success metrics, and EKF parameters

import threading
import json
import time
import os
from pathlib import Path
from typing import Dict, Optional

try:
    from flask import Flask, render_template
    from flask_socketio import SocketIO, emit
    HAS_FLASK = True
except ImportError:
    HAS_FLASK = False
    print("[EKF-DASH] Flask or Flask-SocketIO not installed")


class EKFDashboard:
    """
    Web dashboard for EKF sensor fusion visualization.
    Runs on a separate port from the main dashboard.
    """
    
    def __init__(self, port: int = 5001):
        """
        Initialize EKF dashboard.
        
        Args:
            port: HTTP server port (default 5001, separate from main dashboard)
        """
        if not HAS_FLASK:
            raise ImportError("Flask and Flask-SocketIO required. Install with: pip install flask flask-socketio")
        
        self.port = port
        self.app = Flask(__name__,
                        template_folder=str(Path(__file__).parent.parent / "templates"),
                        static_folder=str(Path(__file__).parent.parent / "static"))
        self.socketio = SocketIO(self.app, async_mode='threading', cors_allowed_origins="*")
        self.is_running = False
        self.server_thread = None
        
        # Data storage for history
        self.data_history = []
        self.max_history = 600  # 60 seconds at 10 Hz
        
        # Statistics
        self.stats = {
            "total_updates": 0,
            "rmse_position": 0.0,
            "rmse_velocity": 0.0,
            "rmse_attitude": 0.0,
            "max_error_position": 0.0,
            "max_error_velocity": 0.0,
            "max_error_attitude": 0.0
        }
        
        # Setup routes
        self._setup_routes()
    
    def _setup_routes(self):
        """Setup Flask routes."""
        
        @self.app.route('/')
        def index():
            return render_template('ekf_dashboard.html')
        
        @self.socketio.on('connect')
        def handle_connect():
            print(f"[EKF-DASH] Client connected")
            # Send current data if available
            if self.data_history:
                emit('ekf_data', self.data_history[-1])
    
    def start_server(self, run_in_thread=True):
        """Start the web server."""
        if self.is_running:
            print("[EKF-DASH] Server already running")
            return
        
        def run_server():
            try:
                print(f"[EKF-DASH] Starting EKF dashboard server on http://127.0.0.1:{self.port}")
                print(f"[EKF-DASH] Server starting... (this may take a few seconds)")
                self.is_running = True
                self.socketio.run(self.app,
                                host='127.0.0.1',
                                port=self.port,
                                debug=False,
                                use_reloader=False,
                                allow_unsafe_werkzeug=True,
                                log_output=True)
            except OSError as e:
                if "Address already in use" in str(e) or "WinError 10048" in str(e):
                    print(f"[EKF-DASH] ERROR: Port {self.port} is already in use!")
                    print(f"[EKF-DASH] Try changing EKF_DASHBOARD_PORT in main.py")
                else:
                    print(f"[EKF-DASH] ERROR starting server: {e}")
                self.is_running = False
            except Exception as e:
                print(f"[EKF-DASH] ERROR starting server: {type(e).__name__}: {e}")
                import traceback
                traceback.print_exc()
                self.is_running = False
        
        if run_in_thread:
            self.server_thread = threading.Thread(target=run_server, daemon=True)
            self.server_thread.start()
            print(f"[EKF-DASH] Waiting for server to initialize...")
            time.sleep(2)  # Initial wait
            
            import socket
            server_ready = False
            for i in range(10):  # Try for 5 seconds
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(0.5)
                    result = sock.connect_ex(('127.0.0.1', self.port))
                    sock.close()
                    if result == 0:
                        server_ready = True
                        print(f"[EKF-DASH] ✓ Server is ready! Open http://127.0.0.1:{self.port}")
                        break
                except Exception:
                    pass
                time.sleep(0.5)
            
            if not server_ready:
                print(f"[EKF-DASH] ⚠ Warning: Server may not be ready yet")
                print(f"[EKF-DASH] Check console for errors above")
                print(f"[EKF-DASH] Try opening http://127.0.0.1:{self.port} in a few seconds")
        else:
            run_server()
    
    def send_data(self, data: Dict):
        """
        Send EKF data to connected clients.
        
        Args:
            data: Dictionary containing:
                - t: time
                - true_state: true state dict
                - estimated_state: EKF estimated state dict
                - ekf_params: EKF parameters (covariance, gains, etc.)
                - sensor_measurements: raw sensor data
                - errors: estimation errors
        """
        if not self.is_running:
            return
        
        try:
            # Calculate statistics
            self._update_statistics(data)
            
            # Add to history
            self.data_history.append(data)
            if len(self.data_history) > self.max_history:
                self.data_history.pop(0)
            
            # Emit to all connected clients
            self.socketio.emit('ekf_data', data, namespace='/')
            
        except Exception as e:
            # Log error for debugging
            import traceback
            print(f"[EKF-DASH] Error sending data: {e}")
            traceback.print_exc()
    
    def _update_statistics(self, data: Dict):
        """Update performance statistics."""
        if 'errors' not in data:
            return
        
        errors = data['errors']
        self.stats["total_updates"] += 1
        
        # Position errors
        if 'position_error' in errors:
            pos_err = errors['position_error']
            self.stats["rmse_position"] = (
                (self.stats["rmse_position"] * (self.stats["total_updates"] - 1) + pos_err**2) / 
                self.stats["total_updates"]
            ) ** 0.5
            self.stats["max_error_position"] = max(self.stats["max_error_position"], pos_err)
        
        # Velocity errors
        if 'velocity_error' in errors:
            vel_err = errors['velocity_error']
            self.stats["rmse_velocity"] = (
                (self.stats["rmse_velocity"] * (self.stats["total_updates"] - 1) + vel_err**2) / 
                self.stats["total_updates"]
            ) ** 0.5
            self.stats["max_error_velocity"] = max(self.stats["max_error_velocity"], vel_err)
        
        # Attitude errors
        if 'attitude_error' in errors:
            att_err = errors['attitude_error']
            self.stats["rmse_attitude"] = (
                (self.stats["rmse_attitude"] * (self.stats["total_updates"] - 1) + att_err**2) / 
                self.stats["total_updates"]
            ) ** 0.5
            self.stats["max_error_attitude"] = max(self.stats["max_error_attitude"], att_err)
    
    def get_statistics(self) -> Dict:
        """Get current performance statistics."""
        return self.stats.copy()

