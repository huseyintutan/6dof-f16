# web_dashboard.py
# -------------------------------------------------------------
# Web-based dashboard for F-16 simulation using Flask-SocketIO
# 
# Provides real-time data visualization via WebSocket
# Much lower CPU usage than matplotlib (~5-15% vs 40-60%)
# -------------------------------------------------------------

import json
import threading
import time
import os
from pathlib import Path

try:
    from flask import Flask, render_template
    from flask_socketio import SocketIO, emit
    HAS_FLASK = True
except ImportError as e:
    HAS_FLASK = False
    print(f"[WEB] Flask or Flask-SocketIO not installed: {e}")
    print("[WEB] Install with: pip install flask flask-socketio")

class WebDashboard:
    """Web-based dashboard using Flask-SocketIO for real-time updates."""
    
    def __init__(self, port=5000, enable_cors=True):
        """
        Initialize web dashboard server.
        
        Args:
            port: HTTP server port (default: 5000)
            enable_cors: Enable CORS for cross-origin requests
        """
        if not HAS_FLASK:
            raise ImportError("Flask and Flask-SocketIO required. Install with: pip install flask flask-socketio")
        
        self.port = port
        
        # Get template folder path (relative to project root)
        script_dir = Path(__file__).parent.parent
        template_dir = script_dir / 'templates'
        static_dir = script_dir / 'static'
        
        # Create directories if they don't exist
        template_dir.mkdir(exist_ok=True)
        static_dir.mkdir(exist_ok=True)
        
        self.app = Flask(__name__, 
                        template_folder=str(template_dir),
                        static_folder=str(static_dir))
        self.app.config['SECRET_KEY'] = 'f16_sim_secret_key'
        
        # Initialize SocketIO with async_mode='threading' for better performance
        self.socketio = SocketIO(self.app, 
                                cors_allowed_origins="*" if enable_cors else None,
                                async_mode='threading',
                                logger=False,
                                engineio_logger=False)
        
        self.server_thread = None
        self.is_running = False
        self.clients_connected = 0
        
        # Setup routes
        self._setup_routes()
        
        print(f"[WEB] Web dashboard initialized (port: {port})")
        print(f"[WEB] Template folder: {template_dir}")
    
    def _setup_routes(self):
        """Setup Flask routes."""
        @self.app.route('/')
        def index():
            return render_template('dashboard.html')
        
        @self.socketio.on('connect')
        def handle_connect():
            self.clients_connected += 1
            print(f"[WEB] Client connected ({self.clients_connected} total)")
            emit('status', {'message': 'Connected to F-16 simulation'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.clients_connected -= 1
            print(f"[WEB] Client disconnected ({self.clients_connected} total)")
    
    def start_server(self, run_in_thread=True):
        """
        Start the web server.
        
        Args:
            run_in_thread: If True, run server in background thread
        """
        if self.is_running:
            print("[WEB] Server already running")
            return
        
        def run_server():
            try:
                print(f"[WEB] Starting web server on http://127.0.0.1:{self.port}")
                print(f"[WEB] Server starting... (this may take a few seconds)")
                self.is_running = True
                # Run server (this will block until server stops)
                self.socketio.run(self.app, 
                                host='127.0.0.1',
                                port=self.port,
                                debug=False,
                                use_reloader=False,
                                allow_unsafe_werkzeug=True,
                                log_output=True)  # Enable logging to see errors
            except OSError as e:
                if "Address already in use" in str(e) or "WinError 10048" in str(e):
                    print(f"[WEB] ERROR: Port {self.port} is already in use!")
                    print(f"[WEB] Try changing WEB_DASHBOARD_PORT in main.py")
                else:
                    print(f"[WEB] ERROR starting server: {e}")
                self.is_running = False
            except Exception as e:
                print(f"[WEB] ERROR starting server: {type(e).__name__}: {e}")
                import traceback
                traceback.print_exc()
                self.is_running = False
        
        if run_in_thread:
            self.server_thread = threading.Thread(target=run_server, daemon=True)
            self.server_thread.start()
            # Give server more time to start - check if it's actually running
            print(f"[WEB] Waiting for server to initialize...")
            time.sleep(2)  # Initial wait
            
            # Check if server is responding
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
                        print(f"[WEB] ✓ Server is ready! Open http://127.0.0.1:{self.port}")
                        break
                except Exception:
                    pass
                time.sleep(0.5)
            
            if not server_ready:
                print(f"[WEB] ⚠ Warning: Server may not be ready yet")
                print(f"[WEB] Check console for errors above")
                print(f"[WEB] Try opening http://127.0.0.1:{self.port} in a few seconds")
        else:
            run_server()
    
    def stop_server(self):
        """Stop the web server."""
        if self.is_running:
            self.is_running = False
            # Note: Flask-SocketIO doesn't have easy stop method
            # Server will stop when main process exits
            print("[WEB] Server shutdown requested")
    
    def send_data(self, data_dict):
        """
        Send simulation data to connected clients.
        
        Args:
            data_dict: Dictionary containing simulation state and sensor data
        """
        if self.is_running and self.clients_connected > 0:
            try:
                # Emit data to all connected clients
                self.socketio.emit('sim_data', data_dict)
            except Exception as e:
                # Silently fail if no clients connected
                pass
    
    def get_client_count(self):
        """Get number of connected clients."""
        return self.clients_connected

