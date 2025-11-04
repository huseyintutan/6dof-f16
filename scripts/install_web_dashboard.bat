@echo off
title Install Web Dashboard Dependencies
echo.
echo ========================================
echo Installing Web Dashboard Dependencies
echo ========================================
echo.
echo Installing Flask and Flask-SocketIO...
echo.

pip install flask flask-socketio

if %ERRORLEVEL% EQU 0 (
    echo.
    echo [OK] Flask and Flask-SocketIO installed successfully!
    echo.
    echo You can now run the simulation with web dashboard.
    echo.
) else (
    echo.
    echo [ERROR] Installation failed!
    echo Make sure pip is available and you have internet connection.
    echo.
)

pause

