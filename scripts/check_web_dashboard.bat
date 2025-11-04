@echo off
title Check Web Dashboard
echo.
echo ========================================
echo Web Dashboard Connection Test
echo ========================================
echo.

:: First check if Flask is installed
echo [1/2] Checking Flask installation...
python -c "import flask_socketio" 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Flask-SocketIO is NOT installed!
    echo.
    echo Solution:
    echo   Run: scripts\install_web_dashboard.bat
    echo   OR: pip install flask flask-socketio
    echo.
    pause
    exit /b 1
)

echo [OK] Flask-SocketIO is installed
echo.

:: Test if port is open
echo [2/2] Testing connection to http://127.0.0.1:5000...
echo.

powershell -Command "Test-NetConnection -ComputerName 127.0.0.1 -Port 5000 -InformationLevel Quiet" 2>nul
if %ERRORLEVEL% EQU 0 (
    echo [OK] Port 5000 is open - Dashboard is running!
    echo.
    echo Opening browser...
    timeout /t 1 /nobreak >nul
    start http://127.0.0.1:5000
) else (
    echo [ERROR] Port 5000 is not accessible
    echo.
    echo Possible reasons:
    echo   1. Simulation is not running
    echo   2. Server hasn't started yet (wait 5-10 seconds)
    echo   3. Port 5000 is blocked by firewall
    echo   4. Another application is using port 5000
    echo.
    echo Solution:
    echo   1. Make sure simulation is running (scripts\run_sim.bat)
    echo   2. Wait 10-15 seconds after starting simulation
    echo   3. Check console for [WEB] messages
    echo   4. Then try opening http://127.0.0.1:5000
)

echo.
pause

