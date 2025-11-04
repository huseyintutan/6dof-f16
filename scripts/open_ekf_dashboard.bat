@echo off
title Open EKF Dashboard
echo.
echo ========================================
echo EKF Sensor Fusion Dashboard
echo ========================================
echo.
echo Opening EKF dashboard in default browser...
echo URL: http://localhost:5001
echo.
echo Note: Make sure simulation is running first!
echo.

:: Wait a moment for server to start (if just started)
timeout /t 2 /nobreak >nul

:: Open default browser
start http://localhost:5001

echo.
echo Dashboard should open in your browser.
echo If simulation is not running, start it first with run_sim.bat
echo.
pause

