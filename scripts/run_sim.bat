@echo off
title Run Python Sim

:: === PATHS ===
set "SIMDIR=%~dp0.."

pushd "%SIMDIR%"
where python >nul 2>nul
if errorlevel 1 (
  echo Python not found. Add to PATH or install Python.
  pause
  exit /b 1
)

python main.py
set ERR=%ERRORLEVEL%
popd

if %ERR% NEQ 0 (
  echo Simulation error code: %ERR%
) else (
  echo Simulation finished.
)
pause
