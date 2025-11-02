@echo off
title Run Python Sim

:: === YOL ===
set "SIMDIR=%~dp0.."

pushd "%SIMDIR%"
where python >nul 2>nul
if errorlevel 1 (
  echo Python bulunamadi. PATH'e ekleyin veya Python yukleyin.
  pause
  exit /b 1
)

python main.py
set ERR=%ERRORLEVEL%
popd

if %ERR% NEQ 0 (
  echo Sim hata kodu: %ERR%
) else (
  echo Sim bitti.
)
pause
