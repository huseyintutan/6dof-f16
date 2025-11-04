@echo off
title Start FlightGear (external FDM)

:: === PATHS ===
set "FGEXE=C:\Program Files\FlightGear 2024.1\bin\fgfs.exe"
set "FGAIR=C:\Users\HUSEYIN\FlightGear\Downloads\Aircraft\org.flightgear.fgaddon.stable_2024\Aircraft"
set "AIRCRAFT=f16-block-52"

:: Note: If myproto.xml is in fgdata\Protocols, just "myproto" name is enough.
:: Open in windowed mode so terminal remains visible.
start "" "%FGEXE%" ^
  --fdm=external ^
  --fg-aircraft="%FGAIR%" ^
  --aircraft=%AIRCRAFT% ^
  --generic="socket,in,60,127.0.0.1,5500,udp,myproto" ^
  --timeofday=noon ^
  --disable-fullscreen ^
  --geometry=1600x900

:: Optionally open HTTP server for readiness check:
::  --httpd=5400
