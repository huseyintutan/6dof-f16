@echo off
title Start FlightGear (external FDM)

:: === YOLLAR ===
set "FGEXE=C:\Program Files\FlightGear 2024.1\bin\fgfs.exe"
set "FGAIR=C:\Users\HUSEYIN\FlightGear\Downloads\Aircraft\org.flightgear.fgaddon.stable_2024\Aircraft"
set "AIRCRAFT=f16-block-52"

:: Not: myproto.xml, fgdata\Protocols altındaysa sadece "myproto" adı yeterli.
:: Ekran pencere modunda açılsın ki terminal görünür kalsın.
start "" "%FGEXE%" ^
  --fdm=external ^
  --fg-aircraft="%FGAIR%" ^
  --aircraft=%AIRCRAFT% ^
  --generic="socket,in,60,127.0.0.1,5500,udp,myproto" ^
  --timeofday=noon ^
  --disable-fullscreen ^
  --geometry=1600x900

:: İstersen hazır olma kontrolü için HTTP sunucu aç:
::  --httpd=5400
