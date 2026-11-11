@echo off
rem Starts the bridge for a sender on this PC (gSender: Ethernet, 127.0.0.1, port 23).
rem The window stays open afterwards, so that an error message can be read.
title GRBL2MACH
cd /d "%~dp0"
echo GRBL2MACH: close Mach3 first. Then connect your sender to 127.0.0.1, port 23.
echo Close this window to stop the bridge.
echo.
grbl2mach.exe --no-gui -u -p 23 -n
echo.
echo GRBL2MACH has stopped (exit code %errorlevel%).
pause
