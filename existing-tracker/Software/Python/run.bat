@echo off
title Camera Node Tracking – Simulation
echo ====================================================
echo   Distributed Camera Node Tracking  ^|  Simulation
echo ====================================================
echo.
echo [1/2] Starting ESP32 Node Simulator in new window...
start "ESP32 Simulator" cmd /k "cd /d "%~dp0" && python esp32_simulator.py"

echo Waiting for simulator to initialise...
timeout /t 2 /nobreak > nul

echo [2/2] Starting YOLO Tracker...
echo.
echo   Default source : webcam (camera index 0)
echo   To use a file  : run.bat --source "C:\path\to\video.mp4"
echo   Track persons  : run.bat --class-id 0
echo   Track cars     : run.bat --class-id 2
echo.
echo   Press Q in the tracker window to quit.
echo.

python YOLOProcessing.py %*

echo.
echo Tracker closed.  Close the Simulator window manually.
pause
