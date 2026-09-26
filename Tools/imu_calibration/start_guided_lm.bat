@echo off
setlocal
cd /d "%~dp0\..\.."
python Tools\imu_calibration\guided_lm_gui.py -o output\guided_lm
if errorlevel 1 pause
endlocal
