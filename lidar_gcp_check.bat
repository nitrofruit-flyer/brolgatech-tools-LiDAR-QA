@echo off
rem Double-click to open the LiDAR check window.
python "%~dp0lidar_gcp_check.py"
if errorlevel 1 pause
