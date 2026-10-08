@echo off
setlocal
where py >nul 2>nul
if errorlevel 1 (
    echo Python was not found. Install Python 3.10 or newer from python.org.
    pause
    exit /b 1
)
py modbus_tcp_client.py
