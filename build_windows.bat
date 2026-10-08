@echo off
setlocal
title Build Modbus TCP Coil Client

where py >nul 2>nul
if errorlevel 1 (
    echo Python was not found. Install Python 3.10 or newer from python.org.
    echo During installation, select "Add Python to PATH".
    pause
    exit /b 1
)

echo Installing/updating PyInstaller...
py -m pip install --upgrade pyinstaller
if errorlevel 1 goto :error

echo Running protocol tests...
py -m unittest -v test_modbus_tcp_client.py
if errorlevel 1 goto :error

echo Building Windows executable...
py -m PyInstaller --noconfirm --clean --onefile --windowed ^
    --name ModbusTCP-Coil-Client modbus_tcp_client.py
if errorlevel 1 goto :error

echo.
echo Build complete: dist\ModbusTCP-Coil-Client.exe
pause
exit /b 0

:error
echo.
echo Build failed. Review the error messages above.
pause
exit /b 1
