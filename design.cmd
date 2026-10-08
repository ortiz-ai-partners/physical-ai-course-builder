@echo off
setlocal
cd /d "%~dp0"
set "SIM_PY=%~dp0.venv\Scripts\python.exe"
if not exist "%SIM_PY%" (
  echo Please run setup.cmd first.
  pause
  exit /b 1
)
"%SIM_PY%" designer.py
if errorlevel 1 pause
