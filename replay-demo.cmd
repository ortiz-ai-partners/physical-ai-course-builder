@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Please run setup.cmd first.
  pause
  exit /b 1
)
if "%~1"=="" (
  ".venv\Scripts\python.exe" pose_replay.py --view
) else (
  ".venv\Scripts\python.exe" pose_replay.py "%~1" --view
)
if errorlevel 1 pause
