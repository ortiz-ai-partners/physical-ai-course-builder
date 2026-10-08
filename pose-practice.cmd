@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Please run setup.cmd first.
  pause
  exit /b 1
)
echo 1: Back    2: Front    3: Rear-left
choice /c 123 /n /m "Choose a practice: "
set "pose_case=back"
if errorlevel 2 set "pose_case=front"
if errorlevel 3 set "pose_case=rear-left"
".venv\Scripts\python.exe" app.py --pose-practice %pose_case%
if errorlevel 1 pause
