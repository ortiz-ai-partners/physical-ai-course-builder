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
if errorlevel 4 exit /b 1
if errorlevel 3 goto rear_left
if errorlevel 2 goto front
if errorlevel 1 goto back
exit /b 1
:back
set "pose_case=back"
goto launch
:front
set "pose_case=front"
goto launch
:rear_left
set "pose_case=rear-left"
:launch
echo Starting: %pose_case%
".venv\Scripts\python.exe" app.py --pose-practice %pose_case%
if errorlevel 1 pause
