@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Please run setup.cmd first.
  pause
  exit /b 1
)
echo 1: Wide hairpin    2: Narrow reversing turn
choice /c 12 /n /m "Choose a course: "
if errorlevel 3 exit /b 1
if errorlevel 2 goto narrow
if errorlevel 1 goto wide
exit /b 1
:wide
set "course_case=wide"
goto launch
:narrow
set "course_case=narrow"
:launch
".venv\Scripts\python.exe" rs_course.py --sport --case %course_case%
if errorlevel 1 pause
