@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Please run setup.cmd first.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" app.py --plan examples\ortiz-gate-plan.json --assemble
if errorlevel 1 pause
