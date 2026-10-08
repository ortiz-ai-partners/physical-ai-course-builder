@echo off
setlocal
cd /d "%~dp0"
if not exist ".cache\tmp" mkdir ".cache\tmp"
set "TEMP=%CD%\.cache\tmp"
set "TMP=%CD%\.cache\tmp"
python -m venv .venv
if errorlevel 1 goto failed
".venv\Scripts\python.exe" -m pip install -r requirements-lock.txt
if errorlevel 1 goto failed
echo Setup complete. Open start.cmd.
pause
exit /b 0
:failed
echo Setup failed. Please keep this message for troubleshooting.
pause
exit /b 1
