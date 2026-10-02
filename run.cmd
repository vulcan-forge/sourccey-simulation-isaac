@echo off
setlocal
cd /d "%~dp0"
if not exist ".runtime\python.bat" (
    echo Isaac Sim is not installed. Run setup.cmd first.
    exit /b 1
)
set "PYTHONPATH=%~dp0"
set "PYTHONHOME="
set "OMNI_KIT_ACCEPT_EULA=YES"
call ".runtime\python.bat" "%~dp0run_isaac.py" %*
exit /b %errorlevel%
