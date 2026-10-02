@echo off
setlocal
cd /d "%~dp0"
py -3 tools\install_runtime.py
if errorlevel 1 exit /b 1
call package.cmd
if errorlevel 1 exit /b 1
call run.cmd --build
