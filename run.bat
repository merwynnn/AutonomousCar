@echo off
setlocal

rem AWAKE simulator: start the FastAPI backend and the Vite dashboard.
rem Double-click from Windows. Requires WSL (Ubuntu) with uv and npm installed.
rem First run only: uv sync --extra dev, then npm install in dashboard\web.

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"

echo Starting AWAKE dashboard...
echo   backend: http://localhost:8000
echo   web:     http://localhost:5173

start "AWAKE backend" wsl.exe --cd "%ROOT%" bash -lc "uv run python -m dashboard.server"
start "AWAKE web" wsl.exe --cd "%ROOT%\dashboard\web" bash -lc "npm run dev"

rem Wait for Vite, then open the dashboard in the default browser.
start "" /min cmd /c "timeout /t 8 /nobreak >nul & start "" http://localhost:5173"

endlocal
