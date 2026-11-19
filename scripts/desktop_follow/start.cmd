@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "runner=python"
if exist "%LOCALAPPDATA%\XiaozhiFollow\venv\Scripts\python.exe" set "runner=%LOCALAPPDATA%\XiaozhiFollow\venv\Scripts\python.exe"
"%runner%" -c "import cv2, flask, numpy; assert hasattr(cv2.aruco,'ArucoDetector') and hasattr(cv2,'FaceRecognizerSF')" 2>nul
if errorlevel 1 (
    python -m venv "%LOCALAPPDATA%\XiaozhiFollow\venv" || goto :failed
    "%LOCALAPPDATA%\XiaozhiFollow\venv\Scripts\python.exe" -m pip install -r requirements.txt || goto :failed
    set "runner=%LOCALAPPDATA%\XiaozhiFollow\venv\Scripts\python.exe"
)
"%runner%" models.py || goto :failed
"%runner%" app.py
if not errorlevel 1 exit /b 0
:failed
pause
exit /b 1
