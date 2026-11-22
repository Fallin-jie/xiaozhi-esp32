@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "venv=%LOCALAPPDATA%\XiaozhiFollow\venv"
set "runner=%venv%\Scripts\python.exe"
if not exist "%runner%" (
    where py >nul 2>nul
    if errorlevel 1 (
        python -m venv "%venv%" || goto :failed
    ) else (
        py -3 -m venv "%venv%" || goto :failed
    )
)
"%runner%" -c "import cv2, flask, numpy, pyrealsense2 as rs; assert flask.Request.max_content_length.fset and hasattr(cv2.aruco,'ArucoDetector') and hasattr(cv2,'FaceRecognizerSF') and hasattr(rs,'align')" 2>nul
if errorlevel 1 (
    "%runner%" -m pip install -r requirements.txt || goto :failed
)
"%runner%" models.py || goto :failed
"%runner%" app.py
if not errorlevel 1 exit /b 0
:failed
pause
exit /b 1
