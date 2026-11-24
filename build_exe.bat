@echo off
rem Build Windows exe (needs: pip install pyinstaller -r requirements.txt)
setlocal
cd /d "%~dp0"
if not defined PYTHON set "PYTHON=python"
set "BUILD_EXCLUDES=--exclude-module IPython --exclude-module matplotlib --exclude-module scipy --exclude-module pytest --exclude-module PySide6 --exclude-module PyQt5 --exclude-module PyQt6"

echo [1/2] onedir build (recommended: no extraction, works under restrictive AV)
"%PYTHON%" -m PyInstaller --noconfirm --onedir --windowed --name RawImuLive ^
    --paths "%CD%\.vendor" --add-data "%CD%\assets;assets" %BUILD_EXCLUDES% ^
    --hidden-import pyopengltk.win32 --hidden-import OpenGL.platform.win32 ^
    --distpath dist --workpath build --specpath build src\rawimu_live.py
if errorlevel 1 goto fail

echo [2/2] onefile build (easy to copy; some AV blocks temp extraction)
"%PYTHON%" -m PyInstaller --noconfirm --onefile --windowed --name RawImuLive_onefile ^
    --paths "%CD%\.vendor" --add-data "%CD%\assets;assets" %BUILD_EXCLUDES% ^
    --hidden-import pyopengltk.win32 --hidden-import OpenGL.platform.win32 ^
    --distpath dist --workpath build --specpath build src\rawimu_live.py
if errorlevel 1 goto fail

echo copy config templates next to the exe ...
copy /y examples\imu_models.example.json dist\RawImuLive\ >nul
copy /y examples\formats.example.json    dist\RawImuLive\ >nul

echo.
echo Done: dist\RawImuLive\RawImuLive.exe   (onedir, double-click)
echo       dist\RawImuLive_onefile.exe      (single file)
goto :eof

:fail
echo Build failed - is pyinstaller installed?
exit /b 1
