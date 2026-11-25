@echo off
rem Build Windows exe (needs: pip install pyinstaller -r requirements.txt)
setlocal
cd /d "%~dp0"
if not defined PYTHON set "PYTHON=python"
set "OUTPUT=dist"
rem A portable copy may hold recordings beside the executable.
if exist "dist\RawImuLive\Recordings\" set "OUTPUT=dist\release_%RANDOM%_%RANDOM%"
set "BUILD_EXCLUDES=--exclude-module IPython --exclude-module matplotlib --exclude-module scipy --exclude-module pytest --exclude-module PySide6 --exclude-module PyQt5 --exclude-module PyQt6"

echo Building folder distribution
"%PYTHON%" -m PyInstaller --noconfirm --onedir --windowed --name RawImuLive ^
    --paths "%CD%\.vendor" --add-data "%CD%\assets;assets" %BUILD_EXCLUDES% ^
    --hidden-import pyopengltk.win32 --hidden-import OpenGL.platform.win32 ^
    --distpath "%OUTPUT%" --workpath build --specpath build src\rawimu_live.py
if errorlevel 1 goto fail

if /i not "%~1"=="onefile" goto templates
echo Optional onefile build
"%PYTHON%" -m PyInstaller --noconfirm --onefile --windowed --name RawImuLive_onefile ^
    --paths "%CD%\.vendor" --add-data "%CD%\assets;assets" %BUILD_EXCLUDES% ^
    --hidden-import pyopengltk.win32 --hidden-import OpenGL.platform.win32 ^
    --distpath dist --workpath build --specpath build src\rawimu_live.py
if errorlevel 1 goto fail

:templates
echo copy config templates next to the exe ...
copy /y examples\imu_models.example.json "%OUTPUT%\RawImuLive\" >nul
copy /y examples\formats.example.json    "%OUTPUT%\RawImuLive\" >nul
copy /y LICENSE "%OUTPUT%\RawImuLive\" >nul
copy /y README.md "%OUTPUT%\RawImuLive\" >nul

echo.
echo Done: %OUTPUT%\RawImuLive\RawImuLive.exe   (onedir, double-click)
if /i "%~1"=="onefile" echo       dist\RawImuLive_onefile.exe      (single file)
goto :eof

:fail
echo Build failed - is pyinstaller installed?
exit /b 1
