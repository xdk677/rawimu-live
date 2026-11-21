@echo off
rem Build Windows exe (needs: pip install pyinstaller pyserial)
setlocal
cd /d "%~dp0"

echo [1/2] onedir build (recommended: no extraction, works under restrictive AV)
python -m PyInstaller --noconfirm --onedir --windowed --name RawImuLive ^
    --distpath dist --workpath build --specpath build src\rawimu_live.py
if errorlevel 1 goto fail

echo [2/2] onefile build (easy to copy; some AV blocks temp extraction)
python -m PyInstaller --noconfirm --onefile --windowed --name RawImuLive_onefile ^
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