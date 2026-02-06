@echo off
set "SCRIPTS=%LOCALAPPDATA%\Packages\PythonSoftwareFoundation.Python.3.13_qbz5n2kfra8p0\LocalCache\local-packages\Python313\Scripts"
set "MBED_EXE=%SCRIPTS%\mbed-tools.exe"
if not exist "%MBED_EXE%" (
    echo mbed-tools.exe not found at: %MBED_EXE%
    exit /b 1
)
rem Prepend Scripts to PATH so Ninja (and other tools) are found during compile
set "PATH=%SCRIPTS%;%PATH%"
"%MBED_EXE%" %*
