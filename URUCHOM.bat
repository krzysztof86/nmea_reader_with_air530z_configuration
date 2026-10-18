@echo off
setlocal
cd /d "%~dp0"
py -3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_py
python -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_python
echo Nie znaleziono Pythona 3.10 lub nowszego.
echo Zainstaluj standardowy Python dla Windows z obsluga Tcl/Tk:
echo https://www.python.org/downloads/windows/
echo Nastepnie uruchom ponownie ten plik.
pause
exit /b 1
:run_py
py -3 "%~dp0nmea_reader.py"
if errorlevel 1 pause
exit /b
:run_python
python "%~dp0nmea_reader.py"
if errorlevel 1 pause
exit /b
