@echo off
rem Sim da solo, headless, DDS solo su loopback: NESSUN Orin necessario. Ctrl+C per fermare.
rem Argomenti utili: --world room6x6  --layout corridor  --rate 6  --set sensors.stereo.width=320  --dry-run
call "%~dp0hil_win_env.bat" || exit /b 1
if exist "%HIL_WIN%run.log" move /y "%HIL_WIN%run.log" "%HIL_WIN%run_prev.log" >nul
echo [hil] log: %HIL_WIN%run.log  (il primo /clock arriva dopo 20-70 s)
"%HIL_PY%" "%HIL_WIN%hil_win.py" sim --net local %* > "%HIL_WIN%run.log" 2>&1
echo [hil] uscita: %ERRORLEVEL%  (vedi %HIL_WIN%run.log)
exit /b %ERRORLEVEL%
