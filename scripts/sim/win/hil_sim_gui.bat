@echo off
rem Come hil_sim_local.bat ma con la finestra di Isaac (loopback, nessun Orin).
call "%~dp0hil_win_env.bat" || exit /b 1
if exist "%HIL_WIN%run.log" move /y "%HIL_WIN%run.log" "%HIL_WIN%run_prev.log" >nul
echo [hil] log: %HIL_WIN%run.log
"%HIL_PY%" "%HIL_WIN%hil_win.py" sim --net local --gui %* > "%HIL_WIN%run.log" 2>&1
echo [hil] uscita: %ERRORLEVEL%  (vedi %HIL_WIN%run.log)
exit /b %ERRORLEVEL%
