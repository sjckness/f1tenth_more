@echo off
rem Controlli senza avviare nulla. Con l'Orin collegato:  hil_preflight.bat --orin
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_win.py" preflight %*
exit /b %ERRORLEVEL%
