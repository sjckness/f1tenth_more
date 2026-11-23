@echo off
rem Chiama /sim/reset (sim gia' avviato).
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_win.py" reset %*
exit /b %ERRORLEVEL%
