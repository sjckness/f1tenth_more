@echo off
rem Frequenza di un topic:  hil_hz.bat /clock 10     (opzionale: --net link)
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_win.py" hz %*
exit /b %ERRORLEVEL%
