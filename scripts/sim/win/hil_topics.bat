@echo off
rem Elenca i topic ROS che il sim pubblica (sim avviato in un'altra finestra).
rem   hil_topics.bat            modalita' local      hil_topics.bat --net link
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_win.py" topics %*
exit /b %ERRORLEVEL%
