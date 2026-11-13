@echo off
rem Il mondo office (SDF) -> report. Non serve Isaac ne' la GPU.
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_win.py" world --plan %*
exit /b %ERRORLEVEL%
