@echo off
rem Converte il mondo office in USD (UNA volta). Avvia Isaac headless: usa la GPU, niente altri carichi GPU.
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_win.py" world --convert %*
exit /b %ERRORLEVEL%
