@echo off
rem Sim sul link HIL: profilo DDS sull'IP del link (link.workstation_ip) e Discovery Server sull'Orin.
rem Prima: hil_link_setup.bat add e hil_firewall.bat add (amministratore), poi hil_orin_check.bat.
call "%~dp0hil_win_env.bat" || exit /b 1
if exist "%HIL_WIN%run.log" move /y "%HIL_WIN%run.log" "%HIL_WIN%run_prev.log" >nul
echo [hil] log: %HIL_WIN%run.log
"%HIL_PY%" "%HIL_WIN%hil_win.py" sim --net link %* > "%HIL_WIN%run.log" 2>&1
echo [hil] uscita: %ERRORLEVEL%  (vedi %HIL_WIN%run.log)
exit /b %ERRORLEVEL%
