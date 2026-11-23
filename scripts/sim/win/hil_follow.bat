@echo off
rem Notte di campagna, lato workstation: esegue le richieste di scripts/sim/campaign/hil_night.sh
rem (Orin) e riavvia Isaac col layout che chiede. Lascialo aperto tutta la notte; Ctrl+C ferma lui
rem e Isaac. Al posto di hil_sim_link.bat: NON avviare Isaac a mano mentre gira (vedi hil_follow.py).
rem Serve la chiave ssh senza password verso orin.ssh. Tiene sveglio Windows finche' gira.
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_follow.py" %*
exit /b %ERRORLEVEL%
