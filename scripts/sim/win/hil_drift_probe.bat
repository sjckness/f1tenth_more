@echo off
rem Deriva di Isaac da fermo: un Isaac headless per variante (DDS su loopback, niente Orin), poi la tabella.
rem   hil_drift_probe.bat                          tutte le varianti, posa 16 del batch office
rem   hil_drift_probe.bat --variants base,hold_pos solo queste
rem   hil_drift_probe.bat --analyze DIR            rifa' solo l'analisi
rem Risultati: scripts\sim\win\drift_probe\<timestamp>\summary.md
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_drift_probe.py" %*
exit /b %ERRORLEVEL%
