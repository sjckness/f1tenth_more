@echo off
rem Batch HIL office: tutte le pose fattibili x 3 planner x 3 ripetizioni, poi l'analisi.
rem Isaac gira qui (un lancio per posa), lo stack sull'Orin via ssh (car-hil). Ctrl+C chiude tutto.
rem   hil_office_batch.bat --smoke            una posa, un modo, una ripetizione (prova della catena)
rem   hil_office_batch.bat                    il batch intero (scripts\sim\office_batch\batch.yaml)
rem   hil_office_batch.bat --session SID      riprende una sessione interrotta
rem Log: scripts\sim\win\batch\<sessione>\  -  risultati: Orin ~/f1tenth_hil_batch/<sessione>/
call "%~dp0hil_win_env.bat" || exit /b 1
"%HIL_PY%" "%HIL_WIN%hil_office_batch.py" %*
exit /b %ERRORLEVEL%
