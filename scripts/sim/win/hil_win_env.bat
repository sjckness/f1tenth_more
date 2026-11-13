@echo off
rem Ambiente nativo Windows per il sim HIL (Isaac Sim da pip, niente Docker).
rem Chiamato da tutti gli altri .bat. Per un prompt pronto: cmd /k hil_win_env.bat
rem Cambia ISAAC_ENV se l'ambiente non e' F:\envs\isaac. Solo variabili di questa sessione.
if not defined ISAAC_ENV set "ISAAC_ENV=F:\envs\isaac"
if not exist "%ISAAC_ENV%\Scripts\python.exe" (
  echo [hil] Manca %ISAAC_ENV%\Scripts\python.exe : imposta ISAAC_ENV sull'ambiente con isaacsim
  exit /b 1
)
call "%ISAAC_ENV%\Scripts\activate.bat"
set "OMNI_KIT_ACCEPT_EULA=YES"
set "PIP_CONFIG_FILE=nul"
set "PYTHONUTF8=1"
set "HIL_WIN=%~dp0"
set "HIL_PY=%ISAAC_ENV%\Scripts\python.exe"
exit /b 0
