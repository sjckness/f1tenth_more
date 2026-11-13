@echo off
rem AMMINISTRATORE. Consente il DDS (UDP) in ingresso dall'Orin sul link, solo tra i due indirizzi.
rem   hil_firewall.bat show | add | remove
rem Windows scarta gli ingressi non richiesti: senza questa regola l'Orin non vede il sim (vedi le note di underwater_ws).
rem La regola vale per python.exe dell'ambiente Isaac. Non e' lanciato da Claude.
setlocal
set "WS_IP=10.42.60.1"
set "ORIN_IP=10.42.60.2"
if not defined ISAAC_ENV set "ISAAC_ENV=F:\envs\isaac"
set "RULE=HIL f1tenth Orin in ingresso (UDP)"
if /i "%~1"=="show" ( netsh advfirewall firewall show rule name="%RULE%" & exit /b 0 )
net session >nul 2>&1 || ( echo [hil] servono i privilegi di amministratore & exit /b 1 )
if /i "%~1"=="add" goto :add
if /i "%~1"=="remove" goto :remove
echo uso: %~nx0 show ^| add ^| remove
exit /b 2
:add
netsh advfirewall firewall add rule name="%RULE%" dir=in action=allow protocol=UDP localip=%WS_IP% remoteip=%ORIN_IP% profile=any
exit /b %ERRORLEVEL%
:remove
netsh advfirewall firewall delete rule name="%RULE%"
exit /b %ERRORLEVEL%
