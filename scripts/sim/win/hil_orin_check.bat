@echo off
rem Solo lettura: ping, porta 22 e ssh verso l'Orin sul link diretto. Non cambia nulla.
rem Indirizzo e utente arrivano da sim\isaac\config\hil.local.yaml (o hil.yaml): link.orin_ip e orin.ssh.
setlocal
call "%~dp0hil_win_env.bat" || exit /b 1
for /f "usebackq delims=" %%i in (`""%HIL_PY%" "%~dp0..\hil_config.py" --get link.orin_ip"`) do set "ORIN_IP=%%i"
for /f "usebackq delims=" %%i in (`""%HIL_PY%" "%~dp0..\hil_config.py" --get orin.ssh"`) do set "ORIN_SSH=%%i"
echo [hil] Orin: %ORIN_SSH%  (%ORIN_IP%)
echo [hil] indirizzi IPv4 di questo PC:
ipconfig | findstr /c:"IPv4"
echo.
echo [hil] ping %ORIN_IP%
ping -n 2 -w 1500 %ORIN_IP%
echo.
powershell -NoProfile -Command "Test-NetConnection %ORIN_IP% -Port 22 -WarningAction SilentlyContinue | Select-Object ComputerName,TcpTestSucceeded"
echo [hil] ssh (nessuna password: BatchMode)
ssh -o BatchMode=yes -o ConnectTimeout=8 %ORIN_SSH% "hostname; ip -br addr; sysctl net.core.rmem_max; docker ps --format '{{.Names}} {{.Image}} {{.Status}}'"
exit /b %ERRORLEVEL%
