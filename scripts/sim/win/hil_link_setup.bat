@echo off
rem AMMINISTRATORE. Assegna/toglie l'IP del link HIL (10.42.60.1/24) sull'adattatore verso l'Orin.
rem   hil_link_setup.bat show
rem   hil_link_setup.bat add [NOME_ADATTATORE]     (default: Ethernet)
rem   hil_link_setup.bat remove [NOME_ADATTATORE]  (torna a DHCP/automatico)
rem Prima di modificare salva ipconfig /all e route print in %TEMP%\hil_win\. Non e' lanciato da Claude.
setlocal
set "WS_IP=10.42.60.1"
set "IFACE=%~2"
if "%IFACE%"=="" set "IFACE=Ethernet"
if /i "%~1"=="show" ( netsh interface ip show address & exit /b 0 )
net session >nul 2>&1 || ( echo [hil] servono i privilegi di amministratore & exit /b 1 )
if /i "%~1"=="add" goto :add
if /i "%~1"=="remove" goto :remove
echo uso: %~nx0 show ^| add [adattatore] ^| remove [adattatore]
exit /b 2
:add
mkdir "%TEMP%\hil_win" 2>nul
ipconfig /all > "%TEMP%\hil_win\rete_prima.txt"
route print >> "%TEMP%\hil_win\rete_prima.txt"
echo [hil] stato salvato in %TEMP%\hil_win\rete_prima.txt
netsh interface ip set address name="%IFACE%" static %WS_IP% 255.255.255.0
exit /b %ERRORLEVEL%
:remove
netsh interface ip set address name="%IFACE%" dhcp
exit /b %ERRORLEVEL%
