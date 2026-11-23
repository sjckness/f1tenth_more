@echo off
rem Sim per Hydra nella distro WSL2 di QUESTO PC: nessun Orin.
rem Le etichette semantiche GT sono accese qui, perche' seg_source:=gt le richiede
rem (/sim/ground_truth/left/semantic). Ctrl+C per fermare.
rem Lato WSL, in un'altra finestra:
rem   cd ~/f1tenth_hil && source scripts/hydra/hydra_env.sh
rem   export FASTRTPS_DEFAULT_PROFILES_FILE="$(scripts/hydra/wsl_dds_profile.sh)"
rem   export ROS_DOMAIN_ID=44 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rem   ros2 launch f1tenth_hydra hydra.launch.py hydra_profile:=workstation_full ^
rem     seg_source:=gt pose_source:=isaac_gt use_sim_time:=true
rem Argomenti utili: --gui  --world room6x6  --rate 6  --dry-run
call "%~dp0hil_win_env.bat" || exit /b 1
if exist "%HIL_WIN%run.log" move /y "%HIL_WIN%run.log" "%HIL_WIN%run_prev.log" >nul
echo [hil] log: %HIL_WIN%run.log  (il primo /clock arriva dopo 20-70 s)
"%HIL_PY%" "%HIL_WIN%hil_win.py" sim --net wsl --set sensors.stereo.semantic.enabled=true %* > "%HIL_WIN%run.log" 2>&1
echo [hil] uscita: %ERRORLEVEL%  (vedi %HIL_WIN%run.log)
exit /b %ERRORLEVEL%
