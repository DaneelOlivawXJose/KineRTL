@echo off
setlocal

:: Coger el tiempo del primer argumento, si está vacío usar 5000000ns
set STOP_TIME=%1
if "%STOP_TIME%"=="" set STOP_TIME=5000000ns

echo [1/3] Compilando archivos...
ghdl -a --std=08 ../../../robot_config_pkg.vhd || exit /b
ghdl -a --std=08 ../../../common/cordic/cordic.vhd || exit /b
ghdl -a --std=08 ../../forward/htm_pkg.vhd || exit /b
ghdl -a --std=08 ../../../common/matrix_ops/mult_m.vhd || exit /b
ghdl -a --std=08 ../../forward/direct_kinematics.vhd || exit /b
ghdl -a --std=08 numerical_ik.vhd || exit /b
ghdl -a --std=08 numerical_ik_tb.vhd || exit /b

echo [2/3] Elaborando...
ghdl -e --std=08 numerical_ik_tb || exit /b

echo [3/3] Simulando por %STOP_TIME%...
ghdl -r --std=08 numerical_ik_tb --fst=ondas_fst.fst --stop-time=%STOP_TIME%

echo Simulación terminada!