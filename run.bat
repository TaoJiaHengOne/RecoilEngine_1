@echo off
title Recoil Engine Launcher

rem --- 使用 %~dp0 确保我们总能从脚本所在位置找到数据目录 ---
set "RECOIL_DATA_DIR=%~dp0demos"
set "SPRING_WRITEDIR=%~dp0demos_cache"

echo =========================================================
echo Launching Recoil Engine...
echo Script Location: %~dp0
echo Data Directory : %RECOIL_DATA_DIR%
echo =========================================================

rem 检查数据目录是否存在
if not exist "%RECOIL_DATA_DIR%" (
    echo.
    echo ERROR: Data directory not found!
    echo Expected to find it at: %RECOIL_DATA_DIR%
    echo Please make sure you have created the 'my_game_data' folder.
    pause
    exit
)

rem 设置Spring/Recoil引擎的环境变量
set SPRING_DATADIR=%RECOIL_DATA_DIR%
set SPRING_WRITEDIR=%RECOIL_DATA_DIR%

rem --- 启动引擎 ---
rem 首先进入引擎程序所在的目录
cd build-windows/install

rem 启动
spring.exe 

rem --- 运行结束后返回 ---
cd ../../