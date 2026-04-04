@echo off
echo ================================
echo    AI Grand Prix — Auto Launch
echo ================================

echo [1/2] Starting Colosseum Simulator...
start "" "C:\Program Files\Epic Games\UE_5.4\Engine\Binaries\Win64\UnrealEditor.exe" "E:\source\repos\Colosseum\Unreal\Environments\BlocksV2\BlocksV2.uproject" -game -windowed -resx=1280 -resy=720

echo Waiting for simulator to load (30 seconds)...
timeout /t 30 /nobreak

echo [2/2] Starting Python autonomy stack...
call venv\Scripts\activate
python comms/telemetry_test.py

echo Done.
pause