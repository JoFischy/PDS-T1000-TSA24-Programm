@echo off
echo Building PDS-T1000 Image Manager...

REM Saubere Kompilierung mit g++ und raylib
g++ -std=c++17 -O2 -Wall -I include -I external/raylib/src src/main.cpp src/image_manager.cpp -o PDS-T1000.exe -L external/raylib/lib -lraylib -lopengl32 -lgdi32 -lwinmm

if exist "PDS-T1000.exe" (
    echo Build erfolgreich!
    echo Starte Programm...
    PDS-T1000.exe
) else (
    echo Build fehlgeschlagen!
    pause
)
