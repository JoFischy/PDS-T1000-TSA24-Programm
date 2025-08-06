# Makefile for PDS-T1000-TSA24-Programm
# Kompiliert das Projekt mit Visual Studio Compiler

# Compiler und Flags
CC = cl
CFLAGS = /D "_USE_MATH_DEFINES" /D "PLATFORM_DESKTOP" /I "include" /I "external\raylib\src" /I "external\raylib\src\external\glfw\include"
LIBS = opengl32.lib gdi32.lib winmm.lib kernel32.lib shell32.lib user32.lib

# Quelldateien
CPP_SOURCES = src\auto.cpp src\main.cpp src\renderer.cpp src\simulation.cpp
C_SOURCES = external\raylib\src\rcore.c external\raylib\src\rshapes.c external\raylib\src\rtextures.c external\raylib\src\rtext.c external\raylib\src\rmodels.c external\raylib\src\raudio.c external\raylib\src\rglfw.c external\raylib\src\utils.c

# Ziel-Executable
TARGET = PDS-T1000.exe

# Standard-Target
all: $(TARGET)

# Build-Target
$(TARGET): $(CPP_SOURCES) $(C_SOURCES)
	$(CC) $(CFLAGS) $(CPP_SOURCES) $(C_SOURCES) /Fe:$(TARGET) /link $(LIBS)
	@echo Räume Objektdateien auf...
	@if exist *.obj del *.obj

# Clean-Target (löscht generierte Dateien)
clean:
	if exist *.obj del *.obj
	if exist *.exe del *.exe
	if exist *.pdb del *.pdb

# Run-Target (startet das Programm)
run: $(TARGET)
	$(TARGET)

# Rebuild-Target (clean + build)
rebuild: clean all

# Hilfe
help:
	@echo Verfügbare Targets:
	@echo   all     - Kompiliert das Projekt
	@echo   clean   - Löscht generierte Dateien
	@echo   run     - Kompiliert und startet das Programm
	@echo   rebuild - Clean + Build
	@echo   help    - Zeigt diese Hilfe

.PHONY: all clean run rebuild help
