# Makefile for PDS-T1000-TSA24-Programm
# Kompiliert das Projekt mit g++ und raylib

# Compiler und Flags
CC = g++
CFLAGS = -std=c++17 -O2 -Wall -I include -I external/raylib/src
LIBS = -L external/raylib/lib -lraylib -lopengl32 -lgdi32 -lwinmm

# Quelldateien
SOURCES = src/main.cpp src/image_manager.cpp

# Ziel-Executable
TARGET = PDS-T1000.exe

# Standard-Target
all: $(TARGET)

# Build-Target
$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) $(SOURCES) -o $(TARGET) $(LIBS)
	@echo Build erfolgreich abgeschlossen.
# Clean-Target (löscht generierte Dateien)
clean:
	if exist *.obj del *.obj
	if exist *.exe del *.exe
	if exist *.pdb del *.pdb

# Run-Target (startet das Programm)
run: $(TARGET)
	./$(TARGET)

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
