#include "simulation.h"
#include <iostream>
#include <cstring>

int main(int argc, char* argv[]) {
    const char* imagePath = "factory_layout.png";
    
    // Check if custom image path is provided
    if (argc > 1) {
        imagePath = argv[1];
    }
    
    std::cout << "Starte Factory Vehicle Simulation..." << std::endl;
    std::cout << "Lade Fabrikbild: " << imagePath << std::endl;
    std::cout << "Verwende vordefiniertes 13-Knoten Netzwerk" << std::endl;
    
    Simulation simulation;
    
    if (!simulation.initialize(imagePath)) {
        std::cerr << "Fehler: Simulation konnte nicht initialisiert werden!" << std::endl;
        return -1;
    }
    
    std::cout << "Simulation erfolgreich gestartet!" << std::endl;
    std::cout << "Steuerung:" << std::endl;
    std::cout << "  WASD - Kamera bewegen" << std::endl;
    std::cout << "  Mausrad - Zoom" << std::endl;
    std::cout << "  SPACE - Pause/Resume" << std::endl;
    std::cout << "  P - Koordinaten-Picker ein/aus" << std::endl;
    std::cout << "  V - Fahrzeug spawnen" << std::endl;
    std::cout << "  +/- - Spawn-Rate ändern" << std::endl;
    std::cout << "  1-5 - Display-Optionen umschalten" << std::endl;
    std::cout << "  R - Kamera zurücksetzen" << std::endl;
    std::cout << "  ESC - Beenden" << std::endl;
    
    simulation.run();
    
    std::cout << "Simulation beendet." << std::endl;
    return 0;
}