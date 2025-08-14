#include "simulation.h"
#include <iostream>

int main() {
    std::cout << "Starte Factory Vehicle Simulation..." << std::endl;

    Simulation simulation;

    if (!simulation.initialize("factory_layout.png")) {
        std::cerr << "Fehler: Simulation konnte nicht initialisiert werden!" << std::endl;
        return -1;
    }

    std::cout << "Simulation erfolgreich gestartet!" << std::endl;
    std::cout << "Steuerung:" << std::endl;
    std::cout << "  WASD - Kamera bewegen" << std::endl;
    std::cout << "  SPACE - Pause/Resume" << std::endl;
    std::cout << "  V - Fahrzeug spawnen" << std::endl;
    std::cout << "  F11 - Vollbildmodus umschalten" << std::endl;
    std::cout << "  F - Gesamte Simulation anzeigen (Fit to View)" << std::endl;
    std::cout << "  R - Kamera zurücksetzen" << std::endl;
    std::cout << "  ESC - Beenden" << std::endl;

    simulation.run();

    std::cout << "Simulation beendet." << std::endl;
    return 0;
}