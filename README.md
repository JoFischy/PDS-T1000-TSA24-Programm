# PDS-T1000 Factory Vehicle Simulation

## Übersicht

Das PDS-T1000 ist ein industrielles Fahrzeugsimulationssystem, das entwickelt wurde, um Fahrzeugbewegungen auf einem vordefinierten 13-Knoten-Netzwerk in einer Fabrikumgebung zu simulieren. Das System verwendet ein manuell definiertes Knotennetzwerk anstelle einer automatischen Pfaderkennung und bietet erweiterte Visualisierungs- und Debugging-Tools.

## 🚀 Hauptfeatures

### Fahrzeugsimulation
- **13-Knoten Factory Network**: Vordefiniertes Knotennetzwerk basierend auf exakten Koordinaten
- **Intelligente Fahrzeugsteuerung**: Automatisches Pathfinding zwischen Knoten
- **Kollisionsvermeidung**: Segmentbasierte Verkehrsmanagement
- **Dynamische Zielzuweisung**: Automatische neue Ziele nach Erreichen der Destination

### Visualisierung & Debugging
- **Interaktiver Koordinaten-Picker**: Tool zur präzisen Koordinatenermittlung (Taste `P`)
- **Erweiterte Kamerasteuerung**: WASD-Bewegung, Zoom, Reset-Funktion
- **Modulare Display-Optionen**: Separate Anzeige von Knoten, Segmenten, Fahrzeug-IDs
- **Real-time Rendering**: 60 FPS mit Raylib Graphics Engine

### Benutzerfreundlichkeit  
- **Einfache Kompilierung**: Ein-Klick Build-Scripts für Windows
- **Umfassende Steuerung**: Vollständige Tastatur- und Maussteuerung
- **Informative UI**: Detaillierte On-Screen-Informationen und Hilfe

## 🎮 Steuerung

### Kamera-Navigation
| Taste | Aktion |
|-------|--------|
| `WASD` | Kamera bewegen (hoch/links/runter/rechts) |
| `Mausrad` | Zoom ein/aus |
| `R` | Kamera zurücksetzen |

### Display-Optionen
| Taste | Aktion |
|-------|--------|
| `1` | Knoten anzeigen/ausblenden |
| `2` | Segmente anzeigen/ausblenden |
| `3` | Kreuzungen anzeigen/ausblenden |
| `4` | Fahrzeug-IDs anzeigen/ausblenden |
| `5` | Debug-Informationen anzeigen/ausblenden |

### Simulation & Tools
| Taste | Aktion |
|-------|--------|
| `SPACE` | Simulation pausieren/fortsetzen |
| `P` | **Koordinaten-Picker ein/aus** |
| `V` | Fahrzeug manuell spawnen |
| `ESC` | Programm beenden |

### Koordinaten-Picker
- **Aktivierung**: Taste `P` drücken
- **Verwendung**: Grünen Kreis mit der Maus ziehen
- **Anzeige**: Aktuelle X/Y-Koordinaten in der UI (oben links)
- **Zweck**: Präzise Koordinatenermittlung für neue Knoten

## 🏗️ Architektur

### Kern-Systeme

#### `Simulation` (Main Controller)
- **Zweck**: Zentrale Steuerung der gesamten Simulation
- **Verantwortlich für**: Initialisierung, Hauptschleife, Input-Handling
- **Schlüsselmethoden**: 
  - `initialize()` - System-Setup
  - `run()` - Hauptsimulationsschleife  
  - `createFactoryPathSystem()` - 13-Knoten-Netzwerk erstellen

#### `PathSystem` (Network Management)
- **Zweck**: Verwaltung des Knotennetzwerks und Verbindungen
- **Komponenten**: PathNode (Knoten), PathSegment (Verbindungen)
- **Features**: Knotenerstellung, Segmentverbindungen, Pathfinding-Support

#### `VehicleController` (Fleet Management)
- **Zweck**: Verwaltung aller Fahrzeuge in der Simulation
- **Features**: Spawning, Bewegung, Zielzuweisung, Kollisionsvermeidung
- **Integration**: Arbeitet eng mit SegmentManager zusammen

#### `SegmentManager` (Traffic Control)
- **Zweck**: Verkehrsfluss und Kollisionsvermeidung
- **Mechanismus**: Segmentbasierte Reservierung und Freigabe
- **Sicherheit**: Verhindert Fahrzeugkollisionen auf Pfaden

#### `Renderer` (Graphics Engine)
- **Zweck**: Alle visuellen Aspekte der Simulation
- **Features**: 2D-Kamera, Koordinaten-Picker, UI-Rendering
- **Backend**: Raylib für performante 2D-Grafiken

#### `Auto` (Vehicle Entity)
- **Zweck**: Einzelfahrzeug-Repräsentation
- **Eigenschaften**: Position, Geschwindigkeit, Status, Pfad
- **Verhalten**: Bewegung entlang Pfaden, Zielsuche

## 📍 13-Knoten Factory Network

### Knotendefinitionen
```cpp
Node 1:  (70, 65)     → Verbindungen: 2, 5
Node 2:  (640, 65)    → Verbindungen: 1, 3, 6
Node 3:  (985, 65)    → Verbindungen: 2, 4, 7
Node 4:  (1860, 65)   → Verbindungen: 3, 8
Node 5:  (70, 470)    → Verbindungen: 1, 6, 11
Node 6:  (640, 470)   → Verbindungen: 2, 5
Node 7:  (985, 320)   → Verbindungen: 3, 8, 9
Node 8:  (1860, 320)  → Verbindungen: 4, 7, 10
Node 9:  (985, 750)   → Verbindungen: 7, 10, 12
Node 10: (1860, 750)  → Verbindungen: 8, 9, 13
Node 11: (70, 1135)   → Verbindungen: 5, 12
Node 12: (985, 1135)  → Verbindungen: 9, 11, 13
Node 13: (1860, 1135) → Verbindungen: 10, 12
```

### Netzwerk-Eigenschaften
- **Total Nodes**: 13
- **Total Segments**: 15
- **Network Layout**: Optimiert für Factory-Layout
- **Connectivity**: Vollständig verbundenes Netzwerk für maximale Flexibilität

## 🛠️ Installation & Build

### Voraussetzungen
- **Compiler**: g++ (MinGW für Windows)
- **Standard**: C++17 oder höher
- **Dependencies**: Raylib (bereits im Projekt enthalten)

### Windows Build
```batch
# Einfacher Build
.\build.bat

# Clean Build (empfohlen nach Änderungen)
.\build.bat rebuild

# Nur Clean
.\build.bat clean

# Build und direkt ausführen
.\build.bat run
```

### Manuelle Kompilierung
```bash
g++ -D_USE_MATH_DEFINES -DPLATFORM_DESKTOP -Iinclude -Iexternal/raylib -std=c++17 -O2 src/auto.cpp src/main.cpp src/renderer.cpp src/simulation.cpp src/point.cpp src/movement_system.cpp src/vehicle_controller.cpp src/image_processor.cpp src/path_detector.cpp src/path_system.cpp src/segment_manager.cpp -o PDS-T1000.exe -Lexternal/raylib/lib -lraylib -lopengl32 -lgdi32 -lwinmm
```

## 🚀 Verwendung

### Programm starten
```bash
.\PDS-T1000.exe
```

### Typischer Workflow
1. **Start**: Programm startet mit 13-Knoten-Netzwerk
2. **Navigation**: WASD/Mausrad für Kamera-Navigation  
3. **Koordinaten finden**: `P` drücken, grünen Kreis bewegen
4. **Simulation**: `SPACE` für Pause/Resume
5. **Anpassungen**: Display-Optionen mit Tasten 1-5

### Koordinaten-Picker verwenden
1. Taste `P` drücken → Grüner Kreis erscheint
2. Kreis mit Maus zum gewünschten Punkt ziehen
3. Koordinaten in UI ablesen (oben links)
4. Für neue Knoten notieren

## 📁 Projektstruktur

```
PDS-T1000-TSA24-Programm/
├── include/                 # Header-Dateien
│   ├── auto.h              # Fahrzeug-Definition
│   ├── image_processor.h   # Bildverarbeitung
│   ├── movement_system.h   # Bewegungssystem
│   ├── path_detector.h     # Pfaderkennung
│   ├── path_system.h       # Pfad-Netzwerk
│   ├── point.h             # 2D-Punkt-Klasse
│   ├── renderer.h          # Graphics Rendering
│   ├── segment_manager.h   # Verkehrsmanagement
│   ├── simulation.h        # Haupt-Simulation
│   └── vehicle_controller.h # Fahrzeugsteuerung
├── src/                    # Implementierung
│   ├── auto.cpp
│   ├── image_processor.cpp
│   ├── main.cpp           # Einstiegspunkt
│   ├── movement_system.cpp
│   ├── path_detector.cpp
│   ├── path_system.cpp
│   ├── point.cpp
│   ├── renderer.cpp
│   ├── segment_manager.cpp
│   ├── simulation.cpp
│   └── vehicle_controller.cpp
├── external/              # Externe Libraries
│   └── raylib/           # Raylib Graphics
├── factory_layout.png    # Factory-Hintergrundbild
├── build.bat            # Windows Build-Script
└── README.md           # Diese Dokumentation
```

## 🔧 Konfiguration & Erweiterung

### Neue Knoten hinzufügen
1. Koordinaten mit Picker ermitteln
2. In `createFactoryPathSystem()` hinzufügen:
```cpp
int newNode = pathSystem.addNode(x, y);
pathSystem.addSegment(newNode, existingNode);
```

### Fahrzeugverhalten anpassen
- `Auto` Klasse: Geschwindigkeit, Größe
- `VehicleController`: Spawn-Verhalten
- `SegmentManager`: Verkehrsregeln

### Rendering anpassen
- `Renderer` Klasse: Farben, Größen
- UI-Layout in `renderUI()`
- Kamera-Verhalten in `updateCamera()`

## 🐛 Debugging & Problemlösung

### Häufige Probleme

#### Build-Fehler
- **"Zugriff verweigert"**: PDS-T1000.exe läuft noch → Prozess beenden
- **Fehlende g++**: MinGW installieren
- **Raylib-Fehler**: external/raylib Ordner überprüfen

#### Runtime-Probleme  
- **Fahrzeuge bewegen sich nicht**: Knotennetzwerk überprüfen
- **Kamera hängt**: `R` drücken für Reset
- **Koordinaten-Picker unsichtbar**: `P` drücken zum Aktivieren

#### Performance-Optimierung
- Debug-Optionen ausschalten (Taste `5`)
- Weniger Fahrzeuge spawnen
- Zoom-Level anpassen

### Debug-Modi
- **Taste `5`**: Debug-Informationen ein/aus
- **Taste `4`**: Fahrzeug-IDs für Tracking
- **Koordinaten-Picker**: Präzise Positionsdaten

## 📝 Changelog

### Version 2.0 (Aktuell)
- ✅ **13-Knoten Factory System** implementiert
- ✅ **Koordinaten-Picker** hinzugefügt  
- ✅ **Automatische Pfaderkennung entfernt**
- ✅ **Erweiterte Kamerasteuerung**
- ✅ **Verbesserte UI und Debugging-Tools**

### Version 1.0 (Vorgänger)
- Automatische Pfaderkennung aus Bildern
- 4 fest definierte Fahrzeuge
- Basis-Rendering und Navigation

## 🤝 Entwicklung

### Code-Stil
- **C++17 Standard**
- **RAII-Prinzipien**
- **Modularer Aufbau**
- **Konsistente Namenskonventionen**

### Beitrag leisten
1. Repository forken
2. Feature-Branch erstellen
3. Änderungen implementieren
4. Tests durchführen
5. Pull Request erstellen

## 📄 Lizenz

Dieses Projekt ist Teil des PDS-TSA24 Programms.

---

**Entwickelt für industrielle Fahrzeugsimulation und Pfadplanung** 🚗🏭

## 📚 Vollständige Dokumentation

Dieses Projekt verfügt über eine umfassende Dokumentations-Suite:

- **[📖 DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md)** - Übersicht aller Dokumentationen
- **[👤 USER_GUIDE.md](USER_GUIDE.md)** - Detailliertes Benutzerhandbuch  
- **[🏗️ ARCHITECTURE.md](ARCHITECTURE.md)** - System-Architektur und Design
- **[⚙️ API.md](API.md)** - Vollständige API-Referenz
- **[🛠️ DEVELOPER.md](DEVELOPER.md)** - Entwicklerhandbuch für Erweiterungen

**Empfehlung**: Starten Sie mit dem [USER_GUIDE.md](USER_GUIDE.md) für die praktische Nutzung!