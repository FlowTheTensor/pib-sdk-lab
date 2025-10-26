# PIB SDK Docker Lab + Cerebra - Vollständige Robotics-Entwicklungsumgebung

Dieses Projekt ermöglicht es Ihnen, mit Python, ROS2 und der Cerebra-Weboberfläche aus dem Docker-Container heraus im Browser zu arbeiten. Der Container basiert auf Ubuntu 24.04 LTS und enthält ROS2 Jazzy, Python 3.11, Jupyter Lab für interaktive Entwicklung und das Cerebra Angular Frontend für die Roboter-Benutzeroberfläche.

## Systemvoraussetzungen
- **Docker Desktop** (empfohlen: neueste Version)
- **Mindestens 12 GB freier Festplattenplatz**
- **Arbeitsspeicher:** 8 GB RAM empfohlen für optimale Performance

## Technische Basis
- **Ubuntu 24.04 LTS** als Base Image
- **ROS2 Jazzy Jalopy** (neueste LTS-Version) 
- **Python 3.11** (kompatibel mit PIB-SDK <3.12 Requirement)
- **PIB-SDK PR-978** mit Speech-Modul Support
- **Robotics Toolbox** mit scipy.randn Kompatibilitäts-Patches

🚀

### Image bauen
Im Terminal eingeben im Projektpfad, also in dem Ordner, in dem die Dateien docker-compose.yml und Dockerfile liegen:
```bash
docker-compose build
```
oder
```bash
docker build -t pib-sdk-lab:latest .
```

## Container starten
```bash
docker-compose up -d 
```
oder
```bash
docker run -d --name pib-sdk-container -p 8888:8888 -p 4200:4200 -p 8000:8000 -p 8080:8080 -p 11311:11311 pib-sdk-lab:latest
```
Danach kann über das Dashboard geschaut werden, ob die Dienste laufen und man kann diese direkt im Browser starten
```powershell
# Öffnen Sie dashboard.html in Ihrem Browser
# Speichern Sie es als Lesezeichen für einfachen Zugriff!
start .\dashboard.html
```
![Dashboard](https://nerdifant.de/pib-sdk-lab-resources/Dashboard.png)

## Zugriffsmöglichkeiten

### Standard-URLs (nach Container-Start)

- **Jupyter Lab:** <http://localhost:8888> ⭐ **Hauptzugang**
- **Cerebra Frontend:** <http://localhost:4200> 🧠 **Roboter-UI**
- **Cerebra Digi-Twin:** <http://localhost:4201> 🤖 **Webots Integration**
- **Digi-Twin Proxy:** <http://localhost:3001> 🔗 **API Gateway**
- **Dashboard:** Öffnen Sie `dashboard.html` im Browser

## Nützliche Befehle

### Container Management
```bash
# Alle Services starten
docker-compose up

# Nur bestimmten Service starten
docker-compose up pib-sdk

# Im Hintergrund starten
docker-compose up -d

# Services stoppen
docker-compose down

# Container neu bauen
docker-compose build

# Cerebra Services starten
docker-compose --profile cerebra-dev up    # Development Mode
docker-compose --profile cerebra-prod up   # Production Build
```

### Interaktive Shell im Container
```bash
# In laufenden Container einsteigen
docker exec -it pib-sdk-container /bin/bash

# Development-Container mit Shell starten
docker-compose --profile dev run pib-dev /bin/bash
```

### Jupyter manuell starten
```bash
# Im Container:
./start_jupyter.sh

# Oder direkt:
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser --allow-root
```

### ROS2-spezifische Befehle
```bash
# ROS2 Environment aktivieren
source /opt/ros/jazzy/setup.bash

# PIB Python Environment aktivieren  
source /opt/pib-venv/bin/activate

# ROSBridge für WebSocket-Kommunikation starten
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# ROS2 Topics anzeigen
ros2 topic list

# ROS2 Services anzeigen  
ros2 service list

# PIB-SDK im Container testen
python3 -c "from pib_sdk import control; print('PIB-SDK OK')"
```

### Cerebra Digi-Twin Studio
```bash
# Cerebra Digi-Twin Service starten
docker-compose --profile cerebra-digi-twin up

# Nur Proxy Server (für Webots/Onshape Integration)
cd /app/cerebra-digi-twin && npm run proxy

# Frontend separat starten
cd /app/cerebra-digi-twin && npm start

# Webots Status prüfen
curl http://localhost:3001/webots/status
```

## Installierte Pakete

### Wissenschaftliche Bibliotheken
- NumPy, SciPy, Matplotlib
- OpenCV für Computer Vision
- Pillow für Bildverarbeitung

### Robotik & ROS2

- **ROS2 Jazzy Jalopy** (Robot Operating System 2 - LTS Version)
- **Python 3.11** Virtual Environment (/opt/pib-venv)
- **PIB SDK PR-978** mit Speech-Modul Integration
- **Robotics Toolbox Python** mit scipy.randn Kompatibilitäts-Patch
- **Spatial Math Python** für 3D-Transformationen
- **ROS2 Python Libraries** (rclpy, geometry_msgs, sensor_msgs, etc.)
- **ROSBridge Suite** für WebSocket-Kommunikation

### Web-Entwicklung

- Jupyter Lab/Notebook
- IPython Widgets
- Plotly (interaktive Plots)
- Bokeh (Datenvisualisierung)

### Roboter Frontend (Cerebra)

- **Angular 18** Frontend für PIB-Roboter
- **Material Design** Benutzeroberfläche
- **ROS2 Integration** über roslibjs und ROSBridge
- **Live-Development** mit Hot-Reload
- **Service Monitoring Dashboard** mit Auto-Refresh

### Cerebra Digi-Twin Studio (NEU)

- **Webots Integration** - Direkte Verbindung zu Webots Simulator (Windows)
- **Onshape CAD API** - Import von 3D-Modellen und Assemblies
- **CORS Proxy Server** - Überbrückt Browser-Sicherheitsbeschränkungen
- **PIB-Webots Bridge** - Steuert PIB-Roboter in Webots-Simulation
- **Real-time WebSocket** - Bidirektionale Kommunikation Container ↔ Webots

### Kommunikation & Integration

- **WebSockets** für Echtzeit-Kommunikation
- **ROSLibPy** für Python-ROS2 Integration
- **ROS2 Topics, Services & Actions** vollständig unterstützt
- **PIB Speech Module** für Voice Assistant Integration
- **Zero-ROS SDK** - PIB-SDK funktioniert ohne ROS-Environment

## Wichtige Hinweise

### Python Version Kompatibilität
- **PIB-SDK benötigt Python ≥3.9, <3.12** 
- **Python 3.14 ist NICHT kompatibel** (automatische Installation in neuen Raspberry Pi OS)
- Container verwendet **Python 3.11** für optimale Kompatibilität

### ROS2-spezifische Features
- **Zero-ROS SDK:** PIB-SDK funktioniert ohne ROS2-Environment-Setup
- **ROSBridge WebSockets** für Browser-Integration auf Port 9090
- **Service Discovery:** Dashboard überwacht automatisch alle Services

### Entwicklungstipps

1. **Kein Token/Passwort:** Jupyter ist für lokale Entwicklung ohne Authentifizierung konfiguriert
2. **Live-Reload:** Ihr Code-Verzeichnis ist als Volume gemountet - Änderungen werden sofort übernommen
3. **Port-Konflikte:** Falls ein Port belegt ist, können Sie ihn in `docker-compose.yml` ändern
4. **GPU-Unterstützung:** Uncommentieren Sie die GPU-Konfiguration in `docker-compose.yml` falls benötigt
5. **PIB Notebook:** Verwenden Sie `test_notebook.ipynb` für vollständige PIB-SDK Funktionsreferenz

## Digitaler Zwilling
Python PIB-SDK ←→ ROS2 ←→ WebSocket ←→ Webots Simulator
     ↕️           ↕️        ↕️              ↕️
 Roboter-Code  ROS Topics   Web-Bridge   3D-Simulation

## Troubleshooting

### Jupyter startet nicht
```bash
# Logs anschauen
docker-compose logs pib-sdk

# Container neu starten
docker-compose restart pib-sdk
```

### Port bereits belegt
```bash
# Andere Ports in docker-compose.yml verwenden, z.B.:
ports:
  - "9999:8888"  # Dann http://localhost:9999
```

### Packages fehlen
```bash
# In den Container einsteigen und installieren
docker exec -it pib-sdk-container pip install paket-name

# Oder requirements.txt erweitern und neu bauen
docker-compose build
```



**⚠️ Wichtiger Hinweis:** ROS2 verwendet andere Topics/Services als ROS1. Bestehender ROS1-Code muss für ROS2 angepasst werden.






