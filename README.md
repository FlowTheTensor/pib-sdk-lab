# PIB SDK Docker Lab + Cerebra - Vollständige Robotics-Entwicklungsumgebung

Dieses Projekt ermöglicht es Ihnen, mit Python, ROS und der Cerebra-Weboberfläche aus dem Docker-Container heraus im Browser zu arbeiten. Der Container enthält ROS Noetic, Jupyter Lab für interaktive Entwicklung und das Cerebra Angular Frontend für die Roboter-Benutzeroberfläche.

Systemvoraussetzungen:
- Docker Desktop
- min. 12 GB Festplattenplatz

🚀

### Image bauen
Im Terminal eingeben im Projektpfad, also in dem Ordner, in dem die Dateien docker-compose.yml und Dockerfile liegen:
```bash
docker-compose build
```
oder
```bash
docker build -t pib-sdk-lab:latest
```

## Container starten
```bash
docker-compose up -d 
```
oder
```bash
docker run -d --name pib-sdk-container -p 8888:8888 -p 4200:4200 -p 8000:8000 -p 8080:8080 -p 11311:11311 pib-sdk-lab
```
Danach kann über das Dashboard geschaut werden, ob die Dienste laufen und man kann diese direkt im Browser starten
```powershell
# Öffnen Sie dashboard.html in Ihrem Browser
# Speichern Sie es als Lesezeichen für einfachen Zugriff!
start .\dashboard.html
```

## Zugriffsmöglichkeiten

### Standard-URLs (nach Container-Start)

- **Jupyter Lab:** <http://localhost:8888> ⭐ **Hauptzugang**
- **Cerebra Frontend:** <http://localhost:4200> 🧠 **Roboter-UI**
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

## Installierte Pakete

### Wissenschaftliche Bibliotheken
- NumPy, SciPy, Matplotlib
- OpenCV für Computer Vision
- Pillow für Bildverarbeitung

### Robotik & ROS

- **ROS Noetic** (Robot Operating System)
- Robotics Toolbox Python
- Spatial Math Python
- PIB SDK
- ROS Python Libraries (rospy, geometry_msgs, sensor_msgs, etc.)

### Web-Entwicklung

- Jupyter Lab/Notebook
- IPython Widgets
- Plotly (interaktive Plots)
- Bokeh (Datenvisualisierung)

### Roboter Frontend (Cerebra)

- **Angular 18** Frontend für PIB-Roboter
- **Material Design** Benutzeroberfläche
- **ROS Integration** über roslib
- **Live-Development** mit Hot-Reload

### Kommunikation

- WebSockets
- ROS LibPy
- ROS Topics, Services & Actions

## Tipps

1. **Kein Token/Passwort:** Jupyter ist für lokale Entwicklung ohne Authentifizierung konfiguriert
2. **Live-Reload:** Ihr Code-Verzeichnis ist als Volume gemountet - Änderungen werden sofort übernommen
3. **Port-Konflikte:** Falls ein Port belegt ist, können Sie ihn in `docker-compose.yml` ändern
4. **GPU-Unterstützung:** Uncommentieren Sie die GPU-Konfiguration in `docker-compose.yml` falls benötigt

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





