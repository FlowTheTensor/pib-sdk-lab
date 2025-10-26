# Dockerfile for PIB SDK with Python 3.11, ROS2 Jazzy on Ubuntu 24.04 LTS
FROM osrf/ros:jazzy-desktop-full-noble

# Set working directory
WORKDIR /app

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV PIP_NO_WARN_SCRIPT_LOCATION=1

# Add deadsnakes PPA for Python 3.11 and install Python 3.11, Node.js and system dependencies
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y \
    python3.11 \
    python3.11-dev \
    python3.11-venv \
    python3-pip \
    gcc \
    g++ \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    pkg-config \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    libeigen3-dev \
    libxml2-dev \
    libxslt-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js 20.x for Cerebra frontend (latest LTS)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && node --version \
    && npm --version

# Create virtual environment to avoid conflicts with ROS2 packages
RUN python3.11 -m venv /opt/pib-venv
ENV PATH="/opt/pib-venv/bin:$PATH"
ENV VIRTUAL_ENV="/opt/pib-venv"

# Upgrade pip in virtual environment
RUN /opt/pib-venv/bin/pip install --upgrade pip

# Install core scientific packages (latest versions)
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    numpy \
    scipy \
    matplotlib

# Install computer vision packages (latest versions)
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    opencv-python \
    opencv-contrib-python \
    pillow

# Install networking packages (rclpy is already included with ROS2)
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    websockets

# Install PyYAML compatible with ROS2
RUN /opt/pib-venv/bin/pip install --no-cache-dir --upgrade pyyaml

# Install Jupyter and web packages (latest versions)
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    jupyter \
    jupyterlab \
    notebook \
    ipywidgets \
    plotly \
    bokeh

# Install PIB SDK from PR-978 branch (includes speech module)
RUN /opt/pib-venv/bin/pip install --no-cache-dir --upgrade git+https://github.com/pib-rocks/pib-sdk.git@PR-978

# Install robotics packages 
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    roboticstoolbox-python \
    spatialmath-python \
    spatialgeometry \
    pgraph-python \
    ansitable \
    colored \
    progress

# Clone PIB Backend from PR-961 branch (neueste Webots Integration)
WORKDIR /app
RUN git clone -b PR-961 --depth 1 https://github.com/pib-rocks/pib-backend.git pib-backend

# Clone Digi-Twin Studio from GitHub (original files)
RUN git clone https://github.com/pib-rocks/digi-twin-studio.git digi-twin-studio

# Install Digi-Twin Studio dependencies
WORKDIR /app/digi-twin-studio
RUN npm install

# Return to main app directory  
WORKDIR /app

# Copy test notebook for Jupyter
COPY test_notebook.ipynb /app/

# Create clean directory for Jupyter notebooks
RUN mkdir -p /app/notebooks
COPY test_notebook.ipynb /app/notebooks/

# Clone Cerebra from GitHub (PR-981 branch) and setup dependencies
RUN git clone -b PR-981 --depth 1 https://github.com/pib-rocks/cerebra.git /app/cerebra && \
    cd /app/cerebra && \
    git submodule update --init --recursive && \
    npm install

# Go back to main working directory
WORKDIR /app

# Create Jupyter configuration directory
RUN mkdir -p /root/.jupyter

# Build PIB ROS2 Workspace
WORKDIR /app/pib-backend
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --packages-select pibsim_webots --symlink-install'

# Setup environment for both ROS2 and virtual environment
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /app/pib-backend/install/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/pib-venv/bin/activate' >> /root/.bashrc && \
    echo 'export PYTHONPATH=/opt/ros/jazzy/lib/python3.11/site-packages:$PYTHONPATH' >> /root/.bashrc

# Generate Jupyter config
RUN /opt/pib-venv/bin/jupyter lab --generate-config && \
    echo "c.ServerApp.ip = '0.0.0.0'" >> /root/.jupyter/jupyter_lab_config.py && \
    echo "c.ServerApp.port = 8888" >> /root/.jupyter/jupyter_lab_config.py && \
    echo "c.ServerApp.open_browser = False" >> /root/.jupyter/jupyter_lab_config.py && \
    echo "c.ServerApp.allow_root = True" >> /root/.jupyter/jupyter_lab_config.py && \
    echo "c.ServerApp.token = ''" >> /root/.jupyter/jupyter_lab_config.py && \
    echo "c.ServerApp.password = ''" >> /root/.jupyter/jupyter_lab_config.py

# Create startup script that sources both ROS2 and virtual env, and starts all services
RUN echo '#!/bin/bash' > /root/start_pib.sh && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/start_pib.sh && \
    echo 'source /app/pib-backend/install/setup.bash' >> /root/start_pib.sh && \
    echo 'source /opt/pib-venv/bin/activate' >> /root/start_pib.sh && \
    echo 'export PYTHONPATH=/opt/ros/jazzy/lib/python3.11/site-packages:$PYTHONPATH' >> /root/start_pib.sh && \
    echo '' >> /root/start_pib.sh && \
    echo '# Start ROS2 Bridge for Webots in background' >> /root/start_pib.sh && \
    echo 'nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 > /tmp/rosbridge.log 2>&1 &' >> /root/start_pib.sh && \
    echo '' >> /root/start_pib.sh && \
    echo '# Start Cerebra JSON Server Backend in background' >> /root/start_pib.sh && \
    echo 'cd /app/cerebra && nohup node ./server/json-server-funktion.mjs > /tmp/backend.log 2>&1 &' >> /root/start_pib.sh && \
    echo '' >> /root/start_pib.sh && \
    echo '# Wait for backend to start' >> /root/start_pib.sh && \
    echo 'sleep 3' >> /root/start_pib.sh && \
    echo '' >> /root/start_pib.sh && \
    echo '# Start Cerebra Angular Frontend in background' >> /root/start_pib.sh && \
    echo 'cd /app/cerebra && nohup ./node_modules/.bin/ng serve --host=0.0.0.0 --port=4200 --disable-host-check --configuration development,rosmock > /tmp/angular.log 2>&1 &' >> /root/start_pib.sh && \
    echo '' >> /root/start_pib.sh && \
    echo '# Start Digi-Twin Studio Proxy Server in background' >> /root/start_pib.sh && \
    echo 'cd /app/digi-twin-studio && nohup npm run proxy > /tmp/proxy.log 2>&1 &' >> /root/start_pib.sh && \
    echo '' >> /root/start_pib.sh && \
    echo '# Start Digi-Twin Studio Angular Frontend in background' >> /root/start_pib.sh && \
    echo 'cd /app/digi-twin-studio && nohup npm run start -- --host=0.0.0.0 --port=4201 --disable-host-check > /tmp/digi-twin.log 2>&1 &' >> /root/start_pib.sh && \
    echo '' >> /root/start_pib.sh && \
    echo '# Start the main command' >> /root/start_pib.sh && \
    echo 'if [ $# -gt 0 ]; then' >> /root/start_pib.sh && \
    echo '  exec "$@"' >> /root/start_pib.sh && \
    echo 'else' >> /root/start_pib.sh && \
    echo '  echo "All services started. Container running..."' >> /root/start_pib.sh && \
    echo '  tail -f /tmp/*.log' >> /root/start_pib.sh && \
    echo 'fi' >> /root/start_pib.sh && \
    chmod +x /root/start_pib.sh

# randn FIX:
RUN find /opt/pib-venv -name "EKF.py" -exec sed -i 's/from scipy import integrate, randn/from scipy import integrate\nfrom numpy.random import randn/' {} \;

# Expose ports
EXPOSE 8000 8888 11311 4200 4201 3001 9090

# Default command
ENTRYPOINT ["/root/start_pib.sh"]
CMD ["/opt/pib-venv/bin/jupyter", "lab", "--ip=0.0.0.0", "--port=8888", "--no-browser", "--allow-root", "--notebook-dir=/app/notebooks"]