# Dockerfile for PIB SDK with Python 3.9, ROS Noetic - Conflict-Free Version
FROM osrf/ros:noetic-desktop-full

# Set working directory
WORKDIR /app

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV ROS_DISTRO=noetic
ENV ROS_VERSION=1
ENV ROS_PYTHON_VERSION=3
ENV PIP_NO_WARN_SCRIPT_LOCATION=1

# Install Python 3.9, Node.js and system dependencies
RUN apt-get update && apt-get install -y \
    python3.9 \
    python3.9-dev \
    python3.9-distutils \
    python3.9-venv \
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

# Install Node.js 18.x for Cerebra frontend
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get install -y nodejs \
    && node --version \
    && npm --version

# Create virtual environment to avoid conflicts with ROS packages
RUN python3.9 -m venv /opt/pib-venv
ENV PATH="/opt/pib-venv/bin:$PATH"
ENV VIRTUAL_ENV="/opt/pib-venv"

# Upgrade pip in virtual environment
RUN /opt/pib-venv/bin/pip install --upgrade pip

# Install core scientific packages
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    numpy==1.24.3 \
    scipy==1.10.1 \
    matplotlib==3.7.2

# Install computer vision packages
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    opencv-python==4.8.1.78 \
    opencv-contrib-python==4.8.1.78 \
    pillow==10.0.1

# Install robotics packages
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    roboticstoolbox-python==1.1.0 \
    spatialmath-python==1.1.14 \
    spatialgeometry==1.1.0 \
    pgraph-python==0.6.3 \
    ansitable==0.11.4 \
    colored==2.3.1 \
    progress==1.6.1

# Install networking packages
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    websockets==11.0.3 \
    roslibpy==2.0.0

# Install PyYAML compatible with ROS
RUN /opt/pib-venv/bin/pip install --no-cache-dir --force-reinstall pyyaml==6.0.1

# Install Jupyter and web packages
RUN /opt/pib-venv/bin/pip install --no-cache-dir \
    jupyter==1.0.0 \
    jupyterlab==4.0.7 \
    notebook==7.0.6 \
    ipywidgets==8.1.1 \
    plotly==5.17.0 \
    bokeh==3.3.0

# Install PIB SDK
RUN /opt/pib-venv/bin/pip install --no-cache-dir pib-sdk==0.2

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

# Setup environment for both ROS and virtual environment
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/pib-venv/bin/activate' >> /root/.bashrc && \
    echo 'export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH' >> /root/.bashrc

# Generate Jupyter config
RUN /opt/pib-venv/bin/jupyter notebook --generate-config && \
    echo "c.NotebookApp.ip = '0.0.0.0'" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.port = 8888" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.open_browser = False" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.allow_root = True" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.token = ''" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.password = ''" >> /root/.jupyter/jupyter_notebook_config.py

# Create startup script that sources both ROS and virtual env, and starts all services
RUN echo '#!/bin/bash' > /root/start_pib.sh && \
    echo 'source /opt/ros/noetic/setup.bash' >> /root/start_pib.sh && \
    echo 'source /opt/pib-venv/bin/activate' >> /root/start_pib.sh && \
    echo 'export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH' >> /root/start_pib.sh && \
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
    echo '# Start the main command' >> /root/start_pib.sh && \
    echo 'exec "$@"' >> /root/start_pib.sh && \
    chmod +x /root/start_pib.sh

# Expose ports
EXPOSE 8000 8888 11311 4200

# Default command
ENTRYPOINT ["/root/start_pib.sh"]
CMD ["/opt/pib-venv/bin/jupyter", "lab", "--ip=0.0.0.0", "--port=8888", "--no-browser", "--allow-root", "--notebook-dir=/app/notebooks"]