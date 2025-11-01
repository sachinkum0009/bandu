FROM ros:rolling-ros-base

WORKDIR /app

RUN apt update && apt install ros-rolling-cv-bridge -y

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv build-essential ca-certificates libffi-dev libssl-dev \
    libgl1 libglib2.0-0 libsm6 libxrender1 libxext6 ffmpeg && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

COPY src/bandu /app

COPY install  /tmp

RUN pip install --upgrade pip setuptools wheel && \
    pip install --no-cache-dir -r requirements.txt

EXPOSE 8000

ENTRYPOINT ["/bin/bash", "-lc", "source /opt/ros/rolling/setup.bash && source /tmp/setup.bash && exec bash start_app.sh"]