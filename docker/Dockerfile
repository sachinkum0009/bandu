FROM ros:rolling-ros-base

WORKDIR /app

# Install system deps including OpenCV runtime libs and ffmpeg
RUN apt update && apt install ros-rolling-cv-bridge -y

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv build-essential ca-certificates libffi-dev libssl-dev \
    libgl1 libglib2.0-0 libsm6 libxrender1 libxext6 ffmpeg && \
    rm -rf /var/lib/apt/lists/*

# Create and activate virtualenv
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Copy project files
COPY src/bandu /app

COPY install  /tmp

# Upgrade pip and install Python deps into venv
RUN pip install --upgrade pip setuptools wheel && \
    pip install --no-cache-dir -r requirements.txt

# Expose port used by your app (adjust if needed)
EXPOSE 8000


# Ensure ROS environment is sourced when container runs
ENTRYPOINT ["/bin/bash", "-lc", "source /opt/ros/rolling/setup.bash && source /tmp/setup.bash && exec bash start_app.sh"]