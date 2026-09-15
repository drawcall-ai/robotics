FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rosbridge-server python3-pil \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY . /app
RUN python3 -m unittest discover -p 'test_*.py'
CMD ["bash", "/app/start.sh"]
