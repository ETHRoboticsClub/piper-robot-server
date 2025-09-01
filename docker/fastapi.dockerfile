# Tactile Teleop FastAPI Application
FROM continuumio/miniconda3:latest

LABEL maintainer="Tactile Robotics <info@tactilerobotics.ai>"
LABEL description="Tactile Teleop FastAPI Web Server"

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV CONDA_ENV_NAME=tactile-teleop

# Install system dependencies including OpenGL for OpenCV
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    build-essential \
    curl \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Create working directory
WORKDIR /app

# Copy environment and dependency files
COPY requirements.txt pyproject.toml ./
COPY src/ ./src/
COPY docker/docker-entrypoint.py ./

# Create conda environment and install dependencies
RUN conda create -n ${CONDA_ENV_NAME} python=3.10 -y && \
    conda run -n ${CONDA_ENV_NAME} pip install --no-cache-dir -r requirements.txt && \
    conda run -n ${CONDA_ENV_NAME} pip install -e . && \
    conda clean -afy

# Create app user for security
RUN useradd -m -u 1000 appuser && chown -R appuser:appuser /app
USER appuser

# Create directory for SSL certificates and sockets
RUN mkdir -p /home/appuser/.piper_teleop/ssl

# Expose port (for container networking)
EXPOSE 8000

# Default command runs the custom Docker entrypoint
CMD ["conda", "run", "-n", "tactile-teleop", "python", "docker-entrypoint.py"]