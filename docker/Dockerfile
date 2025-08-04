# Tactile Teleop FastAPI Application
FROM python:3.10-slim

LABEL maintainer="Tactile Robotics <info@tactilerobotics.ai>"
LABEL description="Tactile Teleop FastAPI Web Server"

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create working directory
WORKDIR /app

# Copy environment and dependency files
COPY requirements.txt pyproject.toml ./
COPY src/ ./src/
COPY docker/docker-entrypoint.py ./

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt && \
    pip install -e .

# Create app user for security
RUN useradd -m -u 1000 appuser && chown -R appuser:appuser /app
USER appuser

# Create directory for SSL certificates and sockets
RUN mkdir -p /home/appuser/.tactile_teleop/ssl

# Expose port (for container networking)
EXPOSE 8000

# Default command runs the custom Docker entrypoint
CMD ["python", "docker-entrypoint.py"]