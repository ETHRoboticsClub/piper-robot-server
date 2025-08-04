#!/bin/bash

# Tactile Teleop Docker Build Script
# Builds and deploys the containerized Tactile Teleop system

set -e

echo "=== Tactile Teleop Docker Deployment ==="
echo

# Parse command line arguments
ENVIRONMENT=${1:-production}
REBUILD=${2:-false}
ENV_FILE=${3:-}

# Set default environment file based on environment if not specified
if [ -z "$ENV_FILE" ]; then
    if [ "$ENVIRONMENT" = "prod" ] || [ "$ENVIRONMENT" = "production" ]; then
        ENV_FILE="production.env"
    else
        ENV_FILE="development.env"
    fi
fi

case $ENVIRONMENT in
    "dev"|"development")
        echo "🔧 Building for development environment"
        COMPOSE_FILE="docker-compose.yml:docker-compose.override.yml"
        ;;
    "prod"|"production")
        echo "🚀 Building for production environment"
        COMPOSE_FILE="docker-compose.yml"
        ;;
    *)
        echo "Usage: $0 [dev|prod] [rebuild] [env_file]"
        echo "  dev/development: Include development overrides"
        echo "  prod/production: Production configuration only"
        echo "  rebuild: Force rebuild of all images"
        echo "  env_file: Environment file to use (default: development.env)"
        echo ""
        echo "Examples:"
        echo "  $0 dev                    # Development with development.env"
        echo "  $0 prod rebuild           # Production rebuild with development.env"
        echo "  $0 prod false production.env  # Production with production.env"
        exit 1
        ;;
esac

# Handle rebuild flag
BUILD_FLAGS=""
if [ "$REBUILD" = "rebuild" ] || [ "$REBUILD" = "true" ]; then
    echo "🔄 Forcing rebuild of all images"
    BUILD_FLAGS="--build --no-cache"
fi

echo "📦 Docker Compose files: $COMPOSE_FILE"
echo "🔧 Environment file: $ENV_FILE"

# Load environment variables for this script
if [ -f "$ENV_FILE" ]; then
    set -a
    source "$ENV_FILE"
    set +a
fi

echo

# Check Docker installation
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed. Please install Docker first."
    exit 1
fi

if ! command -v docker-compose &> /dev/null; then
    echo "❌ Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

# Configure nginx based on environment variables
echo "⚙️  Configuring nginx with $ENV_FILE..."
./docker/scripts/configure-nginx.sh "$ENV_FILE"

echo
# Build and start services
echo "🏗️  Building and starting services..."
export ENV_FILE
COMPOSE_FILE=$COMPOSE_FILE docker-compose up -d $BUILD_FLAGS

echo
echo "⏳ Waiting for services to be healthy..."
sleep 10

# Check service status
echo "📊 Service Status:"
docker-compose ps

echo
echo "🔍 Health Checks:"

# Test nginx (using configured HTTPS port)
HTTPS_PORT=${NGINX_HTTPS_PORT:-8443}
if curl -k -f -s https://localhost:$HTTPS_PORT/health > /dev/null; then
    echo "✅ Nginx proxy: Healthy"
else
    echo "❌ Nginx proxy: Unhealthy"
fi

# Test FastAPI directly (in dev mode)
if [ "$ENVIRONMENT" = "dev" ] || [ "$ENVIRONMENT" = "development" ]; then
    if curl -f -s http://localhost:8000/health > /dev/null; then
        echo "✅ FastAPI direct: Healthy"
    else
        echo "❌ FastAPI direct: Unhealthy"
    fi
fi

echo
echo "=== Deployment Complete ==="
echo
echo "🌐 Web Interface: https://localhost:$HTTPS_PORT/"
echo "📋 Health Check: https://localhost:$HTTPS_PORT/health"

if [ "$ENVIRONMENT" = "dev" ] || [ "$ENVIRONMENT" = "development" ]; then
    echo "🔧 FastAPI Direct: http://localhost:8000/"
    echo "📝 Development mode: Live code reload enabled"
fi

echo
echo "📖 Useful commands:"
echo "  View logs:    docker-compose logs -f"
echo "  Stop services: docker-compose down"
echo "  Restart:      docker-compose restart"
echo
echo "📚 See DOCKER-DEPLOYMENT.md for detailed documentation"