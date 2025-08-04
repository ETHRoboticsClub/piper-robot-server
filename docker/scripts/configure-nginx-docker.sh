#!/bin/bash

# Configure nginx for Docker deployment
# This script generates nginx configs specifically for Docker containers

set -e

# Get project root (script is in docker/scripts/ subdirectory)
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NGINX_DIR="$PROJECT_ROOT/src/tactile_teleop/web_server/nginx"

# Handle environment file parameter
ENV_FILE="${1:-}"
if [ -n "$ENV_FILE" ]; then
    # Make path absolute if relative
    if [[ ! "$ENV_FILE" = /* ]]; then
        ENV_FILE="$PROJECT_ROOT/$ENV_FILE"
    fi
    
    if [ -f "$ENV_FILE" ]; then
        echo "Loading Docker environment from: $ENV_FILE"
        set -a
        source "$ENV_FILE"
        set +a
    else
        echo "WARNING: Environment file not found: $ENV_FILE"
    fi
fi

# Load environment variables
DOMAIN_NAME="${DOMAIN_NAME:-localhost}"
USE_SSL="${USE_SSL:-false}"

echo "Environment variables:"
echo "  ENV_FILE: $ENV_FILE"
echo "  DOMAIN_NAME: $DOMAIN_NAME"
echo "  USE_SSL: $USE_SSL"

# Docker deployment paths
FASTAPI_BACKEND="fastapi:8000"
WEB_ROOT="/usr/share/nginx/html"
CERTBOT_ROOT="/var/www/certbot"

echo "Configuring nginx for Docker deployment..."
echo "  Domain: $DOMAIN_NAME"
echo "  SSL: $USE_SSL"
echo "  Backend: $FASTAPI_BACKEND"
echo "  Web root: $WEB_ROOT"
echo "  Templates: $NGINX_DIR"

# Generate appropriate config
if [ "$USE_SSL" = "true" ]; then
    TEMPLATE_FILE="$NGINX_DIR/nginx.conf.prod.template"
    OUTPUT_FILE="$NGINX_DIR/nginx-tactile-teleop.conf"
    echo "Generating production HTTPS configuration for Docker..."
else
    TEMPLATE_FILE="$NGINX_DIR/nginx.conf.dev.template"
    OUTPUT_FILE="$NGINX_DIR/nginx-tactile-teleop.conf"
    echo "Generating development HTTP configuration for Docker..."
fi

# Check template exists
if [ ! -f "$TEMPLATE_FILE" ]; then
    echo "ERROR: Template file not found: $TEMPLATE_FILE"
    exit 1
fi

# Generate configuration
export DOMAIN_NAME FASTAPI_BACKEND WEB_ROOT CERTBOT_ROOT
envsubst '$DOMAIN_NAME $FASTAPI_BACKEND $WEB_ROOT $CERTBOT_ROOT' < "$TEMPLATE_FILE" > "$OUTPUT_FILE"

echo "Docker nginx configuration generated: $OUTPUT_FILE"