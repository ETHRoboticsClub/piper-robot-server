#!/bin/bash

# Configure nginx.conf from template based on environment variables
# This script generates nginx-tactile-teleop.conf from nginx.conf.dev.template or nginx.conf.prod.template using environment variables
# 
# Usage: ./configure-nginx.sh [environment_file]
# Example: ./configure-nginx.sh production.env

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
NGINX_DIR="$SCRIPT_DIR/nginx"

# Default values
DEFAULT_DOMAIN="localhost"
DEFAULT_USE_CUSTOM_SSL="false"

# Determine environment file to use
ENV_FILE="${1:-$PROJECT_ROOT/development.env}"

# Make path absolute if relative
if [[ ! "$ENV_FILE" = /* ]]; then
    ENV_FILE="$PROJECT_ROOT/$ENV_FILE"
fi

# Load environment variables from specified file
if [ -f "$ENV_FILE" ]; then
    echo "Loading configuration from $ENV_FILE"
    # Export variables from env file
    set -a
    source "$ENV_FILE"
    set +a
else
    echo "WARNING: Environment file not found at $ENV_FILE"
    echo "Available environment files:"
    ls -la "$PROJECT_ROOT"/*.env 2>/dev/null || echo "  No .env files found"
    echo "Using defaults..."
fi

# Set defaults if not provided
DOMAIN_NAME="${DOMAIN_NAME:-$DEFAULT_DOMAIN}"
USE_SSL="${USE_SSL:-true}"

# Configure backend and paths for direct deployment
FASTAPI_BACKEND="unix:/tmp/tactile-teleop.sock"
WEB_ROOT="$PROJECT_ROOT/src/tactile_teleop/web_server/web-ui"
CERTBOT_ROOT="/var/www/certbot"

echo "Configuring nginx for domain: $DOMAIN_NAME"
echo "Using SSL: $USE_SSL"
echo "FastAPI backend: $FASTAPI_BACKEND"
echo "Web root: $WEB_ROOT"

# Determine which config to generate based on SSL mode
if [ "$USE_SSL" = "true" ]; then
    TEMPLATE_FILE="$NGINX_DIR/nginx.conf.prod.template"
    OUTPUT_FILE="$NGINX_DIR/nginx-tactile-teleop.conf"
    echo "Generating production HTTPS configuration..."
else
    TEMPLATE_FILE="$NGINX_DIR/nginx.conf.dev.template"
    OUTPUT_FILE="$NGINX_DIR/nginx-tactile-teleop.conf"
    echo "Generating development HTTP configuration..."
fi

if [ ! -f "$TEMPLATE_FILE" ]; then
    echo "ERROR: Template file not found: $TEMPLATE_FILE"
    exit 1
fi

# Generate nginx.conf from template
echo "Generating $OUTPUT_FILE from template..."

# Use envsubst to replace environment variables in template
export DOMAIN_NAME FASTAPI_BACKEND WEB_ROOT CERTBOT_ROOT
envsubst '$DOMAIN_NAME $FASTAPI_BACKEND $WEB_ROOT $CERTBOT_ROOT' < "$TEMPLATE_FILE" > "$OUTPUT_FILE"

echo "Successfully configured nginx.conf for domain: $DOMAIN_NAME"

# Show what we generated
echo ""
echo "Generated nginx configuration preview:"
echo "======================================"
head -20 "$OUTPUT_FILE"
echo "..."
echo "======================================"
echo ""
echo "Full configuration written to: $OUTPUT_FILE"