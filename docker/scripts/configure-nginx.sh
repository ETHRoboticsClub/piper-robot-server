#!/bin/bash

# Configure nginx.conf from template based on environment variables
# This script generates nginx.conf from nginx.conf.template using environment variables
# 
# Usage: ./configure-nginx.sh [environment_file]
# Example: ./configure-nginx.sh production.env

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DOCKER_DIR="$PROJECT_ROOT/docker"
CONFIG_DIR="$DOCKER_DIR/config"

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
USE_CUSTOM_SSL="${USE_CUSTOM_SSL:-$DEFAULT_USE_CUSTOM_SSL}"

echo "Configuring nginx for domain: $DOMAIN_NAME"
echo "Using custom SSL: $USE_CUSTOM_SSL"

# Check if template exists
TEMPLATE_FILE="$CONFIG_DIR/nginx.conf.template"
OUTPUT_FILE="$CONFIG_DIR/nginx.conf"

if [ ! -f "$TEMPLATE_FILE" ]; then
    echo "ERROR: Template file not found: $TEMPLATE_FILE"
    exit 1
fi

# Generate nginx.conf from template
echo "Generating $OUTPUT_FILE from template..."

# Use envsubst to replace environment variables in template
envsubst '$DOMAIN_NAME' < "$TEMPLATE_FILE" > "$OUTPUT_FILE"

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