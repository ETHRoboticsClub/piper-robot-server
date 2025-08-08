#!/bin/bash

# Configure nginx.conf from template based on environment variables
# This script generates nginx-tactile-teleop.conf from nginx.conf.dev.template or nginx.conf.prod.template using environment variables
# 
# Usage: ./configure-nginx.sh [environment_file] [deployment_mode] [--validate]
# 
# SSL Configuration:
#   Set SSL_ENABLED=true/false to explicitly enable/disable SSL for both Docker and direct deployments
#   If SSL_ENABLED is not set, the script will auto-detect based on Let's Encrypt certificates
#
# Examples: 
#   SSL_ENABLED=false ./configure-nginx.sh development.env docker    # HTTP only
#   SSL_ENABLED=true ./configure-nginx.sh production.env docker     # Force HTTPS
#   ./configure-nginx.sh production.env direct                      # Auto-detect SSL
#   ./configure-nginx.sh production.env docker --validate           # Generate config and validate

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
NGINX_DIR="$SCRIPT_DIR"

# Default values
DEFAULT_DOMAIN="localhost"
DEFAULT_USE_CUSTOM_SSL="false"

# Parse arguments
ENV_FILE="${1:-$PROJECT_ROOT/development.env}"
DEPLOYMENT_MODE="${2:-auto}"
VALIDATE_MODE=false

# Check for --validate flag in any position
for arg in "$@"; do
    if [ "$arg" = "--validate" ]; then
        VALIDATE_MODE=true
        break
    fi
done

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

# Auto-detect deployment mode if not specified
if [ "$DEPLOYMENT_MODE" = "auto" ]; then
    if [ -n "${DOCKER_CONTAINER:-}" ] || [ -f "/.dockerenv" ] || [ "${IN_DOCKER:-false}" = "true" ]; then
        DEPLOYMENT_MODE="docker"
        echo "🐳 Auto-detected Docker environment"
    else
        DEPLOYMENT_MODE="direct"
        echo "🖥️  Auto-detected direct deployment environment"
    fi
fi

# Configure backend and paths based on deployment mode
if [ "$DEPLOYMENT_MODE" = "docker" ]; then
    echo "🐳 Configuring for Docker deployment..."
    FASTAPI_BACKEND="fastapi:8000"
    WEB_ROOT="/usr/share/nginx/html"
    CERTBOT_ROOT="/var/www/certbot"
else
    echo "🖥️  Configuring for direct deployment..."
    FASTAPI_BACKEND="unix:/tmp/tactile-teleop.sock"
    WEB_ROOT="$PROJECT_ROOT/src/tactile_teleop/web_server/web-ui"
    CERTBOT_ROOT="/var/www/certbot"
fi

# Determine SSL configuration using SSL_ENABLED environment variable
# Priority: SSL_ENABLED env var > auto-detection
if [ "${SSL_ENABLED:-}" = "true" ]; then
    USE_SSL="true"
    echo "🔒 SSL explicitly enabled via SSL_ENABLED environment variable"
elif [ "${SSL_ENABLED:-}" = "false" ]; then
    USE_SSL="false"
    echo "🔓 SSL explicitly disabled via SSL_ENABLED environment variable"
else
    # Auto-detect SSL if certificates are available (when SSL_ENABLED not explicitly set)
    if [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" ] && [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem" ]; then
        echo "🔍 Auto-detected Let's Encrypt certificates, enabling SSL"
        USE_SSL="true"
    else
        USE_SSL="false"
        echo "🔍 No SSL certificates found, using HTTP mode"
    fi
fi

echo "Environment variables:"
echo "  ENV_FILE: $ENV_FILE"
echo "  DEPLOYMENT_MODE: $DEPLOYMENT_MODE"
echo "  DOMAIN_NAME: $DOMAIN_NAME"
echo "  USE_SSL: $USE_SSL (auto-detected: $([ -f "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" ] && echo "yes" || echo "no"))"
echo "  FASTAPI_BACKEND: $FASTAPI_BACKEND"
echo "  WEB_ROOT: $WEB_ROOT"
echo "  TEMPLATES: $NGINX_DIR"

# Determine which config to generate based on SSL mode
if [ "$USE_SSL" = "true" ]; then
    TEMPLATE_FILE="$NGINX_DIR/nginx.conf.prod.template"
    OUTPUT_FILE="$NGINX_DIR/nginx-tactile-teleop.conf"
    echo "Generating production HTTPS configuration for $DEPLOYMENT_MODE deployment..."
else
    TEMPLATE_FILE="$NGINX_DIR/nginx.conf.dev.template"
    OUTPUT_FILE="$NGINX_DIR/nginx-tactile-teleop.conf"
    echo "Generating development HTTP configuration for $DEPLOYMENT_MODE deployment..."
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

echo "Successfully configured nginx.conf for domain: $DOMAIN_NAME ($DEPLOYMENT_MODE deployment)"

# Show what we generated
echo ""
echo "Generated nginx configuration preview:"
echo "======================================"
head -20 "$OUTPUT_FILE"
echo "..."
echo "======================================"
echo ""
echo "Full configuration written to: $OUTPUT_FILE"

# SSL Certificate Validation (from nginx-entrypoint.sh logic)
validate_ssl_certificates() {
    if [ "$USE_SSL" = "true" ]; then
        echo ""
        echo "🔒 Validating SSL certificates..."
        
        # Check for Let's Encrypt certificates
        if [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" ] && [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem" ]; then
            echo "✅ Using Let's Encrypt certificates for $DOMAIN_NAME"
            
            # Verify certificate is not expired
            if openssl x509 -in "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" -noout -checkend 0 2>/dev/null; then
                echo "✅ Certificate is valid"
                return 0
            else
                echo "⚠️  Certificate may be expired - please renew"
                return 1
            fi
        else
            echo "❌ SSL enabled but no Let's Encrypt certificates found for $DOMAIN_NAME"
            echo "   Please run: sudo certbot --nginx -d $DOMAIN_NAME -d www.$DOMAIN_NAME"
            return 1
        fi
    else
        echo "ℹ️  SSL disabled, skipping certificate validation"
        return 0
    fi
}

# Nginx Configuration Validation
validate_nginx_config() {
    echo ""
    echo "🧪 Testing nginx configuration..."
    
    # Test nginx configuration
    if command -v nginx >/dev/null 2>&1; then
        if nginx -t -c "$OUTPUT_FILE" 2>/dev/null; then
            echo "✅ Nginx configuration is valid"
            return 0
        else
            echo "❌ Nginx configuration test failed"
            echo "   Run 'nginx -t -c $OUTPUT_FILE' for details"
            return 1
        fi
    else
        echo "ℹ️  Nginx not installed, skipping configuration test"
        return 0
    fi
}

# Run validation if requested
if [ "$VALIDATE_MODE" = true ]; then
    echo ""
    echo "🔍 Running validation checks..."
    
    VALIDATION_FAILED=false
    
    # Validate SSL certificates
    if ! validate_ssl_certificates; then
        VALIDATION_FAILED=true
    fi
    
    # Validate nginx configuration
    if ! validate_nginx_config; then
        VALIDATION_FAILED=true
    fi
    
    if [ "$VALIDATION_FAILED" = true ]; then
        echo ""
        echo "❌ Validation failed - see errors above"
        exit 1
    else
        echo ""
        echo "✅ All validation checks passed"
    fi
fi