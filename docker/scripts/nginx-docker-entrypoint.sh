#!/bin/sh

# Docker-specific nginx validation entrypoint
# This script runs inside the nginx container to validate SSL and nginx config before starting

set -e

echo "🔒 Configuring nginx based on runtime SSL certificate availability..."

# Environment variables
SSL_ENABLED=${SSL_ENABLED:-false}
DOMAIN_NAME=${DOMAIN_NAME:-teleop.tactilerobotics.ai}
FASTAPI_BACKEND="fastapi:8000"
WEB_ROOT="/usr/share/nginx/html"
CERTBOT_ROOT="/var/www/certbot"

# Runtime SSL detection - override SSL_ENABLED if certificates are not available
CERTS_AVAILABLE=false
if [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" ] && [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem" ]; then
    CERTS_AVAILABLE=true
fi

# Determine final SSL configuration
if [ "$SSL_ENABLED" = "true" ] && [ "$CERTS_AVAILABLE" = "true" ]; then
    USE_SSL=true
    echo "✅ SSL enabled and certificates available - using HTTPS configuration"
    
    # Verify certificate is not expired
    if openssl x509 -in "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" -noout -checkend 0 2>/dev/null; then
        echo "✅ Certificate is valid"
    else
        echo "⚠️  Certificate may be expired - please renew"
    fi
elif [ "$SSL_ENABLED" = "true" ] && [ "$CERTS_AVAILABLE" = "false" ]; then
    USE_SSL=false
    echo "⚠️  SSL enabled but certificates not available - falling back to HTTP"
else
    USE_SSL=false
    echo "ℹ️  Using HTTP configuration"
fi

# Generate nginx configuration at runtime
echo "🔧 Generating nginx configuration..."
if [ "$USE_SSL" = "true" ]; then
    TEMPLATE="/etc/nginx/templates/nginx.conf.prod.template"
    echo "📋 Using production HTTPS template"
else
    TEMPLATE="/etc/nginx/templates/nginx.conf.dev.template"
    echo "📋 Using development HTTP template"
fi

# Generate configuration using envsubst
export DOMAIN_NAME FASTAPI_BACKEND WEB_ROOT CERTBOT_ROOT
envsubst '$DOMAIN_NAME $FASTAPI_BACKEND $WEB_ROOT $CERTBOT_ROOT' < "$TEMPLATE" > /etc/nginx/conf.d/tactile-teleop.conf

echo "✅ Generated nginx configuration for $DOMAIN_NAME (SSL: $USE_SSL)"

# Test nginx configuration
echo "🧪 Testing nginx configuration..."
if nginx -t; then
    echo "✅ Nginx configuration is valid"
else
    echo "❌ Nginx configuration test failed"
    exit 1
fi

echo "🚀 SSL setup complete, starting nginx..."