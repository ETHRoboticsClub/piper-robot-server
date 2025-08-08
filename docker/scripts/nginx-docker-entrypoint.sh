#!/bin/sh

# Docker-specific nginx validation entrypoint
# This script runs inside the nginx container to validate SSL and nginx config before starting

set -e

echo "🔒 Validating SSL setup for nginx container..."

# Environment variables
SSL_ENABLED=${SSL_ENABLED:-false}
DOMAIN_NAME=${DOMAIN_NAME:-teleop.tactilerobotics.ai}

if [ "$SSL_ENABLED" = "true" ]; then
    echo "✅ SSL enabled, checking Let's Encrypt certificates..."
    
    # Check for Let's Encrypt certificates
    if [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" ] && [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem" ]; then
        echo "✅ Using Let's Encrypt certificates for $DOMAIN_NAME"
        
        # Verify certificate is not expired
        if openssl x509 -in "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" -noout -checkend 0 2>/dev/null; then
            echo "✅ Certificate is valid"
        else
            echo "⚠️  Certificate may be expired - please renew"
        fi
    else
        echo "❌ SSL enabled but no Let's Encrypt certificates found for $DOMAIN_NAME"
        echo "   Please run: sudo certbot --nginx -d $DOMAIN_NAME -d www.$DOMAIN_NAME"
        exit 1
    fi
else
    echo "ℹ️  SSL disabled, using HTTP only"
fi

# Test nginx configuration
echo "🧪 Testing nginx configuration..."
if nginx -t; then
    echo "✅ Nginx configuration is valid"
else
    echo "❌ Nginx configuration test failed"
    exit 1
fi

echo "🚀 SSL setup complete, starting nginx..."