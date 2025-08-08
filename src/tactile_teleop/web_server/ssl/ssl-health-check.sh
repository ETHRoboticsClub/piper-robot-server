#!/bin/bash

# SSL Health Check and Renewal Script
# Checks SSL certificate status and handles renewal if needed

set -e

DOMAIN_NAME=${1:-teleop.tactilerobotics.ai}
RENEWAL_DAYS=${2:-30}
RESTART_CONTAINERS=${3:-true}

echo "=== SSL Health Check ==="
echo "Domain: $DOMAIN_NAME"
echo "Renewal threshold: $RENEWAL_DAYS days"
echo

# Check if Let's Encrypt certificates exist
CERT_PATH="/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem"
KEY_PATH="/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem"

if [ ! -f "$CERT_PATH" ] || [ ! -f "$KEY_PATH" ]; then
    echo "❌ No Let's Encrypt certificates found for $DOMAIN_NAME"
    echo "   Run: ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh $DOMAIN_NAME"
    exit 1
fi

echo "✅ Let's Encrypt certificates found"

# Check certificate expiration
echo "🔍 Checking certificate expiration..."
EXPIRY_SECONDS=$(date -d "$(openssl x509 -in "$CERT_PATH" -noout -enddate | cut -d= -f2)" +%s)
CURRENT_SECONDS=$(date +%s)
DAYS_UNTIL_EXPIRY=$(( (EXPIRY_SECONDS - CURRENT_SECONDS) / 86400 ))

echo "📅 Certificate expires in $DAYS_UNTIL_EXPIRY days"

if [ $DAYS_UNTIL_EXPIRY -le $RENEWAL_DAYS ]; then
    echo "⚠️  Certificate expires in $DAYS_UNTIL_EXPIRY days (threshold: $RENEWAL_DAYS days)"
    echo "🔄 Attempting certificate renewal..."
    
    # Attempt renewal
    if sudo certbot renew --quiet; then
        echo "✅ Certificate renewal successful"
        
        # Restart containers if requested (certificates are directly mounted)
        if [ "$RESTART_CONTAINERS" = "true" ]; then
            echo "🔄 Restarting nginx container to reload certificates..."
            cd "$(dirname "${BASH_SOURCE[0]}")/../.."
            if docker-compose ps nginx | grep -q "Up"; then
                docker-compose restart nginx
                echo "✅ Nginx container restarted"
            else
                echo "ℹ️  Nginx container not running, no restart needed"
            fi
        fi
        
    else
        echo "❌ Certificate renewal failed"
        exit 1
    fi
else
    echo "✅ Certificate is valid and not due for renewal"
fi

# Validate certificate chain
echo "🔗 Validating certificate chain..."
if openssl verify -CAfile <(curl -s https://letsencrypt.org/certs/isrgrootx1.pem) "$CERT_PATH" >/dev/null 2>&1; then
    echo "✅ Certificate chain is valid"
else
    echo "⚠️  Certificate chain validation failed"
fi

# Check certificate details
echo "📋 Certificate details:"
openssl x509 -in "$CERT_PATH" -noout -subject -issuer -dates

echo
echo "🎉 SSL health check completed"