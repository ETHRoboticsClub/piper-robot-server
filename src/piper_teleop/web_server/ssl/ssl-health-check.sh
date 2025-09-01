#!/bin/bash

# SSL Health Check and Renewal Script
# Checks SSL certificate status and handles renewal if needed
# Uses local root certificate for secure validation to prevent MITM attacks

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
    echo "   Run: ./src/piper_teleop/web_server/ssl/setup-letsencrypt.sh $DOMAIN_NAME"
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

# Use local root certificate for secure validation
SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
ROOT_CERT_PATH="$SCRIPT_DIR/isrgrootx1.pem"

# Note: Local root certificate provides additional validation but is optional
# The system CA bundle (/etc/ssl/certs) is the primary trust anchor

# Perform certificate chain validation if root certificate is available
if [ -f "$ROOT_CERT_PATH" ]; then
    # Check if local root certificate is still valid (not expired)
    ROOT_EXPIRY=$(openssl x509 -in "$ROOT_CERT_PATH" -noout -enddate | cut -d= -f2)
    ROOT_EXPIRY_SECONDS=$(date -d "$ROOT_EXPIRY" +%s 2>/dev/null || echo 0)
    CURRENT_SECONDS=$(date +%s)
    
    if [ $ROOT_EXPIRY_SECONDS -gt $CURRENT_SECONDS ]; then
        if openssl verify -CAfile "$ROOT_CERT_PATH" "$CERT_PATH" >/dev/null 2>&1; then
            echo "✅ Certificate chain is valid"
        else
            echo "⚠️  Certificate chain validation failed with local root"
            
            # Additional verification using system CA bundle as fallback
            echo "🔄 Trying fallback verification with system CA bundle..."
            if openssl verify -CApath /etc/ssl/certs "$CERT_PATH" >/dev/null 2>&1; then
                echo "✅ Certificate validated against system CA bundle"
            else
                echo "❌ Certificate validation failed with both methods"
            fi
        fi
    else
        echo "⚠️  Local root certificate has expired ($(date -d "$ROOT_EXPIRY" +%Y-%m-%d))"
        echo "🔄 Using system CA bundle for validation..."
        if openssl verify -CApath /etc/ssl/certs "$CERT_PATH" >/dev/null 2>&1; then
            echo "✅ Certificate validated against system CA bundle"
            echo "💡 Consider updating local root certificate"
        else
            echo "❌ Certificate validation failed"
        fi
    fi
else
    echo "⚠️  Certificate chain validation skipped (no root certificate available)"
    echo "🔄 Using system CA bundle for validation..."
    if openssl verify -CApath /etc/ssl/certs "$CERT_PATH" >/dev/null 2>&1; then
        echo "✅ Certificate validated against system CA bundle"
    else
        echo "❌ Certificate validation failed"
    fi
fi

# Check certificate details
echo "📋 Certificate details:"
openssl x509 -in "$CERT_PATH" -noout -subject -issuer -dates

# Display root certificate status if available
if [ -f "$ROOT_CERT_PATH" ]; then
    ROOT_EXPIRY=$(openssl x509 -in "$ROOT_CERT_PATH" -noout -enddate | cut -d= -f2)
    ROOT_DAYS_LEFT=$(( ($(date -d "$ROOT_EXPIRY" +%s) - $(date +%s)) / 86400 ))
    echo "📋 Local root certificate expires in $ROOT_DAYS_LEFT days ($ROOT_EXPIRY)"
    
    # Warn if root certificate is approaching expiration (within 1 year)
    if [ $ROOT_DAYS_LEFT -lt 365 ]; then
        echo "⚠️  Local root certificate expires soon - consider updating"
    fi
fi

echo
echo "🎉 SSL health check completed"