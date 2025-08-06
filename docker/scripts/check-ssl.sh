#!/bin/bash
# Check SSL certificate status for Tactile Teleop

DOMAIN_NAME=${1:-teleop.tactilerobotics.ai}
PORT=${2:-443}

echo "🔍 Checking SSL certificate for $DOMAIN_NAME:$PORT"
echo "================================================"

# Check if domain resolves
echo "🌐 DNS Resolution:"
nslookup $DOMAIN_NAME || echo "❌ DNS resolution failed"
echo ""

# Check SSL certificate
echo "🔒 SSL Certificate Details:"
echo | openssl s_client -servername $DOMAIN_NAME -connect $DOMAIN_NAME:$PORT 2>/dev/null | openssl x509 -noout -text | grep -E "(Issuer|Subject|Not Before|Not After|DNS:)"
echo ""

# Check certificate expiration
echo "📅 Certificate Expiration:"
echo | openssl s_client -servername $DOMAIN_NAME -connect $DOMAIN_NAME:$PORT 2>/dev/null | openssl x509 -noout -dates
echo ""

# Check if certificate is self-signed
echo "🏢 Certificate Authority:"
CERT_INFO=$(echo | openssl s_client -servername $DOMAIN_NAME -connect $DOMAIN_NAME:$PORT 2>/dev/null | openssl x509 -noout -issuer -subject)
ISSUER=$(echo "$CERT_INFO" | grep "issuer=" | cut -d'=' -f2-)
SUBJECT=$(echo "$CERT_INFO" | grep "subject=" | cut -d'=' -f2-)

echo "   Issuer:  $ISSUER"
echo "   Subject: $SUBJECT"

if [[ "$ISSUER" == "$SUBJECT" ]]; then
    echo "⚠️  SELF-SIGNED certificate detected"
    echo "   This will cause browser security warnings"
    echo "   Consider using Let's Encrypt or a commercial CA"
else
    # Check if it's a well-known CA
    if echo "$ISSUER" | grep -qE "(Let's Encrypt|DigiCert|GlobalSign|GeoTrust|Comodo|VeriSign)"; then
        echo "✅ Certificate issued by trusted CA"
    else
        echo "⚠️  Certificate issued by: $ISSUER"
        echo "   May be self-signed or from unknown CA"
    fi
fi
echo ""

# Check certificate chain
echo "🔗 Certificate Chain:"
echo | openssl s_client -servername $DOMAIN_NAME -connect $DOMAIN_NAME:$PORT -showcerts 2>/dev/null | grep -c "BEGIN CERTIFICATE" | xargs echo "Certificate count:"
echo ""

# Test HTTPS connection
echo "🌐 HTTPS Connection Test:"
if curl -s -I https://$DOMAIN_NAME > /dev/null 2>&1; then
    echo "✅ HTTPS connection successful"
else
    echo "❌ HTTPS connection failed"
fi