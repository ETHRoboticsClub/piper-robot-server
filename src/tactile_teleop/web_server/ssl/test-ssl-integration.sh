#!/bin/bash

# SSL Integration Test Script
# Tests the complete SSL setup and Docker integration

set -e

DOMAIN_NAME=${1:-teleop.tactilerobotics.ai}
TEST_MODE=${2:-dry-run}

echo "=== SSL Integration Test ==="
echo "Domain: $DOMAIN_NAME"
echo "Test mode: $TEST_MODE"
echo

# Test 1: Check if Let's Encrypt certificates exist
echo "🧪 Test 1: Let's Encrypt Certificate Detection"
if [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" ] && [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem" ]; then
    echo "✅ Let's Encrypt certificates found"
    
    # Check validity
    if openssl x509 -in "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" -noout -checkend 2592000 2>/dev/null; then
        echo "✅ Certificate is valid and not expiring soon"
    else
        echo "⚠️  Certificate may be expired or expiring soon"
    fi
else
    echo "❌ No Let's Encrypt certificates found"
    echo "   Run: sudo certbot --nginx -d $DOMAIN_NAME -d www.$DOMAIN_NAME"
fi

echo

# Test 2: Let's Encrypt certificate accessibility
echo "🧪 Test 2: Certificate Accessibility for Docker"
CERT_DIR="/etc/letsencrypt/live/$DOMAIN_NAME"
if [ -d "$CERT_DIR" ]; then
    echo "✅ Let's Encrypt certificate directory exists"
    
    if [ -r "$CERT_DIR/fullchain.pem" ] && [ -r "$CERT_DIR/privkey.pem" ]; then
        echo "✅ Certificates are readable for Docker mounting"
        
        # Check certificate validity
        if openssl x509 -in "$CERT_DIR/fullchain.pem" -noout -checkend 0 2>/dev/null; then
            echo "✅ Certificate is currently valid"
        else
            echo "⚠️  Certificate may be expired"
        fi
    else
        echo "❌ Certificates not readable (permission issue?)"
        ls -la "$CERT_DIR/" 2>/dev/null || echo "   Directory not accessible"
    fi
else
    echo "❌ Let's Encrypt certificate directory not found"
fi

echo

# Test 3: Nginx configuration generation
echo "🧪 Test 3: Nginx Configuration"
if [ -f "src/tactile_teleop/web_server/nginx/nginx-tactile-teleop.conf" ]; then
    echo "✅ Generated nginx configuration exists"
    
    # Check if it contains SSL configuration
    if grep -q "ssl_certificate" src/tactile_teleop/web_server/nginx/nginx-tactile-teleop.conf; then
        echo "✅ SSL configuration found in nginx config"
    else
        echo "❌ No SSL configuration in nginx config"
    fi
else
    echo "❌ Generated nginx configuration not found"
    echo "   Run: ./src/tactile_teleop/web_server/nginx/configure-nginx.sh production.env docker"
fi

echo

# Test 4: Docker Compose SSL configuration
echo "🧪 Test 4: Docker Compose SSL Integration"
if [ -f "docker-compose.yml" ]; then
    echo "✅ Main Docker Compose file exists"
    
    # Check for Let's Encrypt volume mount
    if grep -q "/etc/letsencrypt" docker-compose.yml; then
        echo "✅ Let's Encrypt volume mount configured in main compose file"
    else
        echo "❌ Let's Encrypt volume mount not found in docker-compose.yml"
    fi
    
    # Check for SSL environment variables
    if grep -q "SSL_ENABLED" docker-compose.yml; then
        echo "✅ SSL environment variables configured"
    else
        echo "❌ SSL environment variables not found"
    fi
else
    echo "❌ Main Docker Compose file not found"
fi

echo

# Test 5: SSL setup script
echo "🧪 Test 5: SSL Setup Script"
if [ -x "src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh" ]; then
    echo "✅ SSL setup script exists and is executable"
    
    # Test help output
    if ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh --help | grep -q "Usage:"; then
        echo "✅ SSL setup script help works"
    else
        echo "❌ SSL setup script help failed"
    fi
else
    echo "❌ SSL setup script not found or not executable"
fi

echo

# Test 6: Docker build script integration
echo "🧪 Test 6: Docker Build Script SSL Integration"
if grep -q "Let's Encrypt certificates found" docker/scripts/docker-build.sh; then
    echo "✅ Docker build script has SSL detection"
else
    echo "❌ Docker build script missing SSL detection"
fi

if grep -q "SSL_ENABLED" docker/scripts/docker-build.sh; then
    echo "✅ Docker build script handles SSL_ENABLED variable"
else
    echo "❌ Docker build script missing SSL_ENABLED handling"
fi

echo

# Test 7: Simulate Docker deployment (dry run)
if [ "$TEST_MODE" = "full" ]; then
    echo "🧪 Test 7: Full Docker Deployment Test"
    echo "⚠️  This will stop and restart containers"
    read -p "Continue? (y/N): " -r
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "🚀 Testing production deployment..."
        ./docker/scripts/docker-build.sh prod false production.env
        
        echo "⏳ Waiting for services to start..."
        sleep 15
        
        # Test HTTPS endpoint
        HTTPS_PORT=${NGINX_HTTPS_PORT:-8443}
        if curl -k -f -s https://localhost:$HTTPS_PORT/health > /dev/null; then
            echo "✅ HTTPS endpoint responding"
        else
            echo "❌ HTTPS endpoint not responding"
        fi
        
        echo "🛑 Stopping test deployment..."
        docker-compose down
    else
        echo "ℹ️  Skipping full deployment test"
    fi
else
    echo "🧪 Test 7: Dry Run Mode (use 'full' to test actual deployment)"
    echo "✅ Would test Docker deployment with SSL"
fi

echo
echo "=== Test Summary ==="
echo "🎉 SSL integration test completed"
echo "ℹ️  For full testing, run: $0 $DOMAIN_NAME full"