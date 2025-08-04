#!/bin/bash
# Setup Let's Encrypt SSL certificates for Tactile Teleop

set -e

DOMAIN_NAME=${1:-teleop.tactilerobotics.ai}
EMAIL=${2:-info@tactilerobotics.ai}

echo "🔐 Setting up Let's Encrypt SSL certificate for $DOMAIN_NAME"

# Check if domain resolves to this server
echo "🌐 Checking DNS resolution for $DOMAIN_NAME..."
if ! nslookup $DOMAIN_NAME > /dev/null 2>&1; then
    echo "❌ Error: Domain $DOMAIN_NAME does not resolve. Please ensure DNS is configured correctly."
    exit 1
fi

# Install certbot if not already installed
if ! command -v certbot &> /dev/null; then
    echo "📦 Installing certbot..."
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        sudo apt-get update
        sudo apt-get install -y certbot
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        brew install certbot
    else
        echo "❌ Unsupported OS. Please install certbot manually."
        exit 1
    fi
fi

# Create SSL directory
mkdir -p ./ssl

# Stop any running containers that might be using ports 80/443
echo "🛑 Stopping existing containers..."
docker-compose down || true

# Use certbot standalone mode to get certificate
echo "🔒 Obtaining SSL certificate from Let's Encrypt..."
sudo certbot certonly \
    --standalone \
    --email $EMAIL \
    --agree-tos \
    --no-eff-email \
    --domains $DOMAIN_NAME \
    --cert-path ./ssl/cert.pem \
    --key-path ./ssl/key.pem \
    --fullchain-path ./ssl/fullchain.pem \
    --chain-path ./ssl/chain.pem

# Copy certificates to local ssl directory
echo "📋 Copying certificates..."
sudo cp /etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem ./ssl/cert.pem
sudo cp /etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem ./ssl/key.pem
sudo chown $USER:$USER ./ssl/cert.pem ./ssl/key.pem

echo "✅ SSL certificate obtained successfully!"
echo "📁 Certificates saved to ./ssl/"
echo ""
echo "🔄 To use these certificates, uncomment the SSL volume mounts in docker-compose.yml:"
echo "    # - ./ssl/cert.pem:/etc/ssl/certs/tactile-teleop.crt:ro"
echo "    # - ./ssl/key.pem:/etc/ssl/private/tactile-teleop.key:ro"
echo ""
echo "⚠️  Remember to set up automatic renewal with: sudo crontab -e"
echo "    Add: 0 12 * * * /usr/bin/certbot renew --quiet"