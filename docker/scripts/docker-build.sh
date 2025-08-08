#!/bin/bash

# Tactile Teleop Docker Build Script
# Builds and deploys the containerized Tactile Teleop system

set -e

echo "=== Tactile Teleop Docker Deployment ==="
echo

# Parse command line arguments
ENVIRONMENT=${1:-production}
REBUILD=${2:-false}
ENV_FILE=${3:-}

# Set default environment file based on environment if not specified
if [ -z "$ENV_FILE" ]; then
    if [ "$ENVIRONMENT" = "prod" ] || [ "$ENVIRONMENT" = "production" ]; then
        ENV_FILE="production.env"
    else
        ENV_FILE="development.env"
    fi
fi

case $ENVIRONMENT in
    "dev"|"development")
        echo "🔧 Building for development environment"
        COMPOSE_FILE="docker-compose.yml:docker-compose.override.yml"
        SSL_ENABLED=false
        ;;
    "prod"|"production")
        echo "🚀 Building for production environment"
        COMPOSE_FILE="docker-compose.yml"
        
        # SSL Certificate Setup for Production
        echo
        echo "🔒 Checking SSL certificate configuration..."
        
        # Set domain configuration
        DOMAIN_NAME=${DOMAIN_NAME:-teleop.tactilerobotics.ai}
        EMAIL=${LETSENCRYPT_EMAIL:-zeno@tactilerobotics.ai}
        AUTO_SSL=${AUTO_SSL:-false}
        
        SSL_ENABLED=false
        
        # Check for Let's Encrypt certificates
        if [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" ] && [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem" ]; then
            echo "✅ Let's Encrypt certificates found for $DOMAIN_NAME"
            
            # Check certificate validity (not expiring in next 30 days)
            if openssl x509 -in "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" -noout -checkend 2592000 2>/dev/null; then
                echo "✅ Certificate is valid and not expiring soon"
                SSL_ENABLED=true
            else
                echo "⚠️  Let's Encrypt certificate is expired or expiring soon"
                SSL_ENABLED=true  # Still enable SSL, nginx will show the expired cert warning
            fi
        fi
        
        # If no valid certificates found, offer to set up Let's Encrypt
        if [ "$SSL_ENABLED" = false ]; then
            echo "⚠️  No valid SSL certificates found - will use self-signed certificates"
            echo "   This will cause browser security warnings"
            echo
            
            # Auto setup for CI/CD or if explicitly requested
            if [ "$AUTO_SSL" = "true" ]; then
                echo "🤖 Auto-setting up Let's Encrypt SSL certificates for $DOMAIN_NAME (AUTO_SSL=true)"
                REPLY="y"
            else
                read -p "🤖 Would you like to automatically set up Let's Encrypt SSL certificates for $DOMAIN_NAME? (y/N): " -r
            fi
            
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                echo "🔐 Setting up Let's Encrypt SSL certificates..."
                
                # Stop any running containers first
                echo "🛑 Stopping existing containers..."
                docker-compose down 2>/dev/null || true
                
                # Run Let's Encrypt setup
                if ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh "$DOMAIN_NAME" "$EMAIL"; then
                    echo "✅ SSL certificates obtained successfully!"
                    SSL_ENABLED=true
                else
                    echo "❌ SSL certificate setup failed. Continuing with self-signed certificates."
                    echo "   You can manually run: ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh $DOMAIN_NAME $EMAIL"
                fi
            else
                echo "ℹ️  Continuing with self-signed certificates"
                echo "   To set up SSL later, run: ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh $DOMAIN_NAME $EMAIL"
            fi
        fi
        
        # Export SSL_ENABLED for docker-compose
        if [ "$SSL_ENABLED" = true ]; then
            echo "🔒 Enabling SSL in Docker containers"
            export SSL_ENABLED=true
        else
            export SSL_ENABLED=false
        fi
        ;;
    *)
        echo "Usage: $0 [dev|prod] [rebuild] [env_file]"
        echo "  dev/development: Include development overrides"
        echo "  prod/production: Production configuration only (includes SSL setup)"
        echo "  rebuild: Force rebuild of all images"
        echo "  env_file: Environment file to use (default: development.env)"
        echo ""
        echo "Environment Variables for SSL (Production):"
        echo "  DOMAIN_NAME: Domain for SSL certificate (default: teleop.tactilerobotics.ai)"
        echo "  LETSENCRYPT_EMAIL: Email for Let's Encrypt (default: zeno@tactilerobotics.ai)"
        echo "  AUTO_SSL: Auto-setup SSL without prompt (default: false)"
        echo ""
        echo "Examples:"
        echo "  $0 dev                    # Development with development.env"
        echo "  $0 prod rebuild           # Production rebuild with SSL setup"
        echo "  $0 prod false production.env  # Production with production.env"
        echo "  AUTO_SSL=true $0 prod     # Production with automatic SSL setup"
        exit 1
        ;;
esac

# Handle rebuild flag
BUILD_FLAGS=""
if [ "$REBUILD" = "rebuild" ] || [ "$REBUILD" = "true" ]; then
    echo "🔄 Forcing rebuild of all images"
    BUILD_FLAGS="--build --no-cache"
fi

echo "📦 Docker Compose files: $COMPOSE_FILE"
echo "🔒 SSL Enabled: $SSL_ENABLED"
echo "🔧 Environment file: $ENV_FILE"

# Load environment variables for this script
if [ -f "$ENV_FILE" ]; then
    set -a
    source "$ENV_FILE"
    set +a
fi

echo

# Check Docker installation
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed. Please install Docker first."
    exit 1
fi

if ! command -v docker-compose &> /dev/null; then
    echo "❌ Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

# Configure nginx based on environment variables
echo "⚙️  Configuring nginx with $ENV_FILE..."
IN_DOCKER=true SSL_ENABLED=$SSL_ENABLED ./src/tactile_teleop/web_server/nginx/configure-nginx.sh "$ENV_FILE" docker

echo
# Build and start services
echo "🏗️  Building and starting services..."
export ENV_FILE
export DOMAIN_NAME
export SSL_ENABLED
COMPOSE_FILE=$COMPOSE_FILE docker-compose up -d $BUILD_FLAGS

echo
echo "⏳ Waiting for services to be healthy..."
sleep 10

# Check service status
echo "📊 Service Status:"
docker-compose ps

echo
echo "🔍 Health Checks:"

# Test nginx (using configured HTTPS port)
HTTPS_PORT=${NGINX_HTTPS_PORT:-8443}
if curl -k -f -s https://localhost:$HTTPS_PORT/health > /dev/null; then
    echo "✅ Nginx proxy: Healthy"
    
    # SSL Certificate validation if SSL is enabled
    if [ "$SSL_ENABLED" = true ]; then
        echo "🔍 Validating SSL certificate..."
        
        # Check certificate expiration
        if echo | openssl s_client -servername "$DOMAIN_NAME" -connect localhost:$HTTPS_PORT 2>/dev/null | openssl x509 -noout -checkend 2592000 2>/dev/null; then
            echo "✅ SSL certificate: Valid and not expiring soon"
        else
            echo "⚠️  SSL certificate: May be expired or expiring soon"
        fi
        
        # Check certificate details
        echo "📋 SSL certificate details:"
        echo | openssl s_client -servername "$DOMAIN_NAME" -connect localhost:$HTTPS_PORT 2>/dev/null | openssl x509 -noout -subject -dates 2>/dev/null | head -2
    fi
else
    echo "❌ Nginx proxy: Unhealthy"
fi

# Test FastAPI directly (in dev mode)
if [ "$ENVIRONMENT" = "dev" ] || [ "$ENVIRONMENT" = "development" ]; then
    if curl -f -s http://localhost:8000/health > /dev/null; then
        echo "✅ FastAPI direct: Healthy"
    else
        echo "❌ FastAPI direct: Unhealthy"
    fi
fi

echo
echo "=== Deployment Complete ==="
echo
echo "🌐 Web Interface: https://localhost:$HTTPS_PORT/"
echo "📋 Health Check: https://localhost:$HTTPS_PORT/health"

if [ "$ENVIRONMENT" = "dev" ] || [ "$ENVIRONMENT" = "development" ]; then
    echo "🔧 FastAPI Direct: http://localhost:8000/"
    echo "📝 Development mode: Live code reload enabled"
fi

echo
echo "📖 Useful commands:"
echo "  View logs:    docker-compose logs -f"
echo "  Stop services: docker-compose down"
echo "  Restart:      docker-compose restart"
echo
echo "📚 See DOCKER-DEPLOYMENT.md for detailed documentation"