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
        
        # Check for Let's Encrypt certificates
        CERTS_PRESENT=false
        if [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" ] && [ -f "/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem" ]; then
            echo "✅ Let's Encrypt certificates found for $DOMAIN_NAME"
            CERTS_PRESENT=true
            
            # Check certificate validity (not expiring in next 30 days)
            if openssl x509 -in "/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem" -noout -checkend 2592000 2>/dev/null; then
                echo "✅ Certificate is valid and not expiring soon"
            else
                echo "⚠️  Let's Encrypt certificate is expired or expiring soon"
                # nginx will serve the existing certificate; consider renewal
            fi
        fi
        
        # If no valid certificates found, offer to set up Let's Encrypt
        if [ "$CERTS_PRESENT" = false ]; then
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
                    CERTS_PRESENT=true
                else
                    echo "❌ SSL certificate setup failed. Continuing with self-signed certificates."
                    echo "   You can manually run: ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh $DOMAIN_NAME $EMAIL"
                fi
            else
                echo "ℹ️  Continuing with self-signed certificates"
                echo "   To set up SSL later, run: ./src/tactile_teleop/web_server/ssl/setup-letsencrypt.sh $DOMAIN_NAME $EMAIL"
            fi
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
BUILD_IMAGES=false
BUILD_FLAGS=""
UP_FLAGS=""

if [ "$REBUILD" = "rebuild" ] || [ "$REBUILD" = "true" ]; then
    echo "🔄 Forcing rebuild of all images (explicit rebuild flag)"
    BUILD_IMAGES=true
    BUILD_FLAGS="--no-cache"
    UP_FLAGS="--force-recreate"
elif [ "$ENVIRONMENT" = "prod" ] || [ "$ENVIRONMENT" = "production" ]; then
    echo "🔄 Production deployment - always rebuilding without cache to ensure latest changes"
    
    # Complete Docker cleanup for production to ensure fresh build
    echo "🧹 Cleaning up existing containers and images..."
    docker-compose down --volumes --remove-orphans 2>/dev/null || true
    
    # Remove project-specific images
    echo "🗑️  Removing tactile-teleop images..."
    docker images | grep tactile-teleop | awk '{print $3}' | xargs -r docker rmi -f 2>/dev/null || true
    
    # Clean up unused Docker resources
    echo "🧽 Cleaning up unused Docker resources..."
    docker system prune -f --volumes 2>/dev/null || true
    
    BUILD_IMAGES=true
    BUILD_FLAGS="--no-cache"
    if [ "$FORCE_RECREATE" = "true" ]; then
        echo "🔄 Forcing container recreation (FORCE_RECREATE=true)"
        UP_FLAGS="--force-recreate"
    fi
else
    BUILD_IMAGES=true
    BUILD_FLAGS=""
fi

echo "📦 Docker Compose files: $COMPOSE_FILE"
echo "🔒 SSL: auto-detected at runtime by nginx"
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

echo
# Build and start services
echo "🏗️  Building and starting services..."
export ENV_FILE
export DOMAIN_NAME
export COMPOSE_FILE

# Build images if needed
if [ "$BUILD_IMAGES" = "true" ]; then
    echo "🔧 Building images with: docker-compose build $BUILD_FLAGS"
    if [ -n "$BUILD_FLAGS" ]; then
        docker-compose build $BUILD_FLAGS
    else
        docker-compose build
    fi
fi

# Start services
echo "🔧 Starting services with: docker-compose up -d $UP_FLAGS"
if [ -n "$UP_FLAGS" ]; then
    docker-compose up -d $UP_FLAGS
else
    docker-compose up -d
fi

echo
echo "⏳ Waiting for services to be healthy..."
sleep 10

# Check service status
echo "📊 Service Status:"
docker-compose ps

# If no services are running, show logs for debugging
if ! docker-compose ps | grep -q "Up"; then
    echo
    echo "🔍 No services appear to be running. Showing recent logs for debugging:"
    docker-compose logs --tail=20
fi

echo
echo "🔍 Health Checks:"

# Test nginx (HTTPS first, then HTTP fallback)
HTTPS_PORT=${NGINX_HTTPS_PORT:-8443}
HTTP_PORT=${NGINX_HTTP_PORT:-8080}
if curl -k -f -s https://localhost:$HTTPS_PORT/health > /dev/null; then
    echo "✅ Nginx proxy (HTTPS): Healthy"
    echo "🔍 Validating SSL certificate..."
    if echo | openssl s_client -servername "$DOMAIN_NAME" -connect localhost:$HTTPS_PORT 2>/dev/null | openssl x509 -noout -checkend 2592000 2>/dev/null; then
        echo "✅ SSL certificate: Valid and not expiring soon"
    else
        echo "⚠️  SSL certificate: May be expired or expiring soon"
    fi
    echo "📋 SSL certificate details:"
    echo | openssl s_client -servername "$DOMAIN_NAME" -connect localhost:$HTTPS_PORT 2>/dev/null | openssl x509 -noout -subject -dates 2>/dev/null | head -2
elif curl -f -s http://localhost:$HTTP_PORT/health > /dev/null; then
    echo "✅ Nginx proxy (HTTP): Healthy"
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