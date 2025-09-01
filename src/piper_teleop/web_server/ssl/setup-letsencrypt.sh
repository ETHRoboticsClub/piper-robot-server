#!/bin/bash

# Let's Encrypt SSL Certificate Setup Script
# Sets up SSL certificates for the Tactile Teleop system using Let's Encrypt

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

DOMAIN_NAME=${1:-teleop.tactilerobotics.ai}
EMAIL=${2:-zeno@tactilerobotics.ai}
STAGING=${3:-false}
FORCE_RENEWAL=${4:-false}

echo "=== Let's Encrypt SSL Setup ==="
echo "Domain: $DOMAIN_NAME"
echo "Email: $EMAIL"
echo "Staging: $STAGING"
echo

# Check if certificates already exist and are valid
check_existing_certificates() {
    local cert_path="/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem"
    local key_path="/etc/letsencrypt/live/$DOMAIN_NAME/privkey.pem"
    
    if [ -f "$cert_path" ] && [ -f "$key_path" ]; then
        echo "🔍 Existing certificates found, checking validity..."
        
        # Check if certificate is valid and not expiring soon (30 days)
        if openssl x509 -in "$cert_path" -noout -checkend 2592000 2>/dev/null; then
            echo "✅ Existing certificate is valid and not expiring soon"
            if [ "$FORCE_RENEWAL" != "true" ]; then
                echo "ℹ️  Use FORCE_RENEWAL=true to force renewal"
                return 0
            else
                echo "🔄 Force renewal requested"
                return 1
            fi
        else
            echo "⚠️  Certificate is expired or expiring soon"
            return 1
        fi
    else
        echo "📋 No existing certificates found"
        return 1
    fi
}

# Backup existing certificates
backup_certificates() {
    if [ -d "/etc/letsencrypt/live/$DOMAIN_NAME" ]; then
        local backup_dir="/etc/letsencrypt/backup/$(date +%Y%m%d_%H%M%S)"
        echo "📦 Backing up existing certificates to $backup_dir"
        sudo mkdir -p "$backup_dir"
        sudo cp -r "/etc/letsencrypt/live/$DOMAIN_NAME" "$backup_dir/"
        echo "✅ Backup completed"
    fi
}

# Request SSL certificate using the simple nginx method
request_certificate() {
    local staging_flag=""
    if [ "$STAGING" = "true" ]; then
        staging_flag="--staging"
        echo "⚠️  Using Let's Encrypt staging environment"
    fi
    
    echo "🔐 Requesting SSL certificate with nginx integration..."
    
    # Install certbot if not present
    if ! command -v certbot &> /dev/null; then
        echo "📦 Installing certbot..."
        sudo apt-get update
        sudo apt-get install -y certbot python3-certbot-nginx
    fi
    
    # Use the simple nginx method for certificate request and deployment
    sudo certbot --nginx \
        --email "$EMAIL" \
        --agree-tos \
        --no-eff-email \
        $staging_flag \
        -d "$DOMAIN_NAME" \
        -d "www.$DOMAIN_NAME" \
        --non-interactive
    
    if [ $? -eq 0 ]; then
        echo "✅ SSL certificate obtained and deployed successfully!"
        return 0
    else
        echo "❌ Failed to obtain SSL certificate"
        return 1
    fi
}

# Validate Docker can access certificates
validate_docker_access() {
    echo "📋 Validating Docker access to Let's Encrypt certificates..."
    
    local cert_dir="/etc/letsencrypt/live/$DOMAIN_NAME"
    
    # Check if certificates are readable
    if [ -r "$cert_dir/fullchain.pem" ] && [ -r "$cert_dir/privkey.pem" ]; then
        echo "✅ Certificates are accessible for Docker mounting"
    else
        echo "⚠️  Certificates may not be accessible - check permissions"
        ls -la "$cert_dir/"
    fi
}

# Set up automatic renewal
setup_renewal() {
    echo "🔄 Setting up automatic certificate renewal..."
    
    # Create renewal hook script
    sudo tee /etc/letsencrypt/renewal-hooks/deploy/docker-restart.sh > /dev/null << EOF
#!/bin/bash
# Restart Docker containers after certificate renewal

# Restart nginx container to reload certificates
cd "$PROJECT_ROOT"
if docker-compose ps nginx | grep -q "Up"; then
    echo "Restarting nginx container after certificate renewal..."
    docker-compose restart nginx
fi
EOF

    sudo chmod +x /etc/letsencrypt/renewal-hooks/deploy/docker-restart.sh
    
    # Test renewal (dry run)
    echo "🧪 Testing certificate renewal..."
    sudo certbot renew --dry-run
    
    echo "✅ Automatic renewal configured"
}

# Validate certificate
validate_certificate() {
    local cert_path="/etc/letsencrypt/live/$DOMAIN_NAME/fullchain.pem"
    
    if [ -f "$cert_path" ]; then
        echo "🔍 Validating certificate..."
        
        # Check certificate details
        echo "📋 Certificate details:"
        openssl x509 -in "$cert_path" -noout -subject -dates -issuer
        
        # Test HTTPS connection
        echo "🌐 Testing HTTPS connection..."
        if curl -Is "https://$DOMAIN_NAME" | head -1 | grep -q "200 OK"; then
            echo "✅ HTTPS connection successful"
            return 0
        else
            echo "⚠️  HTTPS connection test failed (this is normal if nginx isn't configured yet)"
            return 0
        fi
    else
        echo "❌ Certificate file not found"
        return 1
    fi
}

# Main execution
main() {
    echo "🚀 Starting Let's Encrypt SSL setup..."
    
    # Check if running as root for some operations
    if [ "$EUID" -eq 0 ]; then
        echo "⚠️  Running as root - this is not recommended for Docker operations"
    fi
    
    # Check if certificates already exist and are valid
    if check_existing_certificates; then
        echo "✅ Valid certificates already exist, nothing to do"
        validate_docker_access
        return 0
    fi
    
    # Backup existing certificates if they exist
    backup_certificates
    
    # Request certificate using simple nginx method
    if request_certificate; then
        # Validate Docker access
        validate_docker_access
        
        # Set up automatic renewal
        setup_renewal
        
        # Validate certificate
        validate_certificate
        
        echo "🎉 SSL setup completed successfully!"
        echo "📋 Certificates are available at:"
        echo "   System: /etc/letsencrypt/live/$DOMAIN_NAME/"
        echo "   Docker: Direct mount from /etc/letsencrypt/"
        
    else
        echo "❌ SSL setup failed"
        exit 1
    fi
}

# Handle script arguments
case "${1:-}" in
    --help|-h)
        echo "Usage: $0 [DOMAIN] [EMAIL] [STAGING] [FORCE_RENEWAL]"
        echo ""
        echo "Arguments:"
        echo "  DOMAIN        Domain name (default: teleop.tactilerobotics.ai)"
        echo "  EMAIL         Email for Let's Encrypt (default: zeno@tactilerobotics.ai)"
        echo "  STAGING       Use staging environment (default: false)"
        echo "  FORCE_RENEWAL Force certificate renewal (default: false)"
        echo ""
        echo "Examples:"
        echo "  $0                                    # Use defaults"
        echo "  $0 example.com admin@example.com     # Custom domain and email"
        echo "  $0 example.com admin@example.com true # Use staging environment"
        echo "  $0 example.com admin@example.com false true # Force renewal"
        exit 0
        ;;
    *)
        main "$@"
        ;;
esac