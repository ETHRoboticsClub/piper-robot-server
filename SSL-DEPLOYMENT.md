# SSL Deployment Guide

This guide explains how to deploy Tactile Teleop with SSL certificates using both Docker and direct deployment methods.

## Prerequisites

### For Docker Deployment:

- Docker and Docker Compose installed
- Domain name pointing to your server
- Root access for certificate generation

### For Direct Deployment:

- Ubuntu/Debian Linux system
- Nginx installed
- Conda/Miniconda with 'tactile-teleop' environment
- Domain name pointing to your server
- Root access for nginx and systemd configuration

## Quick Start

### 1. Development Deployment (HTTP only)

For local development or testing without SSL:

```bash
make deploy-dev
```

This uses `nginx.conf.dev` which serves HTTP on port 8080 and includes ACME challenge support.

### 2. Production Deployment with SSL

#### Step 1: Test Certificate Generation

First, test your setup in staging mode to avoid hitting Let's Encrypt rate limits:

```bash
make certbot-test DOMAINS="yourdomain.com www.yourdomain.com" EMAIL="admin@yourdomain.com"
```

If you see "Congratulations!" the test was successful.

#### Step 2: Generate Production Certificates

Generate real SSL certificates:

```bash
make certbot-prod DOMAINS="yourdomain.com www.yourdomain.com" EMAIL="admin@yourdomain.com"
```

#### Step 3: Deploy with SSL

Deploy the production configuration with SSL:

```bash
make deploy-prod
```

This uses `nginx.conf.prod` which:

- Redirects HTTP to HTTPS
- Redirects www to non-www
- Uses Let's Encrypt certificates
- Includes security headers

## Direct Deployment (Without Docker)

For deploying directly on Ubuntu/Debian without Docker containers:

### Step 1: Generate Configuration

```bash
# Generate nginx config for direct deployment (HTTP only)
./deploy-setup.sh yourdomain.com false

# Or for SSL production deployment
./deploy-setup.sh yourdomain.com true
```

### Step 2: Install and Configure Nginx

```bash
# Install nginx
sudo apt update && sudo apt install nginx

# Deploy nginx configuration
sudo cp src/tactile_teleop/web_server/nginx/nginx-tactile-teleop.conf /etc/nginx/sites-available/tactile-teleop
sudo ln -sf /etc/nginx/sites-available/tactile-teleop /etc/nginx/sites-enabled/
sudo nginx -t
```

### Step 3: Deploy Systemd Service

```bash
# Install systemd service
sudo cp src/tactile_teleop/web_server/nginx/tactile-teleop.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tactile-teleop
```

### Step 4: SSL Certificates (Production Only)

```bash
# Generate Let's Encrypt certificates
sudo bash src/tactile_teleop/web_server/register_ssl.sh \
  --domains "yourdomain.com www.yourdomain.com" \
  --email "admin@yourdomain.com" \
  --data-path "/etc/letsencrypt" \
  --staging 0
```

### Step 5: Start Services

```bash
# Start FastAPI service
sudo systemctl start tactile-teleop

# Start/restart nginx
sudo systemctl restart nginx

# Verify deployment
curl https://yourdomain.com/health
```

## Manual Certificate Management

If you prefer to use the SSL script directly:

```bash
# Test certificates
sudo bash src/tactile_teleop/web_server/register_ssl.sh \
  --domains "yourdomain.com www.yourdomain.com" \
  --email "admin@yourdomain.com" \
  --data-path "./certbot" \
  --staging 1

# Production certificates
sudo bash src/tactile_teleop/web_server/register_ssl.sh \
  --domains "yourdomain.com www.yourdomain.com" \
  --email "admin@yourdomain.com" \
  --data-path "./certbot" \
  --staging 0
```

## Certificate Renewal

Certificates are automatically renewed by the certbot container every 12 hours. Nginx reloads every 6 hours to pick up new certificates.

## Troubleshooting

### Clean Start

If you need to remove certificates and start fresh:

```bash
make clean-certs
```

### Check Logs

View container logs:

```bash
make logs
```

### Manual Nginx Restart

If certificates are updated but not loaded:

```bash
make restart-nginx
```

## File Structure

```
certbot/
├── conf/              # Let's Encrypt certificates and config
│   ├── live/
│   ├── archive/
│   └── renewal/
└── www/               # ACME challenge files

docker/config/
├── nginx.conf.dev     # Development HTTP config
└── nginx.conf.prod    # Production HTTPS config
```

## Security Notes

- Uses modern TLS 1.2/1.3 protocols
- Includes security headers (HSTS, CSP, etc.)
- 4096-bit RSA keys
- Automatic certificate renewal
- Proper www to non-www redirects

## Domain Configuration

Make sure your DNS has these A records:

- `yourdomain.com` → your server IP
- `www.yourdomain.com` → your server IP

## Ports

- Development: HTTP on port 8080
- Production: HTTP on port 80, HTTPS on port 443
