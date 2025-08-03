# Tactile Teleop Production Deployment Guide

Production deployment of the Tactile Teleop FastAPI web server with nginx reverse proxy and systemd service management.

## Architecture Overview

```
Internet → Nginx (Port 443/80) → Unix Socket → FastAPI Server
                ↓
            Static Files (Direct)
```

- **Nginx**: Handles HTTPS, static file serving, and reverse proxy
- **FastAPI**: API endpoints and health checks via Unix socket
- **Systemd**: Process management and auto-restart

## Prerequisites

- Ubuntu/Debian Linux system (tested on Ubuntu 20.04+)
- Conda/Miniconda installed
- Conda environment `tactile-teleop` configured with all dependencies
- Project cloned to desired location (e.g., `/home/username/telegrip`)
- Sudo privileges for installing nginx and systemd services

## Quick Deployment (Recommended)

### Ubuntu VM Deployment

For a fresh Ubuntu VM deployment:

```bash
# 1. Install system dependencies
sudo apt update && sudo apt install -y nginx git curl

# 2. Install Miniconda (if not already installed)
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b
source ~/miniconda3/etc/profile.d/conda.sh

# 3. Clone and setup the project
git clone <repository-url> telegrip
cd telegrip

# 4. Create conda environment (ensure it's named 'tactile-teleop')
# Set up your tactile-teleop environment with all required dependencies

# 5. Run automated deployment
./deploy-setup.sh your-server-ip-or-domain

# 6. Follow the printed instructions to complete deployment
```

### Automated Setup

Use the provided deployment script to automatically configure files for your system:

```bash
# Clone the repository
git clone <repository-url> telegrip
cd telegrip

# Create conda environment (if not already done)
# Ensure you have the 'tactile-teleop' conda environment setup

# Run automated deployment setup
./deploy-setup.sh

# Or specify a custom domain/IP
./deploy-setup.sh your-domain.com
```

The script will:

- Auto-detect your user, paths, and conda environment (must be named `tactile-teleop`)
- Generate properly configured nginx and systemd files in `deployment/` from templates
- Replace all placeholders with your system-specific values
- Provide step-by-step deployment instructions

### Manual Setup (Advanced)

If you prefer manual configuration:

#### 1. Deploy Nginx Configuration

```bash
# Generate configuration from template
./deploy-setup.sh your-server-name

# Install nginx config
sudo cp deployment/nginx-tactile-teleop.conf /etc/nginx/sites-available/tactile-teleop
sudo ln -sf /etc/nginx/sites-available/tactile-teleop /etc/nginx/sites-enabled/

# Validate and reload
sudo nginx -t
sudo systemctl reload nginx
```

#### 2. Install Systemd Service

```bash
# Deploy service
sudo cp deployment/tactile-teleop.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tactile-teleop
```

#### 3. SSL Certificate Setup

```bash
# Create SSL directories
sudo mkdir -p /etc/ssl/certs /etc/ssl/private

# Generate certificates (replace with your project path)
cd /path/to/your/telegrip
source ~/miniconda3/etc/profile.d/conda.sh && conda activate tactile-teleop
python -c "from tactile_teleop.config import global_config; global_config.ensure_ssl_certificates()"

# Deploy certificates
sudo cp ~/.tactile_teleop/ssl/cert.pem /etc/ssl/certs/tactile-teleop.crt
sudo cp ~/.tactile_teleop/ssl/key.pem /etc/ssl/private/tactile-teleop.key
sudo chmod 644 /etc/ssl/certs/tactile-teleop.crt
sudo chmod 600 /etc/ssl/private/tactile-teleop.key
```

#### 4. Start Services

```bash
# Start FastAPI service
sudo systemctl start tactile-teleop

# Restart nginx to apply all changes
sudo systemctl restart nginx
```

## Verification

### Service Status

```bash
# Check FastAPI service
sudo systemctl status tactile-teleop

# Check nginx
sudo systemctl status nginx

# Verify Unix socket
ls -la /tmp/tactile-teleop.sock
```

### Health Checks

```bash
# Test HTTPS endpoint (replace localhost with your server IP/domain)
curl -k https://localhost/health

# Expected response: {"status": "healthy"}

# Check nginx access
curl -k https://localhost/

# Should return the web UI HTML
```

### Log Monitoring

```bash
# FastAPI logs
sudo journalctl -u tactile-teleop -f

# Nginx logs
sudo tail -f /var/log/nginx/tactile-teleop.access.log
sudo tail -f /var/log/nginx/tactile-teleop.error.log
```

## Configuration Files

### Nginx Config (`/etc/nginx/sites-available/tactile-teleop`)

- HTTPS termination with HTTP→HTTPS redirect
- Static file serving from `web-ui/` directory
- API proxy to Unix socket `/tmp/tactile-teleop.sock`
- Security headers and caching policies

### Systemd Service (`/etc/systemd/system/tactile-teleop.service`)

- Conda environment activation
- Unix socket mode (`--behind-proxy`)
- Auto-restart on failure
- Process isolation and security hardening

## Validation and Testing

### Post-Deployment Validation

After completing deployment, validate that everything is working:

```bash
# 1. Check all services are running
sudo systemctl status tactile-teleop
sudo systemctl status nginx

# 2. Verify Unix socket exists
ls -la /tmp/tactile-teleop.sock

# 3. Test health endpoint
curl -k https://your-server-domain/health

# 4. Check web interface
curl -k https://your-server-domain/

# 5. Monitor logs in real-time (in separate terminals)
sudo journalctl -u tactile-teleop -f
sudo tail -f /var/log/nginx/tactile-teleop.access.log
```

### Common Deployment Gotchas

**Template vs Generated Files:**

- Never edit `deployment/*.conf` or `deployment/*.service` directly
- Always edit the `.template` files and re-run `./deploy-setup.sh`
- Generated files are overwritten each time you run the script

**Conda Environment:**

- Environment MUST be named exactly `tactile-teleop`
- Verify with: `conda env list | grep tactile-teleop`
- Ensure all project dependencies are installed in this environment

**File Permissions:**

- SSL certificates need correct permissions (cert: 644, key: 600)
- Unix socket directory `/tmp/` should be writable by the service user
- Nginx must have read access to the project directory

## Troubleshooting

### Common Issues

**Service won't start:**

```bash
# Check logs
sudo journalctl -u tactile-teleop --no-pager

# Verify conda environment (replace with actual username and path)
sudo -u $USER bash -c "cd /path/to/your/telegrip && source ~/miniconda3/etc/profile.d/conda.sh && conda activate tactile-teleop && python -m tactile_teleop.web_server --help"
```

**Nginx 502 Bad Gateway:**

```bash
# Check Unix socket exists
ls -la /tmp/tactile-teleop.sock

# Verify socket permissions
sudo systemctl restart tactile-teleop
```

**SSL Certificate Issues:**

```bash
# Regenerate certificates (replace with actual username and path)
sudo -u $USER bash -c "cd /path/to/your/telegrip && source ~/miniconda3/etc/profile.d/conda.sh && conda activate tactile-teleop && python -c 'from tactile_teleop.config import global_config; global_config.ensure_ssl_certificates()'"

# Redeploy to nginx locations
sudo cp ~/.tactile_teleop/ssl/cert.pem /etc/ssl/certs/tactile-teleop.crt
sudo cp ~/.tactile_teleop/ssl/key.pem /etc/ssl/private/tactile-teleop.key
sudo systemctl reload nginx
```

### Log Locations

- **FastAPI**: `sudo journalctl -u tactile-teleop`
- **Nginx Access**: `/var/log/nginx/tactile-teleop.access.log`
- **Nginx Error**: `/var/log/nginx/tactile-teleop.error.log`

## Security Considerations

- SSL/TLS encryption enforced
- Security headers configured (HSTS, XSS protection, etc.)
- Service runs as non-root user
- Process isolation via systemd
- Private temp directory for Unix socket

## Maintenance

### Certificate Renewal

Certificates are self-signed and valid for 365 days. To renew:

```bash
# Regenerate certificates (replace with actual username and path)
sudo -u $USER bash -c "cd /path/to/your/telegrip && source ~/miniconda3/etc/profile.d/conda.sh && conda activate tactile-teleop && python -c 'from tactile_teleop.config import global_config; global_config.ensure_ssl_certificates()'"

# Redeploy certificates
sudo cp ~/.tactile_teleop/ssl/cert.pem /etc/ssl/certs/tactile-teleop.crt
sudo cp ~/.tactile_teleop/ssl/key.pem /etc/ssl/private/tactile-teleop.key
sudo systemctl reload nginx
```

### Service Updates

After code changes:

```bash
sudo systemctl restart tactile-teleop
```

### Configuration Changes

After modifying nginx config:

```bash
sudo nginx -t
sudo systemctl reload nginx
```

## Performance Notes

- Static files served directly by nginx (optimal performance)
- Unix socket communication (lower latency than TCP)
- HTTP/2 enabled for modern browsers
- Asset caching configured for static resources

## Portable Deployment

This deployment system uses **template files** with placeholders that are automatically replaced with your system-specific values:

### Template Files:

- `deployment/nginx-tactile-teleop.conf.template` - Template with `{{PROJECT_ROOT}}` and `{{SERVER_NAME}}` placeholders
- `deployment/tactile-teleop.service.template` - Template with `{{USER}}`, `{{PROJECT_ROOT}}`, and `{{CONDA_ENV_PATH}}` placeholders

### Generated Files:

When you run `./deploy-setup.sh`, it creates:

- `deployment/nginx-tactile-teleop.conf` - Ready for deployment to `/etc/nginx/sites-available/`
- `deployment/tactile-teleop.service` - Ready for deployment to `/etc/systemd/system/`

### Important Notes:

- **Always use the `deploy-setup.sh` script** to generate properly configured files
- **Never edit the generated files directly** - they will be overwritten when you re-run the script
- **Template files are preserved** - you can run the script multiple times safely
- **The conda environment must be named `tactile-teleop`**

The script automatically:

1. Auto-detects your environment (user, paths, conda installation)
2. Validates that the `tactile-teleop` conda environment exists
3. Replaces all placeholders with detected values
4. Creates deployment-ready configuration files
5. Provides complete deployment instructions

---

**Access URL**: `https://localhost/` (or your server's IP/domain)
