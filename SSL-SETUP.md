# SSL Certificate Setup for Tactile Teleop

The Tactile Teleop application currently uses self-signed SSL certificates, which cause browser security warnings. This guide explains how to set up trusted SSL certificates.

## Current Issue

The application generates self-signed certificates during Docker build, which are not trusted by browsers. You'll see warnings like:

- "Not secure" in browser address bar
- Certificate errors about untrusted issuer
- SSL/TLS certificate warnings

## Solution Options

### Option 1: Let's Encrypt (Recommended - Free & Automated)

Let's Encrypt provides free, trusted SSL certificates that are automatically renewed.

#### Prerequisites

- Domain name pointing to your server's public IP (`teleop.tactilerobotics.ai`)
- Ports 80 and 443 accessible from the internet
- Email address for certificate notifications

#### Setup Steps

1. **Stop current containers:**

   ```bash
   docker-compose down
   ```

2. **Run the Let's Encrypt setup script:**

   ```bash
   ./docker/scripts/setup-letsencrypt.sh teleop.tactilerobotics.ai zeno@tactilerobotics.ai
   ```

3. **Enable SSL certificates in docker-compose:**

   ```bash
   # Edit docker-compose.ssl.yml and uncomment the Let's Encrypt certificate lines
   # Then start with SSL configuration:
   docker-compose -f docker-compose.yml -f docker-compose.ssl.yml up -d
   ```

4. **Set up automatic renewal:**
   ```bash
   sudo crontab -e
   # Add this line:
   0 12 * * * /usr/bin/certbot renew --quiet && docker-compose restart nginx
   ```

### Option 2: Custom CA Certificate

If you have certificates from a commercial CA (like DigiCert, GlobalSign, etc.):

1. **Place your certificates in the ssl/ directory:**

   ```bash
   mkdir -p ssl/
   cp your-certificate.crt ssl/custom-cert.crt
   cp your-private-key.key ssl/custom-key.key
   ```

2. **Update docker-compose.ssl.yml:**

   ```yaml
   # Uncomment and use the custom CA certificate lines
   - ./ssl/custom-cert.crt:/etc/ssl/certs/tactile-teleop.crt:ro
   - ./ssl/custom-key.key:/etc/ssl/private/tactile-teleop.key:ro
   ```

3. **Start with SSL configuration:**
   ```bash
   docker-compose -f docker-compose.yml -f docker-compose.ssl.yml up -d
   ```

### Option 3: Development/Testing Only

For development or internal testing, you can continue using self-signed certificates but add them to your browser's trusted certificates:

1. **Export the current certificate:**

   ```bash
   docker exec tactile-teleop-nginx openssl x509 -in /etc/ssl/certs/tactile-teleop.crt -out tactile-teleop.crt
   ```

2. **Add to your system's trusted certificates** (varies by OS)

## Verification

After setting up trusted certificates:

1. **Check certificate in browser:**

   - Visit `https://teleop.tactilerobotics.ai`
   - Click the lock icon in address bar
   - Verify certificate is issued by trusted CA

2. **Test certificate with openssl:**

   ```bash
   openssl s_client -connect teleop.tactilerobotics.ai:443 -servername teleop.tactilerobotics.ai
   ```

3. **Check certificate expiration:**
   ```bash
   openssl x509 -in ssl/cert.pem -text -noout | grep "Not After"
   ```

## Troubleshooting

### DNS Issues

- Ensure `teleop.tactilerobotics.ai` resolves to your server's public IP
- Check with: `nslookup teleop.tactilerobotics.ai`

### Port Access Issues

- Ensure ports 80 and 443 are open in firewall
- Check with: `sudo ufw status` or `iptables -L`

### Certificate Renewal Issues

- Check certbot logs: `sudo tail -f /var/log/letsencrypt/letsencrypt.log`
- Test renewal: `sudo certbot renew --dry-run`

## Security Best Practices

1. **Keep certificates secure:**

   - Set proper file permissions (600 for private keys)
   - Store certificates outside web root
   - Use strong private key encryption

2. **Monitor certificate expiration:**

   - Set up monitoring alerts
   - Test renewal process regularly

3. **Use HSTS headers:**

   - Already configured in nginx.conf
   - Forces HTTPS connections

4. **Regular updates:**
   - Keep certbot updated
   - Monitor certificate transparency logs
