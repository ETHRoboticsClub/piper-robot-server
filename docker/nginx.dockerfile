# Nginx reverse proxy for Tactile Teleop
FROM nginx:alpine

LABEL maintainer="Tactile Robotics <info@tactilerobotics.ai>"
LABEL description="Nginx reverse proxy for Tactile Teleop"

# Install openssl for certificate generation
RUN apk add --no-cache openssl

# Remove default nginx config
RUN rm /etc/nginx/conf.d/default.conf

# Copy nginx configuration
COPY docker/config/nginx.conf /etc/nginx/conf.d/tactile-teleop.conf

# Create directories for SSL certificates
RUN mkdir -p /etc/ssl/certs /etc/ssl/private

# Create nginx user directories
RUN mkdir -p /var/cache/nginx /var/log/nginx && \
    chown -R nginx:nginx /var/cache/nginx /var/log/nginx

# Copy static files
COPY src/tactile_teleop/web_server/web-ui /usr/share/nginx/html

# Generate self-signed SSL certificate with dynamic domain name
ARG DOMAIN_NAME=localhost
RUN openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -keyout /etc/ssl/private/tactile-teleop.key \
    -out /etc/ssl/certs/tactile-teleop.crt \
    -subj "/C=US/ST=State/L=City/O=TactileRobotics/CN=${DOMAIN_NAME}"

# Set proper permissions
RUN chmod 600 /etc/ssl/private/tactile-teleop.key && \
    chmod 644 /etc/ssl/certs/tactile-teleop.crt

EXPOSE 80 443

CMD ["nginx", "-g", "daemon off;"]