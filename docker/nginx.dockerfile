# Nginx reverse proxy for Tactile Teleop
FROM nginx:alpine

LABEL maintainer="Tactile Robotics <info@tactilerobotics.ai>"
LABEL description="Nginx reverse proxy for Tactile Teleop"

# Remove default nginx config
RUN rm /etc/nginx/conf.d/default.conf

# Copy generated nginx configuration
COPY src/tactile_teleop/web_server/nginx/nginx-tactile-teleop.conf /etc/nginx/conf.d/tactile-teleop.conf

# Create nginx user directories and SSL directories
RUN mkdir -p /var/cache/nginx /var/log/nginx /etc/ssl/certs /etc/ssl/private && \
    chown -R nginx:nginx /var/cache/nginx /var/log/nginx && \
    chmod 755 /etc/ssl/certs /etc/ssl/private

# Copy static files
COPY src/tactile_teleop/web_server/web-ui /usr/share/nginx/html

# Copy Docker-specific nginx validation script
COPY docker/scripts/nginx-docker-entrypoint.sh /docker-entrypoint.d/99-ssl-setup.sh
RUN chmod +x /docker-entrypoint.d/99-ssl-setup.sh

EXPOSE 80 443

CMD ["nginx", "-g", "daemon off;"]