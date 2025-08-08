# Nginx reverse proxy for Tactile Teleop
FROM nginx:alpine

LABEL maintainer="Tactile Robotics <info@tactilerobotics.ai>"
LABEL description="Nginx reverse proxy for Tactile Teleop"

# Remove default nginx config
RUN rm /etc/nginx/conf.d/default.conf

# Copy nginx configuration templates for runtime generation
RUN mkdir -p /etc/nginx/templates
COPY src/tactile_teleop/web_server/nginx/nginx.conf.dev.template /etc/nginx/templates/
COPY src/tactile_teleop/web_server/nginx/nginx.conf.prod.template /etc/nginx/templates/

# Create nginx user directories and SSL directories
RUN mkdir -p /var/cache/nginx /var/log/nginx /etc/ssl/certs /etc/ssl/private && \
    chown -R nginx:nginx /var/cache/nginx /var/log/nginx && \
    chmod 755 /etc/ssl/certs /etc/ssl/private

# Copy static files
COPY src/tactile_teleop/web_server/web-ui /usr/share/nginx/html

# Copy Docker-specific nginx validation script
COPY docker/scripts/nginx-docker-entrypoint.sh /docker-entrypoint.d/nginx-and-ssl-setup.sh
RUN chmod +x /docker-entrypoint.d/nginx-and-ssl-setup.sh

EXPOSE 80 443

CMD ["nginx", "-g", "daemon off;"]