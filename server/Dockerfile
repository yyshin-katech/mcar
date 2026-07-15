FROM node:18-alpine

WORKDIR /app

COPY server/package.json server/package-lock.json ./server/
RUN cd server && npm ci --omit=dev --omit=optional && npm cache clean --force

COPY server/index.js ./server/index.js
COPY server/src ./server/src
COPY frontend ./frontend

ENV NODE_ENV=production
ENV PORT=8080
ENV STALE_THRESHOLD_MS=10000

EXPOSE 8080

USER node
WORKDIR /app/server

CMD ["node", "index.js"]
