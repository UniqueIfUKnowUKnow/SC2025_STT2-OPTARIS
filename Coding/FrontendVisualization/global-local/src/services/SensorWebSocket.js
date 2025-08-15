/*
  SensorWebSocket: Lightweight WebSocket client for receiving live LiDAR readings.
*/

export default class SensorWebSocket {
  constructor({ url, onMessage, onStatusChange }) {
    this.url = url;
    this.onMessage = onMessage;
    this.onStatusChange = onStatusChange;
    this.socket = null;
    this.shouldRun = false;
    this.retryAttempt = 0;
    this.maxBackoffMs = 15000;
    this.outbox = [];
  }

  start() {
    this.shouldRun = true;
    this.connect();
  }

  stop() {
    this.shouldRun = false;
    if (this.socket) {
      try { this.socket.close(); } catch (_) {}
    }
    this.socket = null;
  }

  connect() {
    if (!this.shouldRun || !this.url) return;
    try {
      this.socket = new WebSocket(this.url);

      this.socket.onopen = () => {
        this.retryAttempt = 0;
        if (this.onStatusChange) this.onStatusChange('connected');
        // Flush any queued messages
        try {
          while (this.outbox.length > 0 && this.socket && this.socket.readyState === WebSocket.OPEN) {
            const next = this.outbox.shift();
            this.socket.send(JSON.stringify(next));
          }
        } catch (_) {}
      };

      this.socket.onclose = () => {
        if (this.onStatusChange) this.onStatusChange('disconnected');
        this.socket = null;
        if (this.shouldRun) this.scheduleReconnect();
      };

      this.socket.onerror = () => {
        if (this.onStatusChange) this.onStatusChange('error');
      };

      this.socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (this.onMessage) this.onMessage(data);
        } catch (_) {
          // ignore malformed
        }
      };
    } catch (_) {
      this.scheduleReconnect();
    }
  }

  scheduleReconnect() {
    this.retryAttempt += 1;
    const backoff = Math.min(this.maxBackoffMs, 500 * 2 ** (this.retryAttempt - 1));
    setTimeout(() => this.connect(), backoff);
  }

  send(data) {
    try {
      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        this.socket.send(JSON.stringify(data));
      } else {
        // queue until open
        this.outbox.push(data);
      }
    } catch (_) {
      // As a fallback, queue the message to attempt later
      this.outbox.push(data);
    }
  }
}
