/*
  TleWebSocket: Lightweight WebSocket client for receiving live TLE updates.

  Expected message format (JSON per message):
  {
    "type": "tle_update",
    "id": "Drone",           // satellite id/key to store under
    "name": "Drone",         // human-readable name
    "tle1": "1 xxxxx...",    // TLE line 1
    "tle2": "2 xxxxx...",    // TLE line 2
    "timestamp": 1731000000000 // optional
  }

  Reconnection: tries to reconnect with exponential backoff.
*/

export default class TleWebSocket {
  constructor({ url, onTle, onStatusChange }) {
    this.url = url;
    this.onTle = onTle;
    this.onStatusChange = onStatusChange;
    this.socket = null;
    this.shouldRun = false;
    this.retryAttempt = 0;
    this.maxBackoffMs = 15000;
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

  setUrl(url) {
    this.url = url;
    if (this.socket) {
      this.stop();
      this.start();
    }
  }

  connect() {
    if (!this.shouldRun || !this.url) return;
    try {
      this.socket = new WebSocket(this.url);

      this.socket.onopen = () => {
        this.retryAttempt = 0;
        if (this.onStatusChange) this.onStatusChange('connected');
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
          if (
            data &&
            data.type === 'tle_update' &&
            typeof data.tle1 === 'string' &&
            typeof data.tle2 === 'string'
          ) {
            if (this.onTle) this.onTle(data);
          }
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
}


