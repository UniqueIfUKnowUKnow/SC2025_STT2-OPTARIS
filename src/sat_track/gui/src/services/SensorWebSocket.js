/*
  SensorWebSocket: Lightweight WebSocket client for receiving live LiDAR readings.

  Expected message format (JSON per message):
  {
    "timestamp": 1731000000000, // ms since epoch (optional)
    "range_m": 123.45,          // required: distance in meters
    "az_deg": 12.3,             // required: azimuth degrees (0=N, +clockwise)
    "el_deg": 15.4              // required: elevation degrees (0=horizon, +up)
  }

  Reconnection: tries to reconnect with exponential backoff.
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
            typeof data.range_m === 'number' &&
            typeof data.az_deg === 'number' &&
            typeof data.el_deg === 'number'
          ) {
            if (this.onMessage) this.onMessage(data);
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


