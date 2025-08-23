// Simple constant-velocity Kalman filter in 3D working directly in scene units
// State vector: [x, y, z, vx, vy, vz]^T

export class KalmanCV3D {
  constructor({ processNoise = 1e-3, measurementNoise = 1e-2 } = {}) {
    this.x = null; // 6x1
    this.P = null; // 6x6
    this.q = processNoise;
    this.r = measurementNoise;
  }

  // Initialize from position and velocity estimates
  initialize({ position, velocity }) {
    this.x = mathVector([position[0], position[1], position[2], velocity[0], velocity[1], velocity[2]]);
    // Large initial uncertainty on velocity, moderate on position
    const pPos = 1;
    const pVel = 10;
    this.P = diag([pPos, pPos, pPos, pVel, pVel, pVel]);
  }

  // Build state transition matrix for dt
  F(dt) {
    return [
      [1, 0, 0, dt, 0, 0],
      [0, 1, 0, 0, dt, 0],
      [0, 0, 1, 0, 0, dt],
      [0, 0, 0, 1, 0, 0],
      [0, 0, 0, 0, 1, 0],
      [0, 0, 0, 0, 0, 1],
    ];
  }

  // Process noise (discrete white-noise accel model simplified)
  Q(dt) {
    const q = this.q;
    const dt2 = dt * dt;
    const dt3 = dt2 * dt;
    // Using simple diagonal model to keep it lightweight
    return diag([q * dt3, q * dt3, q * dt3, q * dt, q * dt, q * dt]);
  }

  H() {
    return [
      [1, 0, 0, 0, 0, 0],
      [0, 1, 0, 0, 0, 0],
      [0, 0, 1, 0, 0, 0],
    ];
  }

  R() {
    const r = this.r;
    return diag([r, r, r]);
  }

  predict(dt) {
    const F = this.F(dt);
    this.x = matVecMul(F, this.x);
    const Ft = transpose(F);
    this.P = add(add(matMul(F, this.P, Ft), this.Q(dt)), zerosLike(this.P));
    return this.x;
  }

  update(z) {
    const H = this.H();
    const Ht = transpose(H);
    const y = sub(z, matVecMul(H, this.x));
    const S = add(matMul(H, this.P, Ht), this.R());
    const K = matMul(this.P, Ht, inv3(S));
    this.x = add(this.x, matVecMul(K, y));
    const I = identity(6);
    this.P = matMul(sub(I, matMul(K, H)), this.P);
  }
}

// Minimal linear algebra helpers (small fixed-size matrices only)
function diag(values) {
  const n = values.length;
  const m = Array.from({ length: n }, () => Array(n).fill(0));
  for (let i = 0; i < n; i++) m[i][i] = values[i];
  return m;
}

function identity(n) {
  return diag(Array.from({ length: n }, () => 1));
}

function zerosLike(A) {
  return A.map(row => row.map(() => 0));
}

function matMul(A, B, C) {
  // If three args: return A * B * C
  const AB = _mul(A, B);
  return C ? _mul(AB, C) : AB;
}

function _mul(A, B) {
  const rows = A.length;
  const cols = B[0].length;
  const inner = B.length;
  const out = Array.from({ length: rows }, () => Array(cols).fill(0));
  for (let i = 0; i < rows; i++) {
    for (let k = 0; k < inner; k++) {
      const aik = A[i][k];
      for (let j = 0; j < cols; j++) out[i][j] += aik * B[k][j];
    }
  }
  return out;
}

function transpose(A) {
  const rows = A.length;
  const cols = A[0].length;
  const out = Array.from({ length: cols }, () => Array(rows).fill(0));
  for (let i = 0; i < rows; i++) for (let j = 0; j < cols; j++) out[j][i] = A[i][j];
  return out;
}

function mathVector(arr) {
  return arr.map(v => [v]);
}

function matVecMul(M, v) {
  const rows = M.length;
  const cols = M[0].length;
  const out = Array.from({ length: rows }, () => [0]);
  for (let i = 0; i < rows; i++) {
    let sum = 0;
    for (let j = 0; j < cols; j++) sum += M[i][j] * v[j][0];
    out[i][0] = sum;
  }
  return out;
}

function add(a, b) {
  if (Array.isArray(a[0])) {
    return a.map((row, i) => row.map((v, j) => v + b[i][j]));
  }
  return a.map((v, i) => [v[0] + b[i][0]]);
}

function sub(a, b) {
  if (Array.isArray(a[0])) {
    return a.map((row, i) => row.map((v, j) => v - b[i][j]));
  }
  return a.map((v, i) => [v[0] - b[i][0]]);
}

// Inverse for 3x3 matrix (S) using adjugate/determinant
function inv3(S) {
  const a = S[0][0], b = S[0][1], c = S[0][2];
  const d = S[1][0], e = S[1][1], f = S[1][2];
  const g = S[2][0], h = S[2][1], i = S[2][2];
  const A =  (e * i - f * h);
  const B = -(d * i - f * g);
  const C =  (d * h - e * g);
  const D = -(b * i - c * h);
  const E =  (a * i - c * g);
  const F = -(a * h - b * g);
  const G =  (b * f - c * e);
  const H = -(a * f - c * d);
  const I =  (a * e - b * d);
  const det = a * A + b * B + c * C;
  const invDet = 1 / det;
  return [
    [A * invDet, D * invDet, G * invDet],
    [B * invDet, E * invDet, H * invDet],
    [C * invDet, F * invDet, I * invDet],
  ];
}


