#pragma once

#ifdef __cplusplus
#define restrict __restrict
#endif

#include <math.h>

#define idx(_nrows, _ncols, _row, _col) ((_row)*(_ncols) + (_col))

static inline void mat_print(
  int m, int n,
  const float *a
) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++)
      printf(" %7.3f", a[idx(m, n, i, j)]);
    putchar('\n');
  }
}

static inline void mat_transpose(
  int m, int n,
  float *restrict t,        // n * m
  const float *restrict a   // m * n
) {
  for (int j = 0; j < n; j++)
    for (int i = 0; i < m; i++)
      t[idx(n, m, j, i)] = a[idx(m, n, i, j)];
}

static inline void mat_mul(
  int m, int n, int p,
  float *restrict c,        // m * p
  const float *restrict a,  // m * n
  const float *restrict b   // n * p
) {
  for (int i = 0; i < m; i++)
    for (int k = 0; k < p; k++) {
      c[idx(m, p, i, k)] = 0;
      for (int j = 0; j < n; j++)
        c[idx(m, p, i, k)] += a[idx(m, n, i, j)] * b[idx(n, p, j, k)];
    }
}

void ekf_step(float x[5], float P[5][5], const float z[2]) {
  float ω = x[0];
  float A = x[1];
  float θ = x[2];
  float B = x[3];
  float ϕ = x[4];
  float dt = 0.01;

  ω = fabs(ω - 1.0f) + 1.0f;
  A = fabs(A);
  B = fabs(B);
  if (θ > M_PI * 2) θ -= M_PI * 2; else if (θ < 0) θ += M_PI * 2;
  if (ϕ > M_PI * 2) ϕ -= M_PI * 2; else if (ϕ < 0) ϕ += M_PI * 2;

  float x1[5] = {
    ω,
    A,
    θ + ω*dt,
    B,
    ϕ,
  };
  float F[5][5] = {
    {1, 0, 0, 0, 0},
    {0, 1, 0, 0, 0},
    {dt, 0, 1, 0, 0},
    {0, 0, 0, 1, 0},
    {0, 0, 0, 0, 1},
  };
  float P1[5][5];
  // P1 = F P F^T
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++) P1[i][j] = P[i][j];
  for (int i = 0; i < 5; i++) P1[2][i] += P[0][i] * dt;
  for (int i = 0; i < 5; i++) P1[i][2] += P[i][0] * dt;
  P1[2][2] += P[0][0] * (dt * dt);
  // P1 += Q
/*
  float Q[5][5] = {
    {1e-2, 0, 0, 0, 0},
    {0, 1e-4, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 1e-4, 0},
    {0, 0, 0, 0, 3e-4},
  };
*/
  P1[0][0] += 1e-2f;
  P1[1][1] += 1e-4f;
  // P1[2][2] += 0;
  P1[3][3] += 1e-4f;
  P1[4][4] += 3e-4f;

  float A_1 = x1[1];
  float θ_1 = x1[2];
  float B_1 = x1[3];
  float ϕ_1 = x1[4];
  float h[2] = {
    A_1 * cosf(θ_1),
    B_1 * cosf(θ_1 + ϕ_1),
  };
  float H[2][5] = {
    {0, cosf(θ_1), A_1 * -sinf(θ_1), 0, 0},
    {0, 0, B_1 * -sinf(θ_1 + ϕ_1), cosf(θ_1 + ϕ_1), B_1 * -sinf(θ_1 + ϕ_1)},
  };
  float H01 = H[0][1];
  float H02 = H[0][2];
  float H12 = H[1][2];
  float H13 = H[1][3];
  float y[2] = {
    z[0] - h[0],
    z[1] - h[1],
  };

  float S[2][2];
  float Ht[5][2];
  float tmpA[5][5];
  mat_transpose(2, 5, (float *)Ht, (float *)H);
  // S = H P1 H^T
  S[0][0] = H01 * (H01 * P1[1][1] + H02 * P1[2][1]) +
            H02 * (H01 * P1[1][2] + H02 * P1[2][2]);
  S[0][1] = H12 * (H01 * (P1[1][2] + P1[1][4]) + H02 * (P1[2][2] + P1[2][4])) +
            H13 * (H01 * P1[1][3] + H02 * P1[2][3]);
  S[1][0] = H01 * (H12 * (P1[2][1] + P1[4][1]) + H13 * P1[3][1]) +
            H02 * (H12 * (P1[2][2] + P1[4][2]) + H13 * P1[3][2]);
  S[1][1] = H12 * (H12 * (P1[2][2] + P1[4][2] + P1[2][4] + P1[4][4]) +
                   H13 * (P1[3][2] + P1[3][4] + P1[2][3] + P1[4][3])) +
            H13 * H13 * P1[3][3];
  // S += R
/*
  float R[2][2] = {
    {0.1, 0},
    {0, 0.08},
  };
*/
  S[0][0] += 0.1f;
  S[1][1] += 0.08f;

  float detS = S[0][0] * S[1][1] - S[0][1] * S[1][0];
  float invS[2][2] = {
    { S[1][1] / detS, -S[0][1] / detS},
    {-S[1][0] / detS,  S[0][0] / detS},
  };
  float K[5][2];
  mat_mul(5, 2, 2, (float *)tmpA, (float *)Ht, (float *)invS);
  mat_mul(5, 5, 2, (float *)K, (float *)P1, (float *)tmpA);

  float x2[5];
  float Ky[5];
  mat_mul(5, 2, 1, (float *)Ky, (float *)K, (float *)y);
  for (int i = 0; i < 5; i++) x2[i] = x1[i] + Ky[i];

  float P2[5][5];
  float KH[5][5];
  mat_mul(5, 2, 5, (float *)KH, (float *)K, (float *)H);
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++)
      KH[i][j] = (i == j ? 1 : 0) - KH[i][j];
  mat_mul(5, 5, 5, (float *)P2, (float *)KH, (float *)P1);

  for (int i = 0; i < 5; i++) x[i] = x2[i];
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++) P[i][j] = P2[i][j];
}
