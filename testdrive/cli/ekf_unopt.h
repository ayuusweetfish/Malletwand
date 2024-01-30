#pragma once

#include <math.h>

void ekf_step_unopt(float x[5], float P[5][5], const float z[2]) {
  float ω = x[0];
  float A = x[1];
  float θ = x[2];
  float B = x[3];
  float ϕ = x[4];
  float dt = 0.01;

  A = fabs(A);
  A = fabs(A);
  B = fabs(B);
  if (θ > M_PI * 2) θ -= M_PI * 2; else if (θ < 0) θ += M_PI * 2;
  if (ϕ > M_PI * 2) ϕ -= M_PI * 2; else if (ϕ < 0) ϕ += M_PI * 2;

  float Q[5][5] = {
    {1e-2, 0, 0, 0, 0},
    {0, 1e-4, 0, 0, 0},
    {0, 0, 1e-2, 0, 0},
    {0, 0, 0, 1e-4, 0},
    {0, 0, 0, 0, 1e-2},
  };
  float R[2][2] = {
    {0.1, 0},
    {0, 0.08},
  };

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
  float Ft[5][5];
  mat_transpose(5, 5, (float *)Ft, (float *)F);
  float P1[5][5], tmpA[5][5], tmpB[5][5];
  mat_mul(5, 5, 5, (float *)tmpA, (float *)P, (float *)Ft);
  mat_mul(5, 5, 5, (float *)tmpB, (float *)F, (float *)tmpA);
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++)
      P1[i][j] = tmpB[i][j] + Q[i][j];
/*
  printf("P = \n"); mat_print(5, 5, P);
  printf("F = \n"); mat_print(5, 5, F);
  printf("P1 = \n"); mat_print(5, 5, P1);
*/

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
  float y[2] = {
    z[0] - h[0],
    z[1] - h[1],
  };

  float S[2][2];
  float Ht[5][2];
  mat_transpose(2, 5, (float *)Ht, (float *)H);
  mat_mul(5, 5, 2, (float *)tmpA, (float *)P1, (float *)Ht);
  mat_mul(2, 5, 2, (float *)S, (float *)H, (float *)tmpA);
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      S[i][j] += R[i][j];

  // float invS[2][2];
  // mat_inv(2, (float *)invS, (float *)S);
  float detS = S[0][0] * S[1][1] - S[0][1] * S[1][0];
  float invS[2][2] = {
    { S[1][1] / detS, -S[0][1] / detS},
    {-S[1][0] / detS,  S[0][0] / detS},
  };
/*
  printf("Ht = \n"); mat_print(5, 2, Ht);
  printf("S = \n"); mat_print(2, 2, S);
*/
  float K[5][2];
  mat_mul(5, 2, 2, (float *)tmpA, (float *)Ht, (float *)invS);
  mat_mul(5, 5, 2, (float *)K, (float *)P1, (float *)tmpA);

  float x2[5];
  float Ky[5];
  mat_mul(5, 2, 1, (float *)Ky, (float *)K, (float *)y);
  for (int i = 0; i < 5; i++) x2[i] = x1[i] + Ky[i];
/*
  printf("K = \n"); mat_print(5, 2, K);
  printf("y = \n"); mat_print(2, 1, y);
  printf("Ky = \n"); mat_print(5, 1, Ky);
  printf("x2 = \n"); mat_print(1, 5, x2);
*/

  float P2[5][5];
/*
  float KHP1[5][5];
  mat_mul(2, 5, 5, (float *)tmpA, (float *)H, (float *)P1);
  printf("H = \n"); mat_print(2, 5, H);
  printf("P1 = \n"); mat_print(5, 5, P1);
  mat_mul(5, 2, 5, (float *)KHP1, (float *)K, (float *)tmpA);
  printf("KHP1 = \n"); mat_print(5, 5, KHP1);
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++)
      P2[i][j] = P1[i][j] - KHP1[i][j];
*/
  float KH[5][5];
  mat_mul(5, 2, 5, (float *)KH, (float *)K, (float *)H);
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++)
      KH[i][j] = (i == j ? 1 : 0) - KH[i][j];
  // printf("I-KH = \n"); mat_print(5, 5, KH);
  mat_mul(5, 5, 5, (float *)P2, (float *)KH, (float *)P1);
  // printf("P2 = \n"); mat_print(5, 5, P2);

  for (int i = 0; i < 5; i++) x[i] = x2[i];
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++) P[i][j] = P2[i][j];
}
