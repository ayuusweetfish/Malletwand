#pragma once

#include "geom.h"
#include <stdio.h>

void eig_sym(const float *const A, float *ev, float *d, uint16_t row);
void inv_mat3_sym(const float A[3][3], float inv[3][3])
{
  float det =
    + A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
    - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
    + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  float adj[3][3] = {{
    +(A[1][1] * A[2][2] - A[2][1] * A[1][2]),
    -(A[0][1] * A[2][2] - A[2][1] * A[0][2]),
    +(A[0][1] * A[1][2] - A[1][1] * A[0][2]),
  }, {
    0, // -(A[1][0] * A[2][2] - A[2][0] * A[1][2]),
    +(A[0][0] * A[2][2] - A[2][0] * A[0][2]),
    -(A[0][0] * A[1][2] - A[1][0] * A[0][2]),
  }, {
    0, // +(A[1][0] * A[2][1] - A[2][0] * A[1][1]),
    0, // -(A[0][0] * A[2][1] - A[2][0] * A[0][1]),
    +(A[0][0] * A[1][1] - A[1][0] * A[0][1]),
  }};
  for (int i = 0; i < 3; i++)
    for (int j = i; j < 3; j++)
      inv[j][i] = inv[i][j] = (adj[i][j] / det);
}

void elli_fit(int m, const vec3 *x, double σ, float inv_tfm[3][3], float c[3])
{
  static double t[3000][3][5];
  for (int i = 0; i < m; i++) {
    for (int d = 0; d < 3; d++) {
      double ξ = vec3_component(x[i], d);
      t[i][d][0] = 1;
      t[i][d][1] = ξ;
      t[i][d][2] = ξ*ξ - σ*σ;
      t[i][d][3] = ξ * (ξ*ξ - 3*σ*σ);
      t[i][d][4] = ξ*ξ * (ξ*ξ - 6*σ*σ) + 3*σ*σ*σ*σ;
    }
  }
  /* for (int i = 0; i < m; i++)
    for (int d = 0; d < 3; d++)
      for (int k = 0; k < 5; k++)
        printf("%d %d %d %.5f\n", i, d, k, t[i][d][k]); */

  static const uint8_t R[10][10][3] = {
    {{4,0,0}, {3,1,0}, {2,2,0}, {3,0,1}, {2,1,1}, {2,0,2}, {3,0,0}, {2,1,0}, {2,0,1}, {2,0,0}},
    {{0,0,0}, {2,2,0}, {1,3,0}, {2,1,1}, {1,2,1}, {1,1,2}, {2,1,0}, {1,2,0}, {1,1,1}, {1,1,0}},
    {{0,0,0}, {0,0,0}, {0,4,0}, {1,2,1}, {0,3,1}, {0,2,2}, {1,2,0}, {0,3,0}, {0,2,1}, {0,2,0}},
    {{0,0,0}, {0,0,0}, {0,0,0}, {2,0,2}, {1,1,2}, {1,0,3}, {2,0,1}, {1,1,1}, {1,0,2}, {1,0,1}},
    {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,2,2}, {0,1,3}, {1,1,1}, {0,2,1}, {0,1,2}, {0,1,1}},
    {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,4}, {1,0,2}, {0,1,2}, {0,0,3}, {0,0,2}},
    {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {2,0,0}, {1,1,0}, {1,0,1}, {1,0,0}},
    {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,2,0}, {0,1,1}, {0,1,0}},
    {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,2}, {0,0,1}},
    {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}},
  };
  float psi[10][10];
  for (int p = 0; p < 10; p++)
    for (int q = p; q < 10; q++) {
      psi[p][q] = 0;
      for (int i = 0; i < m; i++)
        psi[p][q] +=
          t[i][0][R[p][q][0]] *
          t[i][1][R[p][q][1]] *
          t[i][2][R[p][q][2]];
      if (p == 1 || p == 3 || p == 4) psi[p][q] *= 2;
      if (q == 1 || q == 3 || q == 4) psi[p][q] *= 2;
    }
  for (int p = 0; p < 10; p++)
    for (int q = 0; q < p; q++)
      psi[p][q] = psi[q][p];
  /* for (int i = 0; i < 10; i++)
    for (int j = 0; j < 10; j++)
      printf("%9.5f%c", psi[i][j], j == 9 ? '\n' : ' '); */

  float eigenvectors[10][10];
  float lambda[10];
  eig_sym((const float *)psi, (float *)eigenvectors, lambda, 10);
  /* for (int i = 0; i < 10; i++) {
    printf("Eigenvalue = %.8f\n", lambda[i]);
    for (int j = 0; j < 10; j++)
      printf("%9.5f%c", eigenvectors[j][i], j == 9 ? '\n' : ' ');
  } */
  int neg_most = 0;
  for (int i = 1; i < 10; i++)
    if (lambda[i] < lambda[neg_most]) neg_most = i;
  float ev_norm = 0;
  for (int j = 0; j < 10; j++)
    ev_norm += eigenvectors[j][neg_most] * eigenvectors[j][neg_most];
  ev_norm = sqrtf(ev_norm);
  /* for (int j = 0; j < 10; j++)
    printf("%9.5f%c", eigenvectors[j][neg_most] / ev_norm, j == 9 ? '\n' : ' '); */

  float a_est[3][3], b_est[3], d_est;
  for (int j = 0, t = 0; j < 3; j++)
    for (int i = 0; i <= j; i++, t++) {
      a_est[i][j] = a_est[j][i] = eigenvectors[t][neg_most] / ev_norm;
    }
  for (int i = 0; i < 3; i++)
    b_est[i] = eigenvectors[i + 6][neg_most] / ev_norm;
  d_est = eigenvectors[9][neg_most] / ev_norm;

  // Inverse of A_est
  float a_inv[3][3];
  inv_mat3_sym(a_est, a_inv);
  /* for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      printf("%9.5f%c", a_inv[i][j], j == 2 ? '\n' : ' '); */

  float c_est[3] = { 0 };
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      c_est[i] += a_inv[i][j] * b_est[j];
    c_est[i] *= -0.5;
  }
  // printf("%.8f %.8f %.8f\n", c_est[0], c_est[1], c_est[2]);

  // c_est' * A_est * c_est - d_est
  float a_scale = -d_est;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      a_scale += c_est[i] * c_est[j] * a_est[i][j];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      a_est[i][j] /= a_scale;
  /* for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      printf("%9.5f%c", a_est[i][j], j == 2 ? '\n' : ' '); */

  // Matrix square root of A_est
  // VDV' = eigen(inv(scaled A_est)) = eigen(inv(previous A_est) * a_scale)
  float d_eigenvectors[3][3], d_lambda[3];
  eig_sym((const float *)a_inv, (float *)d_eigenvectors, d_lambda, 3);
  /* for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      printf("%9.5f%c", a_inv[i][j], j == 2 ? '\n' : ' ');
  printf("scale = %.8f\n", a_scale);
  for (int i = 0; i < 3; i++) {
    printf("Eigenvalue = %.8f\n", d_lambda[i] * a_scale);
    for (int j = 0; j < 3; j++)
      printf("%9.5f%c", d_eigenvectors[j][i], j == 2 ? '\n' : ' ');
  } */

  // Rebuild transform matrix
  for (int i = 0; i < 3; i++) d_lambda[i] = sqrtf(d_lambda[i] * a_scale);
  float a_tfm[3][3] = {{ 0 }};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        a_tfm[i][j] += d_eigenvectors[i][k] * d_eigenvectors[j][k] * d_lambda[k];
  }
  /* for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      printf("%9.5f%c", a_tfm[i][j], j == 2 ? '\n' : ' '); */

  inv_mat3_sym(a_tfm, inv_tfm);
  for (int i = 0; i < 3; i++) c[i] = c_est[i];
  /* for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      printf("%9.5f%c", inv_tfm[i][j], j == 2 ? '\n' : ' ');
  for (int i = 0; i < 3; i++) printf("%9.5f%c", c[i], i == 2 ? '\n' : ' '); */
}

// https://github.com/swedishembedded/control/blob/5327a24485f246f20cc79f98427bd330cb8c2b37/src/linalg/eig_sym.c
// SPDX-License-Identifier: MIT
/**
 * Copyright 2019 Daniel Mårtensson <daniel.martensson100@outlook.com>
 * Copyright 2022 Martin Schröder <info@swedishembedded.com>
 */

#include <float.h>
#include <math.h>
#include <string.h>

// Private functions
static void tqli(float *d, float *e, uint16_t row, float *z);
static void tridiag(float *a, uint16_t row, float *d, float *e);

void eig_sym(const float *const AA, float *ev, float *d, uint16_t row)
{
  float e[row];

  memcpy(ev, AA, sizeof(float) * row * row);

  memset(e, 0, row * sizeof(float));
  memset(d, 0, row * sizeof(float));
  tridiag(ev, row, d, e);
  tqli(d, e, row, ev);
}

// Create a tridiagonal matrix
static void tridiag(float *a, uint16_t row, float *d, float *e)
{
  int l, k, j, i;
  float scale, hh, h, g, f;

  for (i = row - 1; i > 0; i--) {
    l = i - 1;
    h = scale = 0.0f;
    if (l > 0) {
      for (k = 0; k < l + 1; k++) {
        scale += fabsf(*(a + row * i + k)); //fabs(a[i][k]);
      }
      if (scale == 0.0) {
        *(e + i) = *(a + row * i + l); //a[i][l];
      } else {
        for (k = 0; k < l + 1; k++) {
          *(a + row * i + k) /= scale; //a[i][k] /= scale;
          h += *(a + row * i + k) *
               *(a + row * i + k); //a[i][k] * a[i][k];
        }
        f = *(a + row * i + l); //a[i][l];
        g = (f >= 0.0 ? -sqrtf(h) : sqrtf(h));
        *(e + i) = scale * g;
        h -= f * g;
        *(a + row * i + l) = f - g; // a[i][l]
        f = 0.0f;
        for (j = 0; j < l + 1; j++) {
          /* Next statement can be omitted if eigenvectors not wanted */
          *(a + row * j + i) =
            *(a + row * i + j) / h; //a[j][i] = a[i][j] / h;
          g = 0.0f;
          for (k = 0; k < j + 1; k++)
            g += *(a + row * j + k) *
                 *(a + row * i + k); //a[j][k] * a[i][k];
          for (k = j + 1; k < l + 1; k++)
            g += *(a + row * k + j) *
                 *(a + row * i + k); //a[k][j] * a[i][k];
          *(e + j) = g / h;
          f += *(e + j) * *(a + row * i + j); // a[i][j]
        }
        hh = f / (h + h);
        // l + 1
        for (j = 0; j < l + 1; j++) {
          //a[i][j];
          f = *(a + row * i + j);
          *(e + j) = g = *(e + j) - hh * f;
          for (k = 0; k < j + 1; k++) {
            *(a + row * j + k) -=
              (f * e[k] + g * *(a + row * i + k));
          }
        }
      }
    } else {
      *(e + i) = *(a + row * i + l);
    }
    *(d + i) = h;
  }
  /* Next statement can be omitted if eigenvectors not wanted */
  *(d + 0) = 0.0f;
  *(e + 0) = 0.0f;
  /* Contents of this loop can be omitted if eigenvectors not wanted except for statement d[i]=a[i][i]; */
  for (i = 0; i < row; i++) {
    l = i;
    if (fabsf(*(d + i)) > FLT_EPSILON) {
      for (j = 0; j < l; j++) {
        g = 0.0f;
        for (k = 0; k < l; k++) {
          //a[i][k] * a[k][j];
          g += *(a + row * i + k) * *(a + row * k + j);
        }
        for (k = 0; k < l; k++) {
          //a[k][j] -= g * a[k][i];
          *(a + row * k + j) -= g * *(a + row * k + i);
        }
      }
    }
    //a[i][i];
    *(d + i) = *(a + row * i + i);
    //a[i][i] = 1.0;
    *(a + row * i + i) = 1.0f;
    for (j = 0; j < l; j++) {
      //a[j][i] = a[i][j] = 0.0;
      *(a + row * j + i) = *(a + row * i + j) = 0.0f;
    }
  }
}

static void tqli(float *d, float *e, uint16_t row, float *z)
{
  int m, l, iter, i, k;
  float s, r, p, g, f, dd, c, b;

  for (i = 1; i < row; i++)
    *(e + i - 1) = *(e + i);
  e[row - 1] = 0.0f;
  for (l = 0; l < row; l++) {
    iter = 0;
    do {
      for (m = l; m < row - 1; m++) {
        dd = fabsf(*(d + m)) + fabsf(*(d + m + 1));
        if (fabsf(*(e + m)) + dd == dd)
          break;
      }
      if (m != l) {
        if (iter++ == 30) {
          //fprintf(stderr, "[tqli] Too many iterations in tqli.\n");
          break;
        }
        g = (*(d + l + 1) - *(d + l)) / (2.0f * *(e + l));
        r = hypotf(g, 1.0f);
        g = *(d + m) - *(d + l) + *(e + l) / (g + copysignf(r, g));
        s = c = 1.0f;
        p = 0.0f;
        for (i = m - 1; i >= l; i--) {
          f = s * *(e + i);
          b = c * *(e + i);
          e[i + 1] = (r = hypotf(f, g));
          if (fabsf(r) < FLT_EPSILON) {
            *(d + i + 1) -= p;
            *(e + m) = 0.0f;
            break;
          }
          s = f / r;
          c = g / r;
          g = *(d + i + 1) - p;
          r = (*(d + i) - g) * s + 2.0f * c * b;
          *(d + i + 1) = g + (p = s * r);
          g = c * r - b;
          /* Next loop can be omitted if eigenvectors not wanted */
          for (k = 0; k < row; k++) {
            f = *(z + row * k + i + 1); //z[k][i + 1];
            *(z + row * k + i + 1) =
              s * *(z + row * k + i) +
              c * f; //z[k][i + 1] = s * z[k][i] + c * f;
            *(z + row * k + i) =
              c * *(z + row * k + i) -
              s * f; //z[k][i] = c * z[k][i] - s * f;
          }
        }
        if (r == 0.0f && i >= l)
          continue;
        *(d + l) -= p;
        *(e + l) = g;
        *(e + m) = 0.0f;
      }
    } while (m != l);
  }
}
