/*
 * Small dense linear algebra for the per-spot and calibration work: linear
 * solve and the symmetric eigenproblem. Row-major arrays, n <= AT_LA_MAX.
 */
#ifndef AIMTRK_LINALG_H_
#define AIMTRK_LINALG_H_

#include <stdbool.h>

#define AT_LA_MAX 16

/*
 * Solve a x = b in place by Gaussian elimination with partial pivoting:
 * a is n x n (destroyed), b holds x on return. False if a pivot is below
 * 1e-12 times the largest entry of a (singular to working precision).
 */
bool at_solve(double *a, double *b, int n);

/*
 * Eigen-decomposition of a symmetric n x n matrix by cyclic Jacobi rotations.
 * a is destroyed. evals[i] is the i-th eigenvalue (ascending), and column i
 * of evecs (n x n, row-major) its unit eigenvector. False if it did not
 * converge in 100 sweeps.
 */
bool at_sym_eig(double *a, int n, double *evals, double *evecs);

#endif /* AIMTRK_LINALG_H_ */
