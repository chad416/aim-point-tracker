/*
 * Small dense linear algebra (see linalg.h).
 */
#include <math.h>

#include "aimtrk/linalg.h"

bool at_solve(double *a, double *b, int n)
{
	double scale = 0.0;

	for (int i = 0; i < (n * n); i++) {
		scale = fmax(scale, fabs(a[i]));
	}
	double tiny = 1e-12 * scale;

	if (scale == 0.0) {
		return false;
	}
	for (int k = 0; k < n; k++) {
		int p = k;

		for (int r = k + 1; r < n; r++) {
			if (fabs(a[(r * n) + k]) > fabs(a[(p * n) + k])) {
				p = r;
			}
		}
		if (fabs(a[(p * n) + k]) <= tiny) {
			return false;
		}
		if (p != k) {
			for (int c = 0; c < n; c++) {
				double t = a[(k * n) + c];

				a[(k * n) + c] = a[(p * n) + c];
				a[(p * n) + c] = t;
			}
			double t = b[k];

			b[k] = b[p];
			b[p] = t;
		}
		for (int r = k + 1; r < n; r++) {
			double f = a[(r * n) + k] / a[(k * n) + k];

			for (int c = k; c < n; c++) {
				a[(r * n) + c] -= f * a[(k * n) + c];
			}
			b[r] -= f * b[k];
		}
	}
	for (int k = n - 1; k >= 0; k--) {
		double s = b[k];

		for (int c = k + 1; c < n; c++) {
			s -= a[(k * n) + c] * b[c];
		}
		b[k] = s / a[(k * n) + k];
	}
	return true;
}

bool at_sym_eig(double *a, int n, double *evals, double *evecs)
{
	bool converged = false;

	for (int i = 0; i < (n * n); i++) {
		evecs[i] = ((i / n) == (i % n)) ? 1.0 : 0.0;
	}
	for (int sweep = 0; sweep < 100; sweep++) {
		double off = 0.0;
		double total = 0.0;

		for (int p = 0; p < n; p++) {
			for (int q = 0; q < n; q++) {
				double v = a[(p * n) + q] * a[(p * n) + q];

				total += v;
				if (p != q) {
					off += v;
				}
			}
		}
		if (off <= (1e-30 * total)) {
			converged = true;
			break;
		}
		for (int p = 0; p < (n - 1); p++) {
			for (int q = p + 1; q < n; q++) {
				double apq = a[(p * n) + q];

				if (apq == 0.0) {
					continue;
				}
				/* Rotation that zeroes a[p][q] (Golub & Van Loan, Algorithm 8.5.1). */
				double theta = (a[(q * n) + q] - a[(p * n) + p]) / (2.0 * apq);
				double t = ((theta >= 0.0) ? 1.0 : -1.0) / (fabs(theta) + sqrt((theta * theta) + 1.0));
				double c = 1.0 / sqrt((t * t) + 1.0);
				double s = t * c;

				for (int k = 0; k < n; k++) {
					double akp = a[(k * n) + p];
					double akq = a[(k * n) + q];

					a[(k * n) + p] = (c * akp) - (s * akq);
					a[(k * n) + q] = (s * akp) + (c * akq);
				}
				for (int k = 0; k < n; k++) {
					double apk = a[(p * n) + k];
					double aqk = a[(q * n) + k];

					a[(p * n) + k] = (c * apk) - (s * aqk);
					a[(q * n) + k] = (s * apk) + (c * aqk);
				}
				for (int k = 0; k < n; k++) {
					double vkp = evecs[(k * n) + p];
					double vkq = evecs[(k * n) + q];

					evecs[(k * n) + p] = (c * vkp) - (s * vkq);
					evecs[(k * n) + q] = (s * vkp) + (c * vkq);
				}
			}
		}
	}
	/* Sort ascending (selection sort, swapping eigenvector columns). */
	for (int i = 0; i < n; i++) {
		evals[i] = a[(i * n) + i];
	}
	for (int i = 0; i < (n - 1); i++) {
		int m = i;

		for (int j = i + 1; j < n; j++) {
			if (evals[j] < evals[m]) {
				m = j;
			}
		}
		if (m != i) {
			double t = evals[i];

			evals[i] = evals[m];
			evals[m] = t;
			for (int k = 0; k < n; k++) {
				t = evecs[(k * n) + i];
				evecs[(k * n) + i] = evecs[(k * n) + m];
				evecs[(k * n) + m] = t;
			}
		}
	}
	return converged;
}
