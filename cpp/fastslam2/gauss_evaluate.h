#ifndef GAUSS_EVALUATE_H
#define GAUSS_EVALUATE_H

#include <Eigen/Dense>

using namespace Eigen;

double gauss_evaluate(VectorXd v, MatrixXd S, int logflag);

/*
**INPUTS:
**    v - a set of innovation vectors
**    S - the covariance matrix for the innovations
**    logflat - <optional> - if 1 computes the log-likelihood, otherwise computes
**        the likelihood.
**
**OUTPUT:
**    w- set of Gaussian likelihoods or log-likelihoods for ech v(:i).
**    
**This implementaiton uses the Cholesky factor of S to compute the likelihoods
**and so is more numerically stable than a simple full covariance form.
**This function is identical to gauss_likelihood().
*/

#endif

