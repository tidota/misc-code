// finding a minimum of a convex function
// https://www.gnu.org/software/gsl/doc/html/min.html#examples

#include <iostream>
#include <stdio.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_min.h>

#include <Eigen/Dense>

using namespace Eigen;

struct func_params
{
  MatrixXd invA;
  MatrixXd invB;
};

double fn1 (double x, void * params)
{
  struct func_params *p = (struct func_params*)params;
  MatrixXd invA(p->invA);
  MatrixXd invB(p->invB);
  MatrixXd C = (x * invA + (1-x) * invB).inverse();
  // MatrixXd A = MatrixXd::Zero(2,2);
  // A << 0.0227047020, 0.0165085573, 0.0165085573, 0.0227047020;
  // MatrixXd B = MatrixXd::Zero(2,2);
  // B << 0.0316646858, -0.0023912993, -0.0023912993, 0.0348561699;
  // MatrixXd C = (x * A.inverse() + (1-x) * B.inverse()).inverse();
  return C.trace();
}

int
main (void)
{
  int status;
  int iter = 0, max_iter = 100;
  const gsl_min_fminimizer_type *T;
  gsl_min_fminimizer *s;
  double m = 0.5;
  double a = 0.0, b = 1.0;
  gsl_function F;

  MatrixXd A = MatrixXd::Zero(2,2);
  A << 0.0227047020, 0.0165085573, 0.0165085573, 0.0227047020;
  MatrixXd B = MatrixXd::Zero(2,2);
  B << 0.0316646858, -0.0023912993, -0.0023912993, 0.0348561699;

  struct func_params prm;
  prm.invA = A.inverse();
  prm.invB = B.inverse();

  F.function = &fn1;
  F.params = (void*)&prm;


  T = gsl_min_fminimizer_brent;
  std::cout << "1" << std::endl;
  s = gsl_min_fminimizer_alloc (T);
  std::cout << "2" << std::endl;
  gsl_min_fminimizer_set (s, &F, m, a, b);
  std::cout << "3" << std::endl;

  printf ("using %s method\n",
          gsl_min_fminimizer_name (s));

  printf ("%5s [%9s, %9s] %9s %9s\n",
          "iter", "lower", "upper", "min",
          "err(est)");

  printf ("%5d [%.7f, %.7f] %.7f %.7f\n",
          iter, a, b,
          m, b - a);

  do
    {
      iter++;
      status = gsl_min_fminimizer_iterate (s);

      m = gsl_min_fminimizer_x_minimum (s);
      a = gsl_min_fminimizer_x_lower (s);
      b = gsl_min_fminimizer_x_upper (s);

      status
        = gsl_min_test_interval (a, b, 0.001, 0.0);

      if (status == GSL_SUCCESS)
        printf ("Converged:\n");

      printf ("%5d [%.7f, %.7f] "
              "%.7f %.7f\n",
              iter, a, b,
              m, b - a);
    }
  while (status == GSL_CONTINUE && iter < max_iter);

  gsl_min_fminimizer_free (s);

  return status;
}
