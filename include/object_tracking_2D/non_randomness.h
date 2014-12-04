#pragma once

namespace prosac  {
  //http://www.uni-koeln.de/rrzk/software/mathematik/matlab_help/techdoc/ref/nchoosek.html
  //http://en.wikipedia.org/wiki/Binomial_coefficient
  //Naive implementations of the factorial formula, such as the following snippet in C:
  //int nchoosek(int n, int k)  {
  //  return factorial(n) / (factorial(k) * factorial(n - k));
  //}
  //A direct implementation of the multiplicative formula works well:
  unsigned long long nchoosek(unsigned n, unsigned k)
  {
    if (k > n)
      return 0;

    if (k > n/2)
      k = n-k; // Take advantage of symmetry

    long double accum = 1;
    unsigned i;
    for (i = 1; i <= k; i++)
      accum = accum * (n-k+i) / i;

    return (unsigned long long) (accum + 0.5); // avoid rounding error
  }

  double randomness_i(double beta, int m, int n, int inlier_num)
  {
    //Compute proba
    return pow(beta,(inlier_num-m)) * pow((1-beta),(n-inlier_num+m)) * nchoosek(n-m,inlier_num-m);
  }

  int non_randomness(int m, int n)
  {
    // evaluate the probability that a model is randomly supported by points, as describe in Eqn. 7 and 8 of Chum05cvpr

    double phi = 0.05;   // we want to make sure that randomness < \phi
    double beta = 0.05;  // the probability of an accidential support

    double p = 0;
    int inlier_num = n;
    while (p < phi)
    {
      p = p + randomness_i(inlier_num, m, n, inlier_num);
      inlier_num = inlier_num - 1;
    }

    if (inlier_num == n) // any inlier number will be too random
    {
      inlier_num = std::numeric_limits<int>::max(); //inf
    }
    else
    {
      inlier_num = inlier_num + 1;          // the minimum solution for i
    }
    return inlier_num;
  }

}; // namespace prosac
