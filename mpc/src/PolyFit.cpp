#include "PolyFit.h"

using namespace std;


PolyFit::PolyFit(){}

PolyFit::PolyFit(vector<Point> in)
{
  input = in;
}

PolyFit::~PolyFit()
{}

void PolyFit::setInput(vector<Point> in)
{
  input = in;
}

vector<double> PolyFit::solve()
{
  output.clear();
  for(int i = 0; i <= DEGREE; ++i)
    output.push_back(0);
  if(input.size() == 0)
    return output;

  int n, N;
  N = input.size();
  double *x = new double[N];
  double *y = new double[N];

  for(int i = 0; i < input.size(); ++i)
  {
    x[i] = input[i].x;
    y[i] = input[i].y;
  }

  n = DEGREE;
  double *X = new double[2*n+1];
  for(int i = 0; i < 2*n+1; i++)
  {
    X[i] = 0;
    for(int j = 0; j < N; j++)
      X[i] = X[i] + pow(x[j], i);
  }
  double **B = new double*[n+1];
  for(int i = 0; i < n+1; ++i)
  {
    B[i] = new double[n+2];
  }
  double *a = new double[n+1];
  for(int i = 0; i <= n; i++)
    for(int j = 0; j <= n; j++)
      B[i][j] = X[i+j];

  double *Y = new double[n+1];
  for(int i = 0; i < n+1; i++)
  {
    Y[i] = 0;
    for(int j = 0; j < N; j++)
      Y[i] = Y[i] + pow(x[j], i) * y[j];
  }
  for(int i = 0; i <= n; i++)
    B[i][n+1] = Y[i];
  n = n+1;

  for(int i = 0; i < n; i++)
    for(int k = i+1; k < n; k++)
      if(B[i][i] < B[k][i])
        for(int j = 0; j <= n; j++)
        {
          double temp = B[i][j];
          B[i][j] = B[k][j];
          B[k][j] = temp;
        }

  for(int i = 0; i < n-1; i++)
    for(int k = i+1; k < n; k++)
    {
      double t = B[k][i]/B[i][i];
      for(int j = 0; j <= n; j++)
        B[k][j] = B[k][j] - t*B[i][j];
    }

  for(int i = n-1; i >= 0; i--)
  {
    a[i] = B[i][n];
    for(int j = 0; j < n; j++)
    if(j != i)
      a[i] = a[i] - B[i][j]*a[j];
    a[i] = a[i] / B[i][i];
  }

  for(int i = 0; i <= DEGREE; ++i)
  {
    output[i] = a[i];
  }
  for(int i = 0; i < n; ++i)
  {
    delete[] B[i];
  }
  delete[] x, y, X, B, a, Y;

  return output;
}


double value(int x, vector<double> coeffs)
{
  double result = 0;
  double a = 1;
  for(int i = 0; i < coeffs.size(); ++i)
  {
    result += coeffs[i]*a;
    a *= (double)x;
  }
  return result;
}
