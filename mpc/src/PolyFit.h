#ifndef POLYFIT_H
#define POLYFIT_H

#include <vector>
#include <cmath>

#define DEGREE 2

struct Point
{
  int x;
  int y;
};

double value(int x, std::vector<double> coeffs);

class PolyFit
{
  std::vector<Point> input;
  std::vector<double> output;

public:
  PolyFit();
  PolyFit(std::vector<Point> in);
  ~PolyFit();

  void setInput(std::vector<Point> in);

  std::vector<double> solve();
};


#endif
