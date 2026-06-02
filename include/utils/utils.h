#pragma once

#include <vector>

#include "units/units.hpp"

double normal_pdf(double x, double mu, double sigma);

float cheap_norm_pdf(const float x);

units::Angle angleDifference(const units::Angle x, const units::Angle y);

float sinc(const units::Angle angle);

double interp(const std::vector<double>& x, const std::vector<double>& y,
              const double x0);

double signnum_c(double x);
