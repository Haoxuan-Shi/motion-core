#ifndef MOTION_CORE_NUMERIC_HPP
#define MOTION_CORE_NUMERIC_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace motion_core {
namespace detail {

[[noreturn]] inline void throw_unrepresentable(const char* operation) {
  throw std::overflow_error(std::string(operation) +
                            " is not representable as a finite double");
}

inline double require_finite_result(double value, const char* operation) {
  if (!std::isfinite(value)) {
    throw_unrepresentable(operation);
  }
  return value;
}

inline bool try_add(double left, double right, double& result) noexcept {
  result = left + right;
  return std::isfinite(result);
}

inline bool try_multiply(double left, double right, double& result) noexcept {
  result = left * right;
  return std::isfinite(result);
}

inline double checked_add(double left, double right, const char* operation) {
  return require_finite_result(left + right, operation);
}

inline double checked_subtract(double left, double right,
                               const char* operation) {
  return require_finite_result(left - right, operation);
}

inline double checked_multiply(double left, double right,
                               const char* operation) {
  return require_finite_result(left * right, operation);
}

inline double checked_divide(double numerator, double denominator,
                             const char* operation) {
  return require_finite_result(numerator / denominator, operation);
}

inline double checked_hypot(double x, double y, const char* operation) {
  return require_finite_result(std::hypot(x, y), operation);
}

inline double checked_atan2_product(double first, double second, double x,
                                    const char* operation) {
  const double product = first * second;
  const bool product_underflowed =
      product == 0.0 && first != 0.0 && second != 0.0;
  if (std::isfinite(product) && !product_underflowed) {
    return require_finite_result(std::atan2(product, x), operation);
  }

  int first_exponent = 0;
  int second_exponent = 0;
  int x_exponent = 0;
  const double first_fraction = std::frexp(first, &first_exponent);
  const double second_fraction = std::frexp(second, &second_exponent);
  const double product_fraction = first_fraction * second_fraction;
  const int product_exponent = first_exponent + second_exponent;
  if (x == 0.0) {
    return require_finite_result(std::atan2(product_fraction, x), operation);
  }

  const double x_fraction = std::frexp(x, &x_exponent);
  const int scale_exponent =
      std::max(product_exponent, x_exponent);
  const double scaled_product =
      std::scalbn(product_fraction, product_exponent - scale_exponent);
  const double scaled_x =
      std::scalbn(x_fraction, x_exponent - scale_exponent);
  return require_finite_result(
      std::atan2(scaled_product, scaled_x), operation);
}

inline double checked_product_quotient(double first, double second,
                                       double denominator,
                                       const char* operation) {
  if (denominator == 0.0) {
    throw_unrepresentable(operation);
  }
  const double product = first * second;
  const bool product_underflowed =
      product == 0.0 && first != 0.0 && second != 0.0;
  if (std::isfinite(product) && !product_underflowed) {
    const double direct = product / denominator;
    if (std::isfinite(direct)) {
      return direct;
    }
  }

  if (first == 0.0 || second == 0.0) {
    return 0.0;
  }

  int first_exponent = 0;
  int second_exponent = 0;
  int denominator_exponent = 0;
  const double first_fraction = std::frexp(first, &first_exponent);
  const double second_fraction = std::frexp(second, &second_exponent);
  const double denominator_fraction =
      std::frexp(denominator, &denominator_exponent);
  const double fraction =
      (first_fraction * second_fraction) / denominator_fraction;
  const int exponent =
      first_exponent + second_exponent - denominator_exponent;
  return require_finite_result(std::scalbn(fraction, exponent), operation);
}

inline double checked_quotient_product(double numerator, double denominator,
                                       double factor,
                                       const char* operation) {
  const double quotient = numerator / denominator;
  const bool quotient_underflowed =
      quotient == 0.0 && numerator != 0.0;
  if (std::isfinite(quotient) && !quotient_underflowed) {
    const double direct = quotient * factor;
    if (std::isfinite(direct)) {
      return direct;
    }
  }
  return checked_product_quotient(numerator, factor, denominator, operation);
}

inline double checked_square_quotient(double value, double denominator,
                                      const char* operation) {
  return checked_product_quotient(value, value, denominator, operation);
}

inline double checked_weighted_square(double weight, double value,
                                      const char* operation) {
  if (weight == 0.0 || value == 0.0) {
    return 0.0;
  }

  const double weighted = weight * value;
  if (std::isfinite(weighted)) {
    const double direct = weighted * value;
    if (std::isfinite(direct)) {
      return direct;
    }
  }

  int weight_exponent = 0;
  int value_exponent = 0;
  const double weight_fraction = std::frexp(weight, &weight_exponent);
  const double value_fraction = std::frexp(value, &value_exponent);
  const double fraction =
      weight_fraction * value_fraction * value_fraction;
  const int exponent = weight_exponent + 2 * value_exponent;
  return require_finite_result(std::scalbn(fraction, exponent), operation);
}

}  // namespace detail
}  // namespace motion_core

#endif
