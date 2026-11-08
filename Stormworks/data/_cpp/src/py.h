// Python 3.10 compatible numeric helpers: results bit-identical to CPython on the same libm.
#pragma once
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <charconv>
#include <string>
#include <vector>
#include <initializer_list>

namespace py {

// CPython 3.10 Modules/mathmodule.c vector_norm (math.hypot / math.dist)
inline double vector_norm(int n, double* vec, double max, bool found_nan) {
    const double T27 = 134217729.0;
    double x, scale, oldcsum, csum = 1.0, frac1 = 0.0, frac2 = 0.0, frac3 = 0.0;
    double t, hi, lo, h;
    int max_e;
    if (std::isinf(max)) return max;
    if (found_nan) return NAN;
    if (max == 0.0 || n <= 1) return max;
    std::frexp(max, &max_e);
    if (max_e >= -1023) {
        scale = std::ldexp(1.0, -max_e);
        for (int i = 0; i < n; i++) {
            x = vec[i];
            x *= scale;
            t = x * T27;
            hi = t - (t - x);
            lo = x - hi;
            x = hi * hi;
            oldcsum = csum;
            csum += x;
            frac1 += (oldcsum - csum) + x;
            x = 2.0 * hi * lo;
            oldcsum = csum;
            csum += x;
            frac2 += (oldcsum - csum) + x;
            frac3 += lo * lo;
        }
        h = std::sqrt(csum - 1.0 + (frac1 + frac2 + frac3));
        x = h;
        t = x * T27;
        hi = t - (t - x);
        lo = x - hi;
        x = -hi * hi;
        oldcsum = csum;
        csum += x;
        frac1 += (oldcsum - csum) + x;
        x = -2.0 * hi * lo;
        oldcsum = csum;
        csum += x;
        frac2 += (oldcsum - csum) + x;
        x = -lo * lo;
        oldcsum = csum;
        csum += x;
        frac3 += (oldcsum - csum) + x;
        x = csum - 1.0 + (frac1 + frac2 + frac3);
        return (h + x / (2.0 * h)) / scale;
    }
    // subnormal maximum: rescale (never happens with map coordinates)
    for (int i = 0; i < n; i++) vec[i] /= DBL_MIN;
    return DBL_MIN * vector_norm(n, vec, max / DBL_MIN, found_nan);
}

inline double hypot(double a, double b) {
    double v[2] = {std::fabs(a), std::fabs(b)};
    double max = 0.0;
    bool nan = false;
    for (double x : v) { nan |= std::isnan(x); if (x > max) max = x; }
    return vector_norm(2, v, max, nan);
}

inline double hypot3(double a, double b, double c) {
    double v[3] = {std::fabs(a), std::fabs(b), std::fabs(c)};
    double max = 0.0;
    bool nan = false;
    for (double x : v) { nan |= std::isnan(x); if (x > max) max = x; }
    return vector_norm(3, v, max, nan);
}

// math.dist for 2 or 3 coordinates
inline double dist2(double ax, double ay, double bx, double by) {
    double v[2] = {std::fabs(ax - bx), std::fabs(ay - by)};
    double max = 0.0;
    bool nan = false;
    for (double x : v) { nan |= std::isnan(x); if (x > max) max = x; }
    return vector_norm(2, v, max, nan);
}
inline double dist3(double ax, double ay, double az, double bx, double by, double bz) {
    double v[3] = {std::fabs(ax - bx), std::fabs(ay - by), std::fabs(az - bz)};
    double max = 0.0;
    bool nan = false;
    for (double x : v) { nan |= std::isnan(x); if (x > max) max = x; }
    return vector_norm(3, v, max, nan);
}

// float // and % (Objects/floatobject.c float_divmod)
inline void divmod(double vx, double wx, double& floordiv, double& mod) {
    double div;
    mod = std::fmod(vx, wx);
    div = (vx - mod) / wx;
    if (mod) {
        if ((wx < 0) != (mod < 0)) {
            mod += wx;
            div -= 1.0;
        }
    } else {
        mod = std::copysign(0.0, wx);
    }
    if (div) {
        floordiv = std::floor(div);
        if (div - floordiv > 0.5) floordiv += 1.0;
    } else {
        floordiv = std::copysign(0.0, vx / wx);
    }
}
inline double floordiv(double a, double b) { double d, m; divmod(a, b, d, m); return d; }
inline double mod(double a, double b) { double d, m; divmod(a, b, d, m); return m; }
inline long long ifloordiv(double a, double b) { return (long long)floordiv(a, b); }
inline long long ifloordiv(long long a, long long b) {      // int // int
    long long q = a / b, r = a % b;
    if (r != 0 && ((r < 0) != (b < 0))) q -= 1;
    return q;
}

// round(x) -> int (round half to even)
inline long long round_int(double x) {
    double r = std::round(x);
    if (std::fabs(x - r) == 0.5) r = 2.0 * std::round(x / 2.0);
    return (long long)r;
}
// round(x, ndigits) (correctly rounded through the decimal string, as CPython double_round)
inline double round_nd(double x, int nd) {
    if (!std::isfinite(x) || x == 0.0) return x;
    char buf[400];
    std::snprintf(buf, sizeof buf, "%.*f", nd, x);
    double r = std::strtod(buf, nullptr);
    if (r == 0.0) r = std::copysign(0.0, x);
    return r;
}

// repr(float)
inline std::string repr(double x) {
    if (std::isnan(x)) return "nan";
    if (std::isinf(x)) return x > 0 ? "inf" : "-inf";
    char buf[64];
    auto res = std::to_chars(buf, buf + sizeof buf, x, std::chars_format::scientific);
    std::string s(buf, res.ptr);                      // [-]d[.ddd]e[+-]XX
    bool neg = false;
    if (s[0] == '-') { neg = true; s = s.substr(1); }
    size_t e = s.find('e');
    std::string mant = s.substr(0, e);
    int exp10 = std::atoi(s.c_str() + e + 1);
    std::string digits;
    for (char c : mant) if (c != '.') digits += c;
    int decpt = exp10 + 1;                            // value = 0.digits * 10^decpt
    std::string out;
    if (decpt <= -4 || decpt > 16) {
        out = digits.substr(0, 1);
        if (digits.size() > 1) out += "." + digits.substr(1);
        int ex = decpt - 1;
        char eb[16];
        std::snprintf(eb, sizeof eb, "e%c%02d", ex < 0 ? '-' : '+', ex < 0 ? -ex : ex);
        out += eb;
    } else if (decpt <= 0) {
        out = "0." + std::string(-decpt, '0') + digits;
    } else if ((int)digits.size() <= decpt) {
        out = digits + std::string(decpt - digits.size(), '0') + ".0";
    } else {
        out = digits.substr(0, decpt) + "." + digits.substr(decpt);
    }
    return neg ? "-" + out : out;
}

// "%.Nf" % x
inline std::string fmt_f(double x, int n) {
    char buf[400];
    std::snprintf(buf, sizeof buf, "%.*f", n, x);
    return buf;
}

inline double pow(double a, double b) { return std::pow(a, b); }

// sum() of floats: left to right from 0 (int 0 + floats)
inline double sum(const std::vector<double>& v) { double s = 0.0; for (double x : v) s += x; return s; }

}  // namespace py
