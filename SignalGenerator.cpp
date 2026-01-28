#define _USE_MATH_DEFINES
#include "SignalGenerator.h"
#include <cmath>
#include <algorithm>
#include <limits>

SignalGenerator::SignalGenerator()
    : type_(Type::Step),
    A_(1.0),
    S_(0.0),
    TRZ_(1.0),
    duty_(0.5),
    TT_ms_(200),
    T_(1),
    i_(0)
{
    recomputeDiscretePeriod();
}

void SignalGenerator::recomputeDiscretePeriod()
{
    // validate sampling interval and real period
    if (TT_ms_ <= 0) throw std::invalid_argument("TT_ms must be > 0 (milliseconds)");
    if (!(TRZ_ > 0.0) || !std::isfinite(TRZ_)) throw std::invalid_argument("TRZ must be > 0 (seconds)");

    // samples per period = TRZ (s) * (1000 ms/s) / TT_ms (ms)
    double samplesPerPeriod = TRZ_ * (1000.0 / static_cast<double>(TT_ms_));
    if (!std::isfinite(samplesPerPeriod) || samplesPerPeriod < 1.0) samplesPerPeriod = 1.0;

    // round to nearest integer >= 1
    int newT = static_cast<int>(std::round(samplesPerPeriod));
    if (newT < 1) newT = 1;
    T_ = newT;

    // ensure index in valid range
    wrapIndexIfNeeded();
}

void SignalGenerator::wrapIndexIfNeeded()
{
    if (i_ < 0) i_ = 0;
    if (T_ > 0) {
        // keep i_ small to avoid overflow over long runs
        if (i_ >= static_cast<std::int64_t>(T_)) i_ = static_cast<std::int64_t>(i_ % T_);
    } else {
        i_ = 0;
    }
}

void SignalGenerator::setAmplitude(double A)
{
    if (!std::isfinite(A) || A < 0.0) throw std::invalid_argument("Amplitude must be finite and >= 0");
    A_ = A;
}

void SignalGenerator::setOffset(double S)
{
    if (!std::isfinite(S)) throw std::invalid_argument("Offset must be finite");
    S_ = S;
}

void SignalGenerator::setDuty(double p)
{
    if (!std::isfinite(p)) throw std::invalid_argument("Duty must be finite");
    // clamp to [0,1]
    if (p < 0.0) p = 0.0;
    if (p > 1.0) p = 1.0;
    duty_ = p;
}

void SignalGenerator::setTRZ(double TRZ)
{
    if (!std::isfinite(TRZ) || TRZ <= 0.0) throw std::invalid_argument("TRZ must be > 0 (seconds)");
    TRZ_ = TRZ;
    recomputeDiscretePeriod();
}

void SignalGenerator::setTTms(int TTms)
{
    if (TTms <= 0) throw std::invalid_argument("TTms must be > 0 (milliseconds)");
    TT_ms_ = TTms;
    recomputeDiscretePeriod();
}

void SignalGenerator::setType(Type t)
{
    type_ = t;
    reset();
}

void SignalGenerator::reset()
{
    i_ = 0;
    recomputeDiscretePeriod();
}

double SignalGenerator::next()
{
    // guard on T_ just in case
    int localT = (T_ > 0) ? T_ : 1;
    int k = static_cast<int>( (localT > 0) ? (i_ % localT) : 0 );
    double w = 0.0;

    switch (type_) {
    case Type::Sine: {
        double phase = (static_cast<double>(k) / static_cast<double>(localT)) * 2.0 * M_PI;
        w = A_ * std::sin(phase) + S_;
        break;
    }
    case Type::Rect: {
        // threshold number of samples considered "high"
        int threshold = static_cast<int>(std::round(duty_ * static_cast<double>(localT)));
        if (threshold < 0) threshold = 0;
        if (threshold > localT) threshold = localT;
        if (k < threshold)
            w = A_ + S_;
        else
            w = S_;
        break;
    }
    case Type::Step:
    default: {
        // Step: first sample = offset S, subsequent samples = A + S
        w = (i_ == 0) ? S_ : (A_ + S_);
        break;
    }
    }

    // advance index and keep it wrapped to avoid unbounded growth
    ++i_;
    if (i_ >= static_cast<std::int64_t>(localT) && i_ > 1000000) {
        // only perform modulo occasionally to reduce cost, but avoid overflow
        i_ = static_cast<std::int64_t>(i_ % localT);
    } else if (i_ >= static_cast<std::int64_t>(std::numeric_limits<std::int64_t>::max() / 2)) {
        // fail-safe wrap if extremely large
        i_ = static_cast<std::int64_t>(i_ % localT);
    }

    return w;
}

// getters
double SignalGenerator::getAmplitude() const { return A_; }
double SignalGenerator::getOffset() const { return S_; }
double SignalGenerator::getDuty() const { return duty_; }
double SignalGenerator::getTRZ() const { return TRZ_; }
int    SignalGenerator::getTTms() const { return TT_ms_; }
int    SignalGenerator::getT() const { return T_; }
SignalGenerator::Type SignalGenerator::getType() const { return type_; }
