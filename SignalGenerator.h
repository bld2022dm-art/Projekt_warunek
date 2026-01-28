#pragma once
#include <cmath>
#include <stdexcept>
#include <cstdint>

class SignalGenerator
{
public:
    enum class Type { Step, Sine, Rect };

    SignalGenerator();

    // setters
    // A >= 0
    void setAmplitude(double A);

    void setOffset(double S);

    void setDuty(double p);

    void setTRZ(double TRZ);
    // TT in milliseconds (sampling interval) - must be > 0
    void setTTms(int TTms);
    void setType(Type t);
    void reset();

    double next();

    // getters
    double getAmplitude() const;
    double getOffset() const;
    double getDuty() const;
    double getTRZ() const;
    int    getTTms() const;
    int    getT() const;
    Type   getType() const;

private:
    void recomputeDiscretePeriod();
    void wrapIndexIfNeeded();

    Type type_ = Type::Step;

    double A_ = 1.0;
    double S_ = 0.0;
    double TRZ_ = 1.0;
    double duty_ = 0.5;
    int TT_ms_ = 200;
    int T_ = 1;
    std::int64_t i_ = 0;
};
