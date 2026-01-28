#pragma once
#include <stdexcept>

class PIDController {
public:
    enum class IntegralMode { PreSum = 0, PostSum = 1, AntiWindup = 2 };

    PIDController(double Kp = 0.0, double Ti = 0.0, double Td = 0.0,
                  IntegralMode mode = IntegralMode::PostSum);


    void setParams(double Kp, double Ti, double Td);
    void setIntegralMode(IntegralMode newMode);

    void setDFilterAlpha(double alpha);      // alpha in [0,1]

    // anti-windup beta: must be >= 0. Negative values are clamped to 0.
    void setAntiWindupBeta(double beta);     // beta (>=0)
    void reset();

    double step(double e, double dt, double umin = -1e9, double umax = 1e9);


    double getKp() const;
    double getTi() const;
    double getTd() const;
    double debug_getLastE() const;
    double debug_getSumE() const;
    double getP() const;
    double getI() const;
    double getD() const;
    double getDFilterAlpha() const;
    double getAntiWindupBeta() const;

    void getParams(double &Kp, double &Ti, double &Td) const;
    IntegralMode getIntegralMode() const;

private:
    void initState();


    double Kp_;
    double Ti_;
    double Td_;
    IntegralMode mode_;


    bool initialized_;
    double lastE_;
    double sumE_;
    double sumEoverTi_;
    double lastFilteredD_;


    double uP_, uI_, uD_;


    double dFilterAlpha_;
    double antiWindupBeta_;
};
