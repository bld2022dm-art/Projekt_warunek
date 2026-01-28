#include "PIDController.h"
#include <cmath>

// clamp helper
template<typename T>
static inline T clampT(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

PIDController::PIDController(double Kp, double Ti, double Td, IntegralMode mode)
    : Kp_(Kp), Ti_(Ti), Td_(Td), mode_(mode)
{
    if (Kp < 0.0) throw std::invalid_argument("Kp must be >= 0");
    if (Ti < 0.0) throw std::invalid_argument("Ti must be >= 0");
    if (Td < 0.0) throw std::invalid_argument("Td must be >= 0");
    initState();
}

void PIDController::setParams(double Kp, double Ti, double Td)
{
    if (Kp < 0.0) throw std::invalid_argument("Kp must be >= 0");
    if (Ti < 0.0) throw std::invalid_argument("Ti must be >= 0");
    if (Td < 0.0) throw std::invalid_argument("Td must be >= 0");

    // Handle integrator conversion when Ti changes.
    // Logic ensures bumpless transfer by keeping internal sums consistent.
    if (Ti != Ti_) {
        // First, reconstruct sumE_ if we currently only have sumEoverTi_ meaningful.
        if (Ti_ > 0.0 && (mode_ == IntegralMode::PostSum)) {
            // sumEoverTi_ == sumE_ / Ti_
            sumE_ = sumEoverTi_ * Ti_;
        }

        // Recalculate sumEoverTi_ for the new Ti
        if (Ti > 0.0) {
            sumEoverTi_ = sumE_ / Ti;
        } else {
            sumEoverTi_ = 0.0;
        }
    }

    Kp_ = Kp;
    Ti_ = Ti;
    Td_ = Td;
}

void PIDController::setIntegralMode(IntegralMode newMode)
{
    if (newMode == mode_) return;

    // Maintain consistent integrator state when switching modes
    if (Ti_ > 0.0) {
        if (mode_ == IntegralMode::PreSum && newMode == IntegralMode::PostSum) {
            // convert raw sum -> sum/Ti
            sumEoverTi_ = sumE_ / Ti_;
        } else if (mode_ == IntegralMode::PostSum && newMode == IntegralMode::PreSum) {
            // convert sum/Ti -> raw sum
            sumE_ = sumEoverTi_ * Ti_;
        } else if ((mode_ == IntegralMode::PreSum || mode_ == IntegralMode::PostSum) && newMode == IntegralMode::AntiWindup) {
            // For AntiWindup we use raw-sum representation internally
            if (mode_ == IntegralMode::PostSum) sumE_ = sumEoverTi_ * Ti_;
        } else if (mode_ == IntegralMode::AntiWindup && (newMode == IntegralMode::PreSum || newMode == IntegralMode::PostSum)) {
            // From AntiWindup: if target is PostSum, compute sumEoverTi_
            if (newMode == IntegralMode::PostSum) sumEoverTi_ = sumE_ / Ti_;
        }
    }

    mode_ = newMode;
}

void PIDController::setDFilterAlpha(double alpha) {
    // clamp to [0,1]
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    dFilterAlpha_ = alpha;
}

void PIDController::setAntiWindupBeta(double beta) {
    // clamp to >= 0
    if (beta < 0.0) beta = 0.0;
    antiWindupBeta_ = beta;
}

void PIDController::reset()
{
    initState();
}

double PIDController::step(double e, double dt, double umin, double umax)
{
    if (dt <= 0.0) dt = 1e-6;

    if (!initialized_) {
        lastE_ = e;
        lastFilteredD_ = 0.0;
        initialized_ = true;
    }

    // --- 1. P term (Proportional) ---
    // uP = Kp * e
    uP_ = Kp_ * e;

    // --- 2. I term (Integral) ---
    // INDEPENDENT FORM: uI = (1/Ti) * Integral(e)
    // (Kp does NOT multiply the I term here, consistent with PDF equations)
    if (Ti_ > 0.0) {
        double delta = e * dt;
        if (mode_ == IntegralMode::PostSum) {
            // sumEoverTi_ accumulates (e*dt / Ti)
            sumEoverTi_ += delta / Ti_;
            uI_ = sumEoverTi_; // No Kp_ multiplication

            // keep sumE_ consistent for conversions
            sumE_ = sumEoverTi_ * Ti_;
        } else {
            // PreSum or AntiWindup: use raw-sum representation
            sumE_ += delta;
            sumEoverTi_ = (Ti_ > 0.0) ? (sumE_ / Ti_) : 0.0;
            uI_ = sumEoverTi_; // No Kp_ multiplication
        }
    } else {
        uI_ = 0.0;
    }

    // --- 3. D term (Derivative) ---
    // INDEPENDENT FORM: uD = Td * de/dt
    // (Kp does NOT multiply the D term here)
    if (Td_ > 0.0) {
        double rawD = (e - lastE_) / dt;
        double dTerm = Td_ * rawD; // Removed Kp_ multiplication

        // Apply Low-Pass Filter on D term if alpha is set
        if (dFilterAlpha_ > 0.0 && dFilterAlpha_ < 1.0) {
            uD_ = dFilterAlpha_ * lastFilteredD_ + (1.0 - dFilterAlpha_) * dTerm;
            lastFilteredD_ = uD_;
        } else {
            // Handle edge cases for alpha 0 or 1
            if (dFilterAlpha_ == 1.0) {
                uD_ = lastFilteredD_; // Fully filtered (suppress changes)
            } else {
                uD_ = dTerm; // Raw derivative
                lastFilteredD_ = uD_;
            }
        }
    } else {
        uD_ = 0.0;
    }

    // --- 4. Sum and Saturation ---
    double u_unclamped = uP_ + uI_ + uD_;
    double u_clamped = clampT(u_unclamped, umin, umax);

    // --- 5. Anti-Windup (Back-Calculation) ---
    // Only active in AntiWindup mode
    if (mode_ == IntegralMode::AntiWindup && Ti_ > 0.0 && antiWindupBeta_ != 0.0) {
        double u_error = u_clamped - u_unclamped; // Difference between saturated and calculated

        // Back-calculation correction
        // Since I-term is independent of Kp, we don't divide by Kp here.
        // We feed back the error scaled by Beta to stop integration.
        // Delta = Beta * Error * (dt / Ti) -> approximated for step accumulation

        // Note: We use Ti_ in numerator to match the internal logic of sumE accumulation
        double delta_sumE = antiWindupBeta_ * u_error;

        // Correction applied to the raw integral sum
        sumE_ += delta_sumE;

        // Update derived values for consistency
        sumEoverTi_ = sumE_ / Ti_;
        uI_ = sumEoverTi_; // Re-calculate uI with corrected sum

        // Re-calculate output (optional, but good for consistency in this step)
        u_unclamped = uP_ + uI_ + uD_;
        u_clamped = clampT(u_unclamped, umin, umax);
    }

    lastE_ = e;
    return u_clamped;
}

// getters
double PIDController::getKp() const { return Kp_; }
double PIDController::getTi() const { return Ti_; }
double PIDController::getTd() const { return Td_; }
double PIDController::debug_getLastE() const { return lastE_; }
double PIDController::debug_getSumE() const { return sumE_; }
double PIDController::getP() const { return uP_; }
double PIDController::getI() const { return uI_; }
double PIDController::getD() const { return uD_; }
double PIDController::getDFilterAlpha() const { return dFilterAlpha_; }
double PIDController::getAntiWindupBeta() const { return antiWindupBeta_; }

void PIDController::getParams(double &Kp, double &Ti, double &Td) const {
    Kp = Kp_;
    Ti = Ti_;
    Td = Td_;
}

PIDController::IntegralMode PIDController::getIntegralMode() const {
    return mode_;
}

void PIDController::initState()
{
    lastE_ = 0.0;
    sumE_ = 0.0;
    sumEoverTi_ = 0.0;
    lastFilteredD_ = 0.0;
    uP_ = uI_ = uD_ = 0.0;
    dFilterAlpha_ = 0.0;
    antiWindupBeta_ = 0.0;
    initialized_ = false;
}
