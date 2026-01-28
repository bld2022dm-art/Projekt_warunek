#pragma once

#include <vector>
#include <deque>
#include <random>
#include <stdexcept>
#include <algorithm>

class ARXModel {
public:
    ARXModel(const std::vector<double>& A = {1.0}, const std::vector<double>& B = {1.0}, int k = 1, double sigma = 0.0);


    double simulateStep(double u_raw);


    void setA(const std::vector<double>& A);
    void setB(const std::vector<double>& B);
    void setDelay(int k);
    void setSigma(double s);


    void setULimits(double umin, double umax, bool enabled);
    void setYLimits(double ymin, double ymax, bool enabled);

    void reset();

    const std::vector<double>& getAvec() const { return A_; }
    const std::vector<double>& getBvec() const { return B_; }
    int getDelay() const { return k_; }
    double getSigma() const { return sigma_; }

    bool isULimitsEnabled() const { return limU_; }
    bool isYLimitsEnabled() const { return limY_; }
    double getUmin() const { return umin_; }
    double getUmax() const { return umax_; }
    double getYmin() const { return ymin_; }
    double getYmax() const { return ymax_; }

private:
    std::vector<double> A_;
    std::vector<double> B_;
    int k_; // Opóźnienie transportowe
    double sigma_; // Odchylenie standardowe szumu

    std::deque<double> ybuf_;
    std::deque<double> ubuf_;
    std::deque<double> delayBuf_;

    // Zmienne obsługujące ograniczenia (nasycenie)
    bool limU_ = false;
    bool limY_ = false;
    double umin_ = -1e9, umax_ = 1e9;
    double ymin_ = -1e9, ymax_ = 1e9;

    std::mt19937 rng_;

    double gaussianNoise();
};
