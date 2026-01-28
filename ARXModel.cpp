#include "ARXModel.h"
#include <random>
#include <cmath>

ARXModel::ARXModel(const std::vector<double>& A, const std::vector<double>& B, int k, double sigma)
    : A_(A.empty() ? std::vector<double>{0.0} : A),
    B_(B.empty() ? std::vector<double>{0.0} : B),
    k_(std::max(1, k)),
    sigma_(std::max(0.0, sigma)),
    rng_(std::random_device{}())
{
    ybuf_ = std::deque<double>(A_.size(), 0.0);
    ubuf_ = std::deque<double>(B_.size(), 0.0);
    delayBuf_ = std::deque<double>(k_, 0.0);
}

double ARXModel::gaussianNoise() {
    if (sigma_ <= 0.0) return 0.0;
    std::normal_distribution<double> dist(0.0, sigma_);
    return dist(rng_);
}

double ARXModel::simulateStep(double u_raw) {
    // 1. Nasycenie sterowania PRZED obliczeniami [cite: 34]
    double u_in = u_raw;
    if (limU_) {
        if (u_in < umin_) u_in = umin_;
        if (u_in > umax_) u_in = umax_;
    }

    // 2. Obsługa opóźnienia transportowego (FIFO)
    delayBuf_.push_front(u_in);
    double u_delayed = delayBuf_.back();
    delayBuf_.pop_back();

    // 3. Aktualizacja bufora sterowania dla wielomianu B
    ubuf_.push_front(u_delayed);
    if ((int)ubuf_.size() > (int)B_.size()) ubuf_.pop_back();

    // 4. Obliczenie części sterującej (B * u)
    double partB = 0.0;
    for (size_t i = 0; i < B_.size() && i < ubuf_.size(); ++i) partB += B_[i] * ubuf_[i];

    // 5. Obliczenie części autoregresyjnej (A * y)
    double partA = 0.0;
    for (size_t i = 0; i < A_.size() && i < ybuf_.size(); ++i) partA += A_[i] * ybuf_[i];

    // Równanie różnicowe ARX
    double y = partB - partA;

    // 6. Dodanie zakłócenia (szumu)
    if (sigma_ > 0.0) y += gaussianNoise();

    // 7. Nasycenie wyjścia PO obliczeniach, ale PRZED zapisem do historii [cite: 35]
    if (limY_) {
        if (y < ymin_) y = ymin_;
        if (y > ymax_) y = ymax_;
    }

    // 8. Aktualizacja bufora historii wyjść
    ybuf_.push_front(y);
    if ((int)ybuf_.size() > (int)A_.size()) ybuf_.pop_back();

    return y;
}

// --- Settery z zachowaniem ciągłości pamięci (nie resetują symulacji) ---
void ARXModel::setA(const std::vector<double>& A) {
    std::vector<double> newA = A.empty() ? std::vector<double>{0.0} : A;
    std::deque<double> newYbuf(newA.size(), 0.0);
    for (size_t i = 0; i < newYbuf.size() && i < ybuf_.size(); ++i) newYbuf[i] = ybuf_[i];
    ybuf_.swap(newYbuf);
    A_ = std::move(newA);
}

void ARXModel::setB(const std::vector<double>& B) {
    std::vector<double> newB = B.empty() ? std::vector<double>{0.0} : B;
    std::deque<double> newUbuf(newB.size(), 0.0);
    for (size_t i = 0; i < newUbuf.size() && i < ubuf_.size(); ++i) newUbuf[i] = ubuf_[i];
    ubuf_.swap(newUbuf);
    B_ = std::move(newB);
}

void ARXModel::setDelay(int k) {
    if (k < 1) k = 1; // Ograniczenie dolne na 1 [cite: 132]
    std::deque<double> newDelay(k, 0.0);
    for (size_t i = 0; i < newDelay.size() && i < delayBuf_.size(); ++i) newDelay[i] = delayBuf_[i];
    delayBuf_.swap(newDelay);
    k_ = k;
}

void ARXModel::setSigma(double s) {
    sigma_ = std::max(0.0, s);
}

void ARXModel::setULimits(double umin, double umax, bool enabled) {
    if (umin > umax) std::swap(umin, umax);
    umin_ = umin; umax_ = umax; limU_ = enabled;
}

void ARXModel::setYLimits(double ymin, double ymax, bool enabled) {
    if (ymin > ymax) std::swap(ymin, ymax);
    ymin_ = ymin; ymax_ = ymax; limY_ = enabled;
}

void ARXModel::reset() {
    // Pełny reset pamięci (zerowanie buforów)
    std::fill(ybuf_.begin(), ybuf_.end(), 0.0);
    std::fill(ubuf_.begin(), ubuf_.end(), 0.0);
    std::fill(delayBuf_.begin(), delayBuf_.end(), 0.0);
}
