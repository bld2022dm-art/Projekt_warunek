#include "Simulation.h"
#include "ARXModel.h"
#include "PIDController.h"
#include "SignalGenerator.h"
#include <algorithm>

Simulation::Simulation(std::shared_ptr<ARXModel> arx,
                       std::shared_ptr<PIDController> pid,
                       std::shared_ptr<SignalGenerator> gen)
    : arx_(std::move(arx)),
    pid_(std::move(pid)),
    gen_(std::move(gen)),
    lastY_(0.0), lastU_(0.0), lastW_(0.0),
    lastE_(0.0), lastP_(0.0), lastI_(0.0), lastD_(0.0)
{
}

Simulation::~Simulation() = default;

void Simulation::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (arx_) arx_->reset();
    if (pid_) pid_->reset();
    if (gen_) gen_->reset();

    // Zerujemy wszystkie zmienne stanu
    lastY_ = lastU_ = lastW_ = lastE_ = lastP_ = lastI_ = lastD_ = 0.0;
}

void Simulation::step(double dtSec) {
    if (dtSec <= 0.0) return;

    // 1. Pobranie wartości zadanej (W) z generatora
    // Generator sam dba o upływ czasu (metoda next())
    double w = 0.0;
    if (gen_) {
        w = gen_->next();
    }

    // 2. Odczyt wartości mierzonej (Y) z poprzedniego kroku
    // (Symulacja pętli zamkniętej: sensor czyta stan obiektu z chwili k-1)
    double ymeas = 0.0;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        ymeas = lastY_;
    }

    // 3. Obliczenie uchybu (E)
    double e = w - ymeas;

    // 4. Pobranie ograniczeń sterowania z modelu ARX
    // (PID nie powinien wystawiać sterowania większego niż przyjmuje obiekt)
    double umin = -1e9, umax = 1e9;
    if (arx_ && arx_->isULimitsEnabled()) {
        umin = arx_->getUmin();
        umax = arx_->getUmax();
    }

    // 5. Obliczenie sterowania (U) przez regulator PID
    // PID uwzględnia Anti-Windup dzięki przekazanym limitom umin/umax
    double u = 0.0;
    if (pid_) {
        u = pid_->step(e, dtSec, umin, umax);
    }

    // 6. Symulacja obiektu ARX
    // Obiekt przyjmuje sterowanie U i zwraca nową wartość wyjścia Y
    double y = ymeas;
    if (arx_) {
        // simulateStep wewnątrz obsługuje: nasycenie U -> opóźnienie -> ARX -> szum -> nasycenie Y
        y = arx_->simulateStep(u);
    } else {
        // Fallback jeśli brak modelu
        y = u;
    }

    // 7. Pobranie diagnostyki PID (do wykresów)
    double P = 0.0, I = 0.0, D = 0.0, E_debug = 0.0;
    if (pid_) {
        E_debug = pid_->debug_getLastE(); // Powinno być tożsame z 'e'
        P = pid_->getP();
        I = pid_->getI();
        D = pid_->getD();
    }

    // 8. Aktualizacja stanu symulacji (wątkowo bezpieczna)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        lastW_ = w;
        lastU_ = u;
        lastY_ = y; // To będzie ymeas w następnym kroku
        lastE_ = e;
        lastP_ = P;
        lastI_ = I;
        lastD_ = D;
    }
}

/* component accessors */
std::shared_ptr<ARXModel> Simulation::getARX() const { return arx_; }
std::shared_ptr<PIDController> Simulation::getPID() const { return pid_; }
std::shared_ptr<SignalGenerator> Simulation::getGenerator() const { return gen_; }

/* state getters */
double Simulation::getLastY() const { std::lock_guard<std::mutex> lk(mtx_); return lastY_; }
double Simulation::getLastU() const { std::lock_guard<std::mutex> lk(mtx_); return lastU_; }
double Simulation::getLastW() const { std::lock_guard<std::mutex> lk(mtx_); return lastW_; }
double Simulation::getLastE() const { std::lock_guard<std::mutex> lk(mtx_); return lastE_; }
double Simulation::getLastP() const { std::lock_guard<std::mutex> lk(mtx_); return lastP_; }
double Simulation::getLastI() const { std::lock_guard<std::mutex> lk(mtx_); return lastI_; }
double Simulation::getLastD() const { std::lock_guard<std::mutex> lk(mtx_); return lastD_; }
