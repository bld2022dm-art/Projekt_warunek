#include "UARService.h"
#include <QDebug>

UARService::UARService(std::shared_ptr<Simulation> sim, QObject *parent)
    : QObject(parent), sim_(sim)
{
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &UARService::onTimerTick);
}

// --- Logika Czasu ---
void UARService::start() {
    if (!timer_->isActive()) timer_->start(intervalMs_);
}

void UARService::stop() {
    timer_->stop();
}

void UARService::reset() {
    stop();
    currentTime_ = 0.0;
    if (sim_) sim_->reset();
    emit simulationStepFinished(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

bool UARService::isRunning() const {
    return timer_->isActive();
}

void UARService::setSimulationInterval(int ms) {
    if (ms < 10) ms = 10;
    intervalMs_ = ms;
    if (timer_->isActive()) timer_->start(intervalMs_);

    if (sim_ && sim_->getGenerator()) {
        sim_->getGenerator()->setTTms(ms);
    }
}

int UARService::getSimulationInterval() const {
    return intervalMs_;
}

void UARService::onTimerTick() {
    if (!sim_) return;
    double dt = static_cast<double>(intervalMs_) / 1000.0;
    currentTime_ += dt;
    sim_->step(dt);
    emit simulationStepFinished(
        currentTime_,
        sim_->getLastW(), sim_->getLastY(), sim_->getLastE(), sim_->getLastU(),
        sim_->getLastP(), sim_->getLastI(), sim_->getLastD()
        );
}

// --- IMPLMENTACJA METOD DLA SECDIALOG (Fix błędów) ---

void UARService::getARXVectors(std::vector<double>& A, std::vector<double>& B, int& k) const {
    if (sim_ && sim_->getARX()) {
        A = sim_->getARX()->getAvec();
        B = sim_->getARX()->getBvec();
        k = sim_->getARX()->getDelay();
    } else {
        A = {1.0}; B = {1.0}; k = 1;
    }
}

void UARService::getARXLimits(double& umin, double& umax, bool& uEnabled, double& ymin, double& ymax, bool& yEnabled) const {
    if (sim_ && sim_->getARX()) {
        umin = sim_->getARX()->getUmin();
        umax = sim_->getARX()->getUmax();
        uEnabled = sim_->getARX()->isULimitsEnabled();
        ymin = sim_->getARX()->getYmin();
        ymax = sim_->getARX()->getYmax();
        yEnabled = sim_->getARX()->isYLimitsEnabled();
    } else {
        umin = -1e9; umax = 1e9; uEnabled = false;
        ymin = -1e9; ymax = 1e9; yEnabled = false;
    }
}

double UARService::getSigma() const {
    if (sim_ && sim_->getARX()) return sim_->getARX()->getSigma();
    return 0.0;
}

void UARService::setARX(const std::vector<double>& A, const std::vector<double>& B, int k, double sigma) {
    setARXParams(A, B, k, sigma);
}

// --- Reszta Setterów/Getterów ---

void UARService::setARXParams(const std::vector<double>& A, const std::vector<double>& B, int k, double sigma) {
    if(sim_ && sim_->getARX()) {
        sim_->getARX()->setA(A);
        sim_->getARX()->setB(B);
        sim_->getARX()->setDelay(k);
        sim_->getARX()->setSigma(sigma);
    }
}

void UARService::setARXLimits(double umin, double umax, bool uEnabled, double ymin, double ymax, bool yEnabled) {
    if(sim_ && sim_->getARX()) {
        sim_->getARX()->setULimits(umin, umax, uEnabled);
        sim_->getARX()->setYLimits(ymin, ymax, yEnabled);
    }
}

std::shared_ptr<ARXModel> UARService::getARX() const { return sim_ ? sim_->getARX() : nullptr; }

void UARService::setPIDParams(double Kp, double Ti, double Td) {
    if(sim_ && sim_->getPID()) sim_->getPID()->setParams(Kp, Ti, Td);
}

void UARService::getPIDParams(double& Kp, double& Ti, double& Td) const {
    if(sim_ && sim_->getPID()) sim_->getPID()->getParams(Kp, Ti, Td);
}

void UARService::setPIDIntegralMode(PIDController::IntegralMode mode) {
    if(sim_ && sim_->getPID()) sim_->getPID()->setIntegralMode(mode);
}

PIDController::IntegralMode UARService::getPIDIntegralMode() const {
    if(sim_ && sim_->getPID()) return sim_->getPID()->getIntegralMode();
    return PIDController::IntegralMode::PostSum;
}

void UARService::setAntiWindupBeta(double beta) {
    if(sim_ && sim_->getPID()) sim_->getPID()->setAntiWindupBeta(beta);
}

double UARService::getAntiWindupBeta() const {
    if(sim_ && sim_->getPID()) return sim_->getPID()->getAntiWindupBeta();
    return 0.0;
}

void UARService::setDFilterAlpha(double alpha) {
    if(sim_ && sim_->getPID()) sim_->getPID()->setDFilterAlpha(alpha);
}

double UARService::getDFilterAlpha() const {
    if(sim_ && sim_->getPID()) return sim_->getPID()->getDFilterAlpha();
    return 0.0;
}

std::shared_ptr<PIDController> UARService::getPID() const { return sim_ ? sim_->getPID() : nullptr; }

void UARService::setGeneratorParams(double amp, double offset, double trz, double duty, int tt_ms) {
    if(sim_ && sim_->getGenerator()) {
        sim_->getGenerator()->setAmplitude(amp);
        sim_->getGenerator()->setOffset(offset);
        sim_->getGenerator()->setTRZ(trz);
        sim_->getGenerator()->setDuty(duty);
        sim_->getGenerator()->setTTms(tt_ms);
    }
}

void UARService::getGeneratorParams(double& amp, double& offset, double& trz, double& tt_ms, double& duty) const {
    if(sim_ && sim_->getGenerator()) {
        amp = sim_->getGenerator()->getAmplitude();
        offset = sim_->getGenerator()->getOffset();
        trz = sim_->getGenerator()->getTRZ();
        tt_ms = static_cast<double>(sim_->getGenerator()->getTTms());
        duty = sim_->getGenerator()->getDuty();
    }
}

void UARService::setGeneratorType(SignalGenerator::Type type) {
    if(sim_ && sim_->getGenerator()) sim_->getGenerator()->setType(type);
}

SignalGenerator::Type UARService::getGeneratorType() const {
    if(sim_ && sim_->getGenerator()) return sim_->getGenerator()->getType();
    return SignalGenerator::Type::Step;
}

std::shared_ptr<SignalGenerator> UARService::getGenerator() const { return sim_ ? sim_->getGenerator() : nullptr; }

// --- JSON --- (Tutaj skróciłem dla czytelności, wklej tu pełną wersję z poprzedniej odpowiedzi, jeśli jej brakuje, ale logika getterów teraz zadziała)

bool UARService::saveConfig(const QString& filePath) {
    if (!sim_) return false;
    QJsonObject root;

    // ARX
    if (auto arx = sim_->getARX()) {
        QJsonObject jArx;
        QJsonArray jA, jB;
        for (double v : arx->getAvec()) jA.append(v);
        for (double v : arx->getBvec()) jB.append(v);
        jArx["A"] = jA; jArx["B"] = jB;
        jArx["Delay"] = arx->getDelay();
        jArx["Sigma"] = arx->getSigma();
        jArx["MinU"] = arx->getUmin(); jArx["MaxU"] = arx->getUmax(); jArx["LimitU_Enabled"] = arx->isULimitsEnabled();
        jArx["MinY"] = arx->getYmin(); jArx["MaxY"] = arx->getYmax(); jArx["LimitY_Enabled"] = arx->isYLimitsEnabled();
        root["ARX"] = jArx;
    }

    // PID
    if (auto pid = sim_->getPID()) {
        QJsonObject jPid;
        jPid["Kp"] = pid->getKp(); jPid["Ti"] = pid->getTi(); jPid["Td"] = pid->getTd();
        jPid["Mode"] = static_cast<int>(pid->getIntegralMode());
        jPid["Beta"] = pid->getAntiWindupBeta(); jPid["Alpha"] = pid->getDFilterAlpha();
        root["PID"] = jPid;
    }

    // Generator
    if (auto gen = sim_->getGenerator()) {
        QJsonObject jGen;
        jGen["Type"] = static_cast<int>(gen->getType());
        jGen["Amp"] = gen->getAmplitude(); jGen["Offset"] = gen->getOffset();
        jGen["TRZ"] = gen->getTRZ(); jGen["Duty"] = gen->getDuty();
        root["Generator"] = jGen;
    }
    root["IntervalMs"] = intervalMs_;

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) return false;
    file.write(QJsonDocument(root).toJson());
    return true;
}

bool UARService::loadConfig(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) return false;
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (doc.isNull()) return false;
    QJsonObject root = doc.object();

    if (root.contains("ARX") && sim_->getARX()) {
        QJsonObject o = root["ARX"].toObject();
        std::vector<double> A, B;
        for (auto v : o["A"].toArray()) A.push_back(v.toDouble());
        for (auto v : o["B"].toArray()) B.push_back(v.toDouble());
        sim_->getARX()->setA(A); sim_->getARX()->setB(B);
        sim_->getARX()->setDelay(o["Delay"].toInt());
        sim_->getARX()->setSigma(o["Sigma"].toDouble());
        sim_->getARX()->setULimits(o["MinU"].toDouble(), o["MaxU"].toDouble(), o["LimitU_Enabled"].toBool());
        sim_->getARX()->setYLimits(o["MinY"].toDouble(), o["MaxY"].toDouble(), o["LimitY_Enabled"].toBool());
    }

    if (root.contains("PID") && sim_->getPID()) {
        QJsonObject o = root["PID"].toObject();
        sim_->getPID()->setParams(o["Kp"].toDouble(), o["Ti"].toDouble(), o["Td"].toDouble());
        sim_->getPID()->setIntegralMode(static_cast<PIDController::IntegralMode>(o["Mode"].toInt()));
        sim_->getPID()->setAntiWindupBeta(o["Beta"].toDouble());
        sim_->getPID()->setDFilterAlpha(o["Alpha"].toDouble());
    }

    if (root.contains("Generator") && sim_->getGenerator()) {
        QJsonObject o = root["Generator"].toObject();
        sim_->getGenerator()->setType(static_cast<SignalGenerator::Type>(o["Type"].toInt()));
        sim_->getGenerator()->setAmplitude(o["Amp"].toDouble());
        sim_->getGenerator()->setOffset(o["Offset"].toDouble());
        sim_->getGenerator()->setTRZ(o["TRZ"].toDouble());
        sim_->getGenerator()->setDuty(o["Duty"].toDouble());
    }

    if (root.contains("IntervalMs")) setSimulationInterval(root["IntervalMs"].toInt());

    reset();
    return true;
}
