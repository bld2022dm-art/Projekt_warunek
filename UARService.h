#pragma once

#include <QObject>
#include <QTimer>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <memory>
#include <vector>

#include "Simulation.h"
#include "ARXModel.h"
#include "PIDController.h"
#include "SignalGenerator.h"

class UARService : public QObject
{
    Q_OBJECT

public:
    explicit UARService(std::shared_ptr<Simulation> sim, QObject *parent = nullptr);

    // --- Sterowanie ---
    void start();
    void stop();
    void reset();
    bool isRunning() const;

    void setSimulationInterval(int ms);
    int getSimulationInterval() const;

    // --- JSON ---
    bool saveConfig(const QString& filePath);
    bool loadConfig(const QString& filePath);

    // --- Metody dostępowe (Proxy) ---

    // ARX - Podstawowe
    void setARXParams(const std::vector<double>& A, const std::vector<double>& B, int k, double sigma);
    void setARXLimits(double umin, double umax, bool uEnabled, double ymin, double ymax, bool yEnabled);
    std::shared_ptr<ARXModel> getARX() const;

    // --- Metody kompatybilności dla SecDialog (NAPRAWA BŁĘDÓW) ---
    // Te metody pozwolą SecDialog pobrać dane do wyświetlenia
    void getARXVectors(std::vector<double>& A, std::vector<double>& B, int& k) const;
    void getARXLimits(double& umin, double& umax, bool& uEnabled, double& ymin, double& ymax, bool& yEnabled) const;
    double getSigma() const;
    // Alias dla setARXParams (bo SecDialog używa nazwy setARX)
    void setARX(const std::vector<double>& A, const std::vector<double>& B, int k, double sigma);


    // PID
    void setPIDParams(double Kp, double Ti, double Td);
    void setPIDIntegralMode(PIDController::IntegralMode mode);
    void setAntiWindupBeta(double beta);
    void setDFilterAlpha(double alpha);

    // Gettery PID (potrzebne do JSON i GUI)
    void getPIDParams(double& Kp, double& Ti, double& Td) const;
    PIDController::IntegralMode getPIDIntegralMode() const;
    double getAntiWindupBeta() const;
    double getDFilterAlpha() const;
    std::shared_ptr<PIDController> getPID() const;


    // Generator
    void setGeneratorParams(double amp, double offset, double trz, double duty, int tt_ms);
    void setGeneratorType(SignalGenerator::Type type);

    // Gettery Generatora (potrzebne do JSON)
    void getGeneratorParams(double& amp, double& offset, double& trz, double& tt_ms, double& duty) const;
    SignalGenerator::Type getGeneratorType() const;
    std::shared_ptr<SignalGenerator> getGenerator() const;

signals:
    void simulationStepFinished(double time, double w, double y, double e, double u, double uP, double uI, double uD);

private slots:
    void onTimerTick();

private:
    std::shared_ptr<Simulation> sim_;
    QTimer* timer_;
    int intervalMs_ = 200;
    double currentTime_ = 0.0;
};
