#pragma once
#include <memory>
#include <mutex>

class ARXModel;
class PIDController;
class SignalGenerator;

class Simulation {
public:
    Simulation(std::shared_ptr<ARXModel> arx,
               std::shared_ptr<PIDController> pid,
               std::shared_ptr<SignalGenerator> gen);

    ~Simulation();

    // lifecycle
    void reset();
    void step(double dtSec); // dt in seconds

    // component accessors
    std::shared_ptr<ARXModel> getARX() const;
    std::shared_ptr<PIDController> getPID() const;
    std::shared_ptr<SignalGenerator> getGenerator() const;

    // state getters (thread-safe)
    double getLastY() const;
    double getLastU() const;
    double getLastW() const;
    double getLastE() const;
    double getLastP() const;
    double getLastI() const;
    double getLastD() const;

private:
    std::shared_ptr<ARXModel> arx_;
    std::shared_ptr<PIDController> pid_;
    std::shared_ptr<SignalGenerator> gen_;

    mutable std::mutex mtx_;
    // last values
    double lastY_;
    double lastU_;
    double lastW_;
    double lastE_;
    double lastP_;
    double lastI_;
    double lastD_;
};
