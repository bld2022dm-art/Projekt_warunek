#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>
#include <vector>

// Twoje nagłówki backendu
#include "UARService.h"
#include "ARXModel.h"
#include "PIDController.h"
#include "SignalGenerator.h"
#include "Simulation.h"
#include "secdialog.h"

// Biblioteka wykresów
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // --- SERCE PROGRAMU: Odbiór danych z symulacji ---
    // Ten slot jest wywoływany automatycznie przez UARService co np. 200ms
    void onSimulationUpdate(double time, double w, double y, double e, double u, double uP, double uI, double uD);

    // --- Obsługa UI (Przyciski i Pola) ---

    // Start / Stop / Reset
    void on_pushButton_start_2_clicked();
    void on_pushButton_stop_2_clicked();
    void on_pushButton_reset_ALL_clicked();

    // Reset pojedynczych modułów
    void on_pushButton_resetPID_clicked();
    void on_pushButton_resetGenerator_clicked();

    // Otwarcie okna parametrów ARX
    void onARXButtonClicked();

    // Zapis / Odczyt JSON
    void on_pushButton_save_clicked();
    void on_pushButton_load_clicked();

    // --- Zmiany parametrów "w locie" (EditingFinished) ---
    void syncPIDParams();       // Wysyła Kp, Ti, Td do serwisu
    void syncGeneratorParams(); // Wysyła Amplitudę, Okres itd. do serwisu

    void onSignalTypeChanged(int index);
    void onAntiWindupChanged(bool checked);
    void onIntervalChanged();   // Zmiana czasu próbkowania (dt)

private:
    Ui::MainWindow *ui;

    // Główny obiekt Warstwy Usług (trzyma timer i logikę)
    std::shared_ptr<UARService> service;

    // Wskaźniki do wykresów (dla wygody)
    QCustomPlot *plotFlow = nullptr;    // W i Y
    QCustomPlot *plotError = nullptr;   // E
    QCustomPlot *plotControl = nullptr; // U
    QCustomPlot *plotPID = nullptr;     // P, I, D

    // Grafy na wykresach
    QCPGraph *gW = nullptr;
    QCPGraph *gY = nullptr;
    QCPGraph *gE = nullptr;
    QCPGraph *gU = nullptr;
    QCPGraph *gP = nullptr;
    QCPGraph *gI = nullptr;
    QCPGraph *gD = nullptr;

    QComboBox* comboIntegralMode = nullptr;

    // Ustawienia wizualizacji
    double windowWidth = 20.0; // Szerokość okna w sekundach (ile historii widzimy)

    // Metody pomocnicze
    void setupBackend();       // Tworzenie obiektów (Dependency Injection)
    void setupGraphs();        // Konfiguracja kolorów i osi
    void updateUiFromService(); // Aktualizacja SpinBoxów po wczytaniu pliku
};

#endif // MAINWINDOW_H
