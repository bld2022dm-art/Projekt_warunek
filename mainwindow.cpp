#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "secdialog.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 1. Inicjalizacja Backendu
    setupBackend();

    // 2. Konfiguracja Wykresów
    setupGraphs();

    // 3. --- BEZPIECZNE WARTOŚCI STARTOWE (TO NAPRAWIA BŁĄD TRZ > 0) ---
    // Musimy ustawić spinboxy na wartości > 0 zanim wywołamy syncGeneratorParams
    ui->periodSpinBox->setValue(10.0);       // TRZ = 10 sekund (bezpieczne)
    ui->amplitudeSpinBox->setValue(1.0);     // Amplituda
    ui->doubleSpinBox_interval->setValue(0.2); // Interwał symulacji
    ui->kpSpinBox->setValue(1.0);
    ui->tiSpinBox->setValue(5.0);
    ui->tdSpinBox->setValue(0.1);

    // Uzupełnienie ComboBoxa jeśli pusty
    if (ui->signalTypeComboBox->count() == 0) {
        ui->signalTypeComboBox->addItem("Skok (Step)");
        ui->signalTypeComboBox->addItem("Sinus");
        ui->signalTypeComboBox->addItem("Prostokąt");
    }
    ui->signalTypeComboBox->setCurrentIndex(0); // Ustaw Skok

    // 4. Podpięcie zdarzeń UI
    connect(service.get(), &UARService::simulationStepFinished,
            this, &MainWindow::onSimulationUpdate);

    connect(ui->pushButton_start_2, &QPushButton::clicked, this, &MainWindow::on_pushButton_start_2_clicked);
    connect(ui->pushButton_stop_2, &QPushButton::clicked, this, &MainWindow::on_pushButton_stop_2_clicked);
    connect(ui->pushButton_reset_ALL, &QPushButton::clicked, this, &MainWindow::on_pushButton_reset_ALL_clicked);

    connect(ui->pushButton_resetPID, &QPushButton::clicked, this, &MainWindow::on_pushButton_resetPID_clicked);
    connect(ui->pushButton_resetGenerator, &QPushButton::clicked, this, &MainWindow::on_pushButton_resetGenerator_clicked);
    connect(ui->pushButton_ARX, &QPushButton::clicked, this, &MainWindow::onARXButtonClicked);

    if (ui->pushButton_save) connect(ui->pushButton_save, &QPushButton::clicked, this, &MainWindow::on_pushButton_save_clicked);
    if (ui->pushButton_load) connect(ui->pushButton_load, &QPushButton::clicked, this, &MainWindow::on_pushButton_load_clicked);

    connect(ui->kpSpinBox, &QDoubleSpinBox::editingFinished, this, &MainWindow::syncPIDParams);
    connect(ui->tiSpinBox, &QDoubleSpinBox::editingFinished, this, &MainWindow::syncPIDParams);
    connect(ui->tdSpinBox, &QDoubleSpinBox::editingFinished, this, &MainWindow::syncPIDParams);

    connect(ui->amplitudeSpinBox, &QDoubleSpinBox::editingFinished, this, &MainWindow::syncGeneratorParams);
    connect(ui->periodSpinBox, &QDoubleSpinBox::editingFinished, this, &MainWindow::syncGeneratorParams);
    connect(ui->offsetSpinBox, &QDoubleSpinBox::editingFinished, this, &MainWindow::syncGeneratorParams);
    connect(ui->doubleSpinBox_infil, &QDoubleSpinBox::editingFinished, this, &MainWindow::syncGeneratorParams);

    connect(ui->signalTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onSignalTypeChanged);

    connect(ui->antiWindupCheckBox, &QCheckBox::toggled, this, &MainWindow::onAntiWindupChanged);
    connect(ui->doubleSpinBox_interval, &QDoubleSpinBox::editingFinished, this, &MainWindow::onIntervalChanged);

    // 5. Wymuszamy pierwsze przesłanie parametrów (W TRY-CATCH)
    // To zapobiegnie crashowi, nawet jeśli coś jest nie tak
    try {
        syncPIDParams();
        syncGeneratorParams();
        onIntervalChanged();
    } catch (const std::exception& e) {
        qWarning() << "Błąd inicjalizacji parametrów:" << e.what();
    }

    ui->pushButton_start_2->setEnabled(true);
    ui->pushButton_stop_2->setEnabled(false);

    ui->doubleSpinBox_infil->setRange(0.0, 1.0); // Wypełnienie to ułamek od 0 do 1
    ui->doubleSpinBox_infil->setSingleStep(0.1);
    ui->doubleSpinBox_infil->setValue(0.5);      // Domyślnie 50%

    if (ui->comboBox_integralMode->count() == 0) {
        // Index 0: To jest domyślny tryb (zgodnie z instrukcją )
        ui->comboBox_integralMode->addItem("Stała PRZED sumą (Standard)");

        // Index 1: Alternatywny tryb
        ui->comboBox_integralMode->addItem("Stała POD sumą (Niezależna)");
    }

    connect(ui->comboBox_integralMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::syncPIDParams);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupBackend()
{
    // Tworzenie "cegiełek" (Smart Pointers)
    auto arx = std::make_shared<ARXModel>();

    // --- REALISTYCZNE DOMYŚLNE WARTOŚCI ---
    arx->setA({-0.5});
    arx->setB({0.5});
    arx->setDelay(1);

    // LIMIT U (Sterowanie): Standardowo w automatyce to -10 do 10 (np. napięcie) lub 0 do 100 (PWM)
    // Ustawiamy -10 do 10 i WŁĄCZAMY (true)
    arx->setULimits(-10.0, 10.0, true);

    // LIMIT Y (Wyjście obiektu): Np. temperatura.
    // Ustawiamy szeroko -100 do 100, ale domyślnie WYŁĄCZONE (false), bo ARX sam z siebie nie ma ścian
    arx->setYLimits(-100.0, 100.0, false);

    auto pid = std::make_shared<PIDController>(1.0, 5.0, 0.0, PIDController::IntegralMode::PostSum);
    auto gen = std::make_shared<SignalGenerator>();

    auto sim = std::make_shared<Simulation>(arx, pid, gen);
    service = std::make_shared<UARService>(sim, this);
}

void MainWindow::setupGraphs()
{
    // Przypisanie wskaźników z UI
    plotFlow = ui->widget_czasowy;
    plotError = ui->widget_E;
    plotControl = ui->widget_U;
    plotPID = ui->widget_PID;

    // Funkcja stylizująca (szary techniczny styl)
    auto stylePlot = [](QCustomPlot* plot) {
        if(!plot) return;
        plot->setBackground(Qt::NoBrush);
        plot->legend->setVisible(true);
        plot->legend->setBrush(QBrush(QColor(255,255,255,150)));
        plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignRight);
    };

    // 1. Zadana (W) i Regulowana (Y)
    stylePlot(plotFlow);
    if(plotFlow) {
        gW = plotFlow->addGraph();
        gW->setName("Zadana (W)");
        gW->setPen(QPen(Qt::blue, 2, Qt::DashLine));

        gY = plotFlow->addGraph();
        gY->setName("Wyjście (Y)");
        gY->setPen(QPen(Qt::green, 2));
    }

    // 2. Uchyb (E)
    stylePlot(plotError);
    if(plotError) {
        gE = plotError->addGraph();
        gE->setName("Uchyb (E)");
        gE->setPen(QPen(Qt::red, 2));
    }

    // 3. Sterowanie (U)
    stylePlot(plotControl);
    if(plotControl) {
        gU = plotControl->addGraph();
        gU->setName("Sterowanie (U)");
        gU->setPen(QPen(QColor(255, 140, 0), 2)); // Pomarańczowy
    }

    // 4. PID
    stylePlot(plotPID);
    if(plotPID) {
        gP = plotPID->addGraph(); gP->setName("P"); gP->setPen(QPen(Qt::red));
        gI = plotPID->addGraph(); gI->setName("I"); gI->setPen(QPen(Qt::green));
        gD = plotPID->addGraph(); gD->setName("D"); gD->setPen(QPen(Qt::blue));
    }
}

// --- SLOT GLOWNY: Rysowanie Wykresów ---
void MainWindow::onSimulationUpdate(double time, double w, double y, double e, double u, double uP, double uI, double uD)
{

    // --- SZPIEG START ---
    // Wypisujemy co 1 sekundę (żeby nie zalać konsoli), jakie są parametry
    static double lastPrint = -10.0;
    if (time - lastPrint > 1.0) {
        lastPrint = time;

        // Pobieramy to, co widzi SERWIS (a nie GUI)
        std::vector<double> A, B; int k;
        service->getARXVectors(A, B, k);

        double amp, off, trz, tt, duty;
        service->getGeneratorParams(amp, off, trz, tt, duty);

        qDebug() << "CZAS:" << time;
        qDebug() << "  ARX A1:" << (A.size()>0 ? A[0] : 0.0) << " (Powinno byc -0.9)";
        qDebug() << "  GEN Duty:" << duty << " (Powinno byc 0.5)";
        qDebug() << "  GEN Typ:" << (int)service->getGeneratorType() << " (2 = Prostokat)";
    }
    // 1. Dodaj dane do grafów
    if(gW) gW->addData(time, w);
    if(gY) gY->addData(time, y);
    if(gE) gE->addData(time, e);
    if(gU) gU->addData(time, u);
    if(gP) gP->addData(time, uP);
    if(gI) gI->addData(time, uI);
    if(gD) gD->addData(time, uD);

    // 2. Obsługa przesuwanego okna (Rolling Window)
    // Pobieramy długość trwania z UI (jeśli chcesz, by użytkownik tym sterował)
    // Ale w Twoim UI to pole nazywa się 'doubleSpinBox_duration' - użyjmy go jako szerokość okna
    double userWindow = ui->doubleSpinBox_duration->value();
    if (userWindow < 1.0) userWindow = 10.0;
    windowWidth = userWindow;

    double keyMax = time;
    double keyMin = (keyMax > windowWidth) ? (keyMax - windowWidth) : 0.0;

    // 3. Usuwanie starych danych (Optymalizacja pamięci)
    // Usuwamy dane starsze niż okno + margines, żeby nie zapchać RAMu
    double deletePoint = keyMin - 5.0;
    if (deletePoint > 0) {
        if(gW) gW->data()->removeBefore(deletePoint);
        if(gY) gY->data()->removeBefore(deletePoint);
        if(gE) gE->data()->removeBefore(deletePoint);
        if(gU) gU->data()->removeBefore(deletePoint);
        if(gP) gP->data()->removeBefore(deletePoint);
    }

    // 4. Skalowanie Osi
    auto updateAxis = [&](QCustomPlot* p) {
        if(!p) return;
        p->xAxis->setRange(keyMin, keyMax);
        p->yAxis->rescale(true); // Auto-scale Y based on visible data
        // Dodaj mały margines góra/dół dla estetyki
        double rLower = p->yAxis->range().lower;
        double rUpper = p->yAxis->range().upper;
        double diff = rUpper - rLower;
        if(diff < 0.1) diff = 1.0;
        p->yAxis->setRange(rLower - 0.1*diff, rUpper + 0.1*diff);
        p->replot();
    };

    updateAxis(plotFlow);
    updateAxis(plotError);
    updateAxis(plotControl);
    updateAxis(plotPID);
}

// --- Sterowanie ---

void MainWindow::on_pushButton_start_2_clicked()
{
    // Przed startem upewnij się, że parametry są aktualne
    syncPIDParams();
    syncGeneratorParams();
    onIntervalChanged();

    service->start();

    ui->pushButton_start_2->setEnabled(false);
    ui->pushButton_stop_2->setEnabled(true);

}

void MainWindow::on_pushButton_stop_2_clicked()
{
    service->stop();

    ui->pushButton_start_2->setEnabled(true);
    ui->pushButton_stop_2->setEnabled(false);

}

void MainWindow::on_pushButton_reset_ALL_clicked()
{
    service->reset();

    // Czyścimy wykresy
    if(gW) gW->data()->clear();
    if(gY) gY->data()->clear();
    if(gE) gE->data()->clear();
    if(gU) gU->data()->clear();
    if(gP) gP->data()->clear();
    if(gI) gI->data()->clear();
    if(gD) gD->data()->clear();

    // Odświeżamy puste
    if(plotFlow) plotFlow->replot();
    if(plotError) plotError->replot();
    if(plotControl) plotControl->replot();
    if(plotPID) plotPID->replot();

    ui->pushButton_start_2->setEnabled(true);
    ui->pushButton_stop_2->setEnabled(false);
}

void MainWindow::on_pushButton_resetPID_clicked()
{
    ui->kpSpinBox->setValue(1.0);
    ui->tiSpinBox->setValue(1.0);
    ui->tdSpinBox->setValue(0.0);
    syncPIDParams();
}

void MainWindow::on_pushButton_resetGenerator_clicked()
{
    ui->amplitudeSpinBox->setValue(1.0);
    ui->periodSpinBox->setValue(1.0);
    ui->offsetSpinBox->setValue(0.0);
    syncGeneratorParams();
}

// --- Parametry ---

void MainWindow::syncPIDParams()
{
    if(!service) return;

    // 1. Wysyłamy liczby (Kp, Ti, Td)
    service->setPIDParams(
        ui->kpSpinBox->value(),
        ui->tiSpinBox->value(),
        ui->tdSpinBox->value()
        );

    // 2. Ustalamy tryb całkowania (TO JEST KLUCZOWE DLA ZALICZENIA)
    PIDController::IntegralMode mode;

    // Najpierw sprawdzamy Anti-Windup (ma priorytet wg logiki UI)
    if (ui->antiWindupCheckBox->isChecked()) {
        mode = PIDController::IntegralMode::AntiWindup;
    }
    else {
        // --- TUTAJ BYŁ BRAK ---
        // Teraz sprawdzamy, co wybrał użytkownik w ComboBoxie

        if (ui->comboBox_integralMode->currentIndex() == 0) {
            // Index 0: Stała PRZED sumą (Standard / PostSum)
            mode = PIDController::IntegralMode::PostSum;
        } else {
            // Index 1: Stała POD sumą (Niezależna / PreSum)
            mode = PIDController::IntegralMode::PreSum;
        }
    }

    // 3. Wysyłamy ostateczny tryb do serwisu
    service->setPIDIntegralMode(mode);
}

void MainWindow::syncGeneratorParams()
{
    if(!service) return;

    // 1. Walidacja wstępna - "Bramkarz"
    // Jeśli kluczowe wartości są bez sensu (np. 0 lub ujemne),
    // to w ogóle nie wysyłamy ich do silnika, żeby nie spowodować crasha.

    double amplitude = ui->amplitudeSpinBox->value();
    double offset = ui->offsetSpinBox->value();
    double period = ui->periodSpinBox->value();
    double duty = ui->doubleSpinBox_infil->value();
    double interval = ui->doubleSpinBox_interval->value();

    // Zabezpieczenie: Okres musi być > 0
    if (period <= 0.0001) return;

    // Zabezpieczenie: Interwał musi być > 0
    if (interval <= 0.0001) return;

    // Zabezpieczenie: Wypełnienie musi być 0..1
    if (duty < 0.0) duty = 0.0;
    if (duty > 1.0) duty = 1.0;

    // 2. Bezpieczna wysyłka w bloku try-catch
    try {
        service->setGeneratorParams(
            amplitude,
            offset,
            period, // TRZ
            duty,   // Duty Cycle
            static_cast<int>(interval * 1000) // TT_ms
            );
    } catch (const std::exception& e) {
        // Jeśli cokolwiek pójdzie nie tak, wypisz błąd w konsoli zamiast zamykać program
        qWarning() << "Ignored invalid generator params:" << e.what();
    }
}
void MainWindow::onIntervalChanged()
{
    // Konwersja sekundy -> milisekundy
    double dt = ui->doubleSpinBox_interval->value();
    if(dt < 0.01) dt = 0.01;
    int ms = static_cast<int>(dt * 1000.0);
    service->setSimulationInterval(ms);
}

void MainWindow::onSignalTypeChanged(int index)
{
    if(!service) return;

    SignalGenerator::Type type = SignalGenerator::Type::Step;

    // Logika UI: włączanie/wyłączanie pola Infill (Wypełnienie)
    bool isRect = false;

    if(index == 0) {
        type = SignalGenerator::Type::Step; // Skok
        isRect = false;
    }
    else if(index == 1) {
        type = SignalGenerator::Type::Sine; // Sinus
        isRect = false;
    }
    else if(index == 2) {
        type = SignalGenerator::Type::Rect; // Prostokąt
        isRect = true;
    }

    service->setGeneratorType(type);

    // Obsługa pola Infill (Wypełnienie)
    ui->doubleSpinBox_infil->setEnabled(isRect);

    if (isRect) {
        // Jeśli użytkownik ma 0.0, ustawmy mu domyślnie 0.5
        if (ui->doubleSpinBox_infil->value() <= 0.01) {
            ui->doubleSpinBox_infil->setValue(0.5);
        }
    }

    // --- TO MUSISZ DODAĆ (WYJĘTE Z IF-a) ---
    // Wysyłamy parametry do serwisu ZAWSZE przy zmianie typu,
    // żeby mieć pewność, że silnik ma aktualne dane.
    syncGeneratorParams();
}

void MainWindow::onAntiWindupChanged(bool checked)
{
    if(!service) return;
    // Beta = 1.0 (standard back-calc) lub 0.0 (wyłączone)
    // Albo zmiana trybu PIDController::IntegralMode
    if(checked) {
        service->setPIDIntegralMode(PIDController::IntegralMode::AntiWindup);
        service->setAntiWindupBeta(1.0); // Domyślna beta
    } else {
        service->setPIDIntegralMode(PIDController::IntegralMode::PostSum);
    }
}

// --- Okna Dialogowe i JSON ---

void MainWindow::onARXButtonClicked()
{

    SecDialog dlg(this, service);
    dlg.exec();
}

void MainWindow::on_pushButton_save_clicked()
{
    QString fn = QFileDialog::getSaveFileName(this, "Zapisz", "", "JSON (*.json)");
    if(fn.isEmpty()) return;

    // Upewnij się, że stan w serwisie odpowiada GUI
    syncPIDParams();
    syncGeneratorParams();

    if(service->saveConfig(fn)) {
        QMessageBox::information(this, "Info", "Zapisano poprawnie.");
    } else {
        QMessageBox::warning(this, "Błąd", "Nie udało się zapisać.");
    }
}

void MainWindow::on_pushButton_load_clicked()
{
    QString fn = QFileDialog::getOpenFileName(this, "Wczytaj", "", "JSON (*.json)");
    if(fn.isEmpty()) return;

    if(service->loadConfig(fn)) {
        updateUiFromService();
        QMessageBox::information(this, "Info", "Wczytano poprawnie.");
    } else {
        QMessageBox::warning(this, "Błąd", "Nie udało się wczytać.");
    }
}

void MainWindow::updateUiFromService()
{
    // Pobieramy dane z serwisu i ustawiamy w GUI
    auto pid = service->getPID();
    if(pid) {
        ui->kpSpinBox->setValue(pid->getKp());
        ui->tiSpinBox->setValue(pid->getTi());
        ui->tdSpinBox->setValue(pid->getTd());

        bool isAntiWindup = (pid->getIntegralMode() == PIDController::IntegralMode::AntiWindup);
        ui->antiWindupCheckBox->setChecked(isAntiWindup);
    }

    auto gen = service->getGenerator();
    if(gen) {
        ui->amplitudeSpinBox->setValue(gen->getAmplitude());
        ui->offsetSpinBox->setValue(gen->getOffset());
        ui->periodSpinBox->setValue(gen->getTRZ());
        ui->doubleSpinBox_infil->setValue(gen->getDuty());

        int idx = 0;
        if(gen->getType() == SignalGenerator::Type::Sine) idx = 1;
        if(gen->getType() == SignalGenerator::Type::Rect) idx = 2;
        ui->signalTypeComboBox->setCurrentIndex(idx);
    }

    // Interwał
    ui->doubleSpinBox_interval->setValue(service->getSimulationInterval() / 1000.0);
}
