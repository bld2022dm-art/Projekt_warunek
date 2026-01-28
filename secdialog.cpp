#include "secdialog.h"
#include "ui_secdialog.h"
#include "UARService.h"
#include <QMessageBox>
#include <cmath>

SecDialog::SecDialog(QWidget *parent, std::shared_ptr<UARService> service)
    : QDialog(parent), ui(new Ui::SecDialog), service_(service)
{
    ui->setupUi(this);

    // Kosmetyka: usunięcie pytajnika z paska tytułu (Windows)
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    setWindowTitle("Konfiguracja Obiektu ARX");

    // Konfiguracja zakresów (zabezpieczenie UI)
    auto setupSpin = [](QDoubleSpinBox* sb) {
        sb->setRange(-1000.0, 1000.0);
        sb->setDecimals(3);
        sb->setSingleStep(0.1);
    };

    setupSpin(ui->doubleSpinBox_A1);
    setupSpin(ui->doubleSpinBox_A2);
    setupSpin(ui->doubleSpinBox_A3);

    setupSpin(ui->doubleSpinBox_B1);
    setupSpin(ui->doubleSpinBox_B2);
    setupSpin(ui->doubleSpinBox_B3);

    // Limity
    ui->doubleSpinBox_Umin->setRange(-200.0, 200.0);
    ui->doubleSpinBox_Umax->setRange(-200.0, 200.0);

    ui->doubleSpinBox_Ymin->setRange(-200.0, 200.0);
    ui->doubleSpinBox_Ymax->setRange(-200.0, 200.0);

    // Szum i opóźnienie
    ui->doubleSpinBox_sigma->setRange(0.0, 100.0);
    ui->spinBox_opznienie->setRange(1, 100); // k >= 1 (Wymóg PDF)


    connect(ui->pushButton_ARX_zatwiedz, &QPushButton::clicked, this, &SecDialog::on_pushButton_ARX_zatwiedz_clicked);
    connect(ui->pushButton_ARX_anuluj, &QPushButton::clicked, this, &SecDialog::on_pushButton_ARX_anuluj_clicked);

    // Wczytaj aktualne dane z serwisu do kontrolek
    loadFromService();
}

SecDialog::~SecDialog()
{
    delete ui;
}

void SecDialog::loadFromService()
{
    if (!service_) return;

    // 1. Pobierz parametry modelu
    std::vector<double> A, B;
    int k;
    service_->getARXVectors(A, B, k);

    // Bezpieczne mapowanie wektorów na 3 pola (nawet jak wektor jest krótszy)
    auto safeGet = [](const std::vector<double>& v, size_t idx) {
        return (idx < v.size()) ? v[idx] : 0.0;
    };

    ui->doubleSpinBox_A1->setValue(safeGet(A, 0));
    ui->doubleSpinBox_A2->setValue(safeGet(A, 1));
    ui->doubleSpinBox_A3->setValue(safeGet(A, 2));

    ui->doubleSpinBox_B1->setValue(safeGet(B, 0));
    ui->doubleSpinBox_B2->setValue(safeGet(B, 1));
    ui->doubleSpinBox_B3->setValue(safeGet(B, 2));

    ui->spinBox_opznienie->setValue(k);
    ui->doubleSpinBox_sigma->setValue(service_->getSigma());

    // 2. Pobierz limity
    double uMin, uMax, yMin, yMax;
    bool uEn, yEn;
    service_->getARXLimits(uMin, uMax, uEn, yMin, yMax, yEn);

    ui->checkBox_limitU->setChecked(uEn);
    ui->doubleSpinBox_Umin->setValue(uMin);
    ui->doubleSpinBox_Umax->setValue(uMax);

    ui->checkBox_limitY->setChecked(yEn);
    ui->doubleSpinBox_Ymin->setValue(yMin);
    ui->doubleSpinBox_Ymax->setValue(yMax);
}

bool SecDialog::validateValues()
{
    // Sprawdzenie logiczne limitów
    if (ui->checkBox_limitU->isChecked()) {
        if (ui->doubleSpinBox_Umin->value() >= ui->doubleSpinBox_Umax->value()) {
            QMessageBox::warning(this, "Błąd", "Limit U_min musi być mniejszy od U_max!");
            return false;
        }
    }

    if (ui->checkBox_limitY->isChecked()) {
        if (ui->doubleSpinBox_Ymin->value() >= ui->doubleSpinBox_Ymax->value()) {
            QMessageBox::warning(this, "Błąd", "Limit Y_min musi być mniejszy od Y_max!");
            return false;
        }
    }

    return true;
}

void SecDialog::saveToService()
{
    if (!service_) return;

    // Tworzenie wektorów z pól GUI
    std::vector<double> A = {
        ui->doubleSpinBox_A1->value(),
        ui->doubleSpinBox_A2->value(),
        ui->doubleSpinBox_A3->value()
    };

    std::vector<double> B = {
        ui->doubleSpinBox_B1->value(),
        ui->doubleSpinBox_B2->value(),
        ui->doubleSpinBox_B3->value()
    };

    // Ustawienie modelu
    service_->setARXParams(
        A,
        B,
        ui->spinBox_opznienie->value(),
        ui->doubleSpinBox_sigma->value()
        );

    // Ustawienie limitów
    service_->setARXLimits(
        ui->doubleSpinBox_Umin->value(),
        ui->doubleSpinBox_Umax->value(),
        ui->checkBox_limitU->isChecked(),
        ui->doubleSpinBox_Ymin->value(),
        ui->doubleSpinBox_Ymax->value(),
        ui->checkBox_limitY->isChecked()
        );
}

void SecDialog::on_pushButton_ARX_zatwiedz_clicked()
{
    if (validateValues()) {
        saveToService();
        accept(); // Zamyka okno z wynikiem QDialog::Accepted
    }
}

void SecDialog::on_pushButton_ARX_anuluj_clicked()
{
    reject(); // Zamyka okno bez zapisywania
}
