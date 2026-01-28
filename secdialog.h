#ifndef SECDIALOG_H
#define SECDIALOG_H

#include <QDialog>
#include <memory>
#include <vector>

namespace Ui { class SecDialog; }

// Forward declaration
class UARService;

class SecDialog : public QDialog
{
    Q_OBJECT

public:
    // Konstruktor przyjmuje wskaźnik do serwisu, aby móc pobrać i zapisać dane
    explicit SecDialog(QWidget *parent, std::shared_ptr<UARService> service);
    ~SecDialog();

private slots:
    void on_pushButton_ARX_zatwiedz_clicked();
    void on_pushButton_ARX_anuluj_clicked();

private:
    Ui::SecDialog *ui;
    std::shared_ptr<UARService> service_;

    // Metoda sprawdzająca poprawność danych (np. czy min < max)
    bool validateValues();

    // Metody pomocnicze do ładowania/zapisywania danych
    void loadFromService();
    void saveToService();
};

#endif // SECDIALOG_H
