QT       += core gui widgets printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

SOURCES += \
    ARXModel.cpp \
    PIDController.cpp \
    SignalGenerator.cpp \
    Simulation.cpp \
    UARService.cpp \
    main.cpp \
    mainwindow.cpp \
    secdialog.cpp \
    qcustomplot.cpp

HEADERS += \
    ARXModel.h \
    PIDController.h \
    SignalGenerator.h \
    Simulation.h \
    UARService.h \
    mainwindow.h \
    secdialog.h \
    qcustomplot.h

FORMS += \
    mainwindow.ui \
    secdialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
