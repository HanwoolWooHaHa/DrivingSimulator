#-------------------------------------------------
#
# Project created by QtCreator 2017-01-03T03:35:20
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DrivingSimulator
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    Src/birdView.cpp \
    Src/database.cpp \
    Src/loopManager.cpp \
    Src/trainer.cpp \
    Library/libsvm-3.20/svm.cpp \
    Src/leastsquare.cpp \
    Library/libpf/bessel.cpp \
    Library/libpf/potential.cpp \
    Library/libpf/vonMises.cpp \
    Src/bayesian.cpp \
    Src/majoritycount.cpp \
    Src/predictor.cpp \
    Src/evaluation.cpp \
    Src/estimator.cpp \
    Src/Navigation/sinusoidal.cpp \
    Src/mySVM.cpp \
    Src/calcIntgrt.cpp \
    Src/calcNormalCum.cpp \
    Src/extractor.cpp \
    Src/graph.cpp \
    Src/helper.cpp

HEADERS  += mainwindow.h \
    Include/birdView.h \
    Include/database.h \
    Include/defines.h \
    Include/loopManager.h \
    Include/myThread.h \
    Include/trainer.h \
    Library/libsvm-3.20/svm.h \
    Include/leastsquare.h \
    Library/libpf/bessel.h \
    Library/libpf/potential.h \
    Library/libpf/vonMises.h \
    Include/bayesian.h \
    Include/majoritycount.h \
    Include/predictor.h \
    Include/Navigation/navigationMethod.h \
    Include/evaluation.h \
    Include/estimator.h \
    Include/method.h \
    Include/Navigation/sinusoidal.h \
    Include/mySVM.h \
    Include/calcIntgrt.h \
    Include/calcNormalCum.h \
    Include/extractor.h \
    Include/graph.h \
    Include/helper.h

FORMS    += mainwindow.ui
LIBS += -lGLU
