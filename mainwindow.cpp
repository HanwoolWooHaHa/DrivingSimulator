#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "Include/database.h"
#include "Include/myThread.h"
#include "Include/loopManager.h"
#include "Include/trainer.h"

#include <QDebug>
#include <QFileDialog>
#include <QGridLayout>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_nDataMode = DRIVING_SIMULATOR_DATA;

    m_pThread = new CThread();
    m_pLoopManager = new CLoopManager();

    m_pLoopManager->moveToThread(m_pThread);
    connect(ui->btnStart, SIGNAL(clicked()), m_pLoopManager, SLOT(DoWork()));

    m_pThread->start();

    QGridLayout *layout = new QGridLayout;
    layout->addWidget(m_pLoopManager->pWindow, 0, 0);
    ui->widget->setLayout(layout);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete m_pThread;
    delete m_pLoopManager;
}

void MainWindow::OpenFile()
{
    QString filePath;
    QString fileName;

    filePath = DS_FILE_PATH;
    fileName = QFileDialog::getOpenFileName(this, tr("driving simulator file"), filePath, tr("Data file (*.csv);;All Files (*)"));

    int nReturn = CDatabase::GetInstance()->LoadData(m_nDataMode, fileName);

    if (nReturn == DONE)
    {
        ui->btnStart->setEnabled(true);
        ui->btnReset->setEnabled(true);
        ui->btnTraining->setEnabled(true);
        ui->btnTesting->setEnabled(true);

        ui->lblFileName->setText(fileName);

        int nTrial = CDatabase::GetInstance()->GetNumTrial();

        ui->lblNumTrial->setText(QString::number(nTrial));
    }

    m_pLoopManager->Initialize();
}

void MainWindow::ResetButtonEntered()
{
    m_pLoopManager->ResetTime();

    m_pLoopManager->SetLoopFlag(false);
    ui->btnStart->setText("Start");
}

void MainWindow::StartButtonEntered()
{
    bool bFlag = m_pLoopManager->GetLoopFlag();

    if (bFlag)
    {
        m_pLoopManager->SetLoopFlag(false);
        ui->btnStart->setText("Start");
    }
    else
    {
        m_pLoopManager->SetLoopFlag(true);
        ui->btnStart->setText("Pause");
    }
}

void MainWindow::TrainingButtonEntered()
{
    CTrainer::GetInstance()->Train();

    ui->btnTraining->setEnabled(false);
    ui->btnTraining->setText("Done");

    ui->btnTesting->setEnabled(false);
    ui->btnTesting->setText("Done");
}

void MainWindow::TestingButtonEntered()
{
    CTrainer::GetInstance()->LoadModel();

    ui->btnTraining->setEnabled(false);
    ui->btnTraining->setText("Done");

    ui->btnTesting->setEnabled(false);
    ui->btnTesting->setText("Done");
}

void MainWindow::SaveDataPerTrial()
{
    int nRet = CDatabase::GetInstance()->SaveDataPerTrial();

    if(nRet == FAIL)
        qDebug() << "Saving file per trial was failed...";
    else
        qDebug() << "Saving file was completed successfully";
}
