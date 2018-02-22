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
    this->setFixedSize(1240, 680);
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
    }

    m_pLoopManager->Initialize();
}

void MainWindow::OpenAllFile()
{
    ui->btnStart->setEnabled(true);
    ui->btnReset->setEnabled(true);
    ui->btnTesting->setEnabled(true);
    ui->lblFileName->setText("auto simulation was selected");

    m_pLoopManager->SetDataMode(DRIVING_SIMULATOR_ALLDATA);

    m_pLoopManager->Initialize();
}

void MainWindow::OpenThreeFiles()
{
    ui->btnTraining->setEnabled(true);


    QString filePath;

    filePath = DS_FILE_PATH;
    m_nDataMode = DRIVING_SIMULATOR_THREE;

    int nReturn = CDatabase::GetInstance()->LoadData(m_nDataMode, NULL);

    if(nReturn)
        m_pLoopManager->Initialize();
    else
        qDebug() << "MainWindow @ Problem occured during loading";
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

void MainWindow::ChkBoxGroundClicked()
{
    bool bFlag = ui->chb_groundtruth->isChecked();
    m_pLoopManager->DrawTrajectory( GROUND_TRUTH, bFlag );
}

void MainWindow::ChkBoxTargetClicked()
{
    bool bFlag = ui->chb_target->isChecked();
    m_pLoopManager->DrawTrajectory( TARGET_TRAJECTORY, bFlag );
}

void MainWindow::ChkBoxPrecedingClicked()
{
    bool bFlag = ui->chb_preceding->isChecked();
    m_pLoopManager->DrawTrajectory( PRECEDING_TRAJECTORY, bFlag );
}
void MainWindow::ChkBoxLeadClicked()
{
    bool bFlag = ui->chb_lead->isChecked();
    m_pLoopManager->DrawTrajectory( LEAD_TRAJECTORY, bFlag );
}
