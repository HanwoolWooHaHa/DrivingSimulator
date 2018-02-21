#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class CThread;
class CLoopManager;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    CThread* m_pThread;
    CLoopManager* m_pLoopManager;

    int m_nDataMode;

private slots:
    void OpenFile();
    void OpenAllFile();
    void OpenThreeFiles();
    void ResetButtonEntered();
    void StartButtonEntered();
    void TrainingButtonEntered();
    void TestingButtonEntered();
    void SaveDataPerTrial();
};

#endif // MAINWINDOW_H
