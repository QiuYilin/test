#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QTextStream>
#include <QDebug>

QString text;                               //用于存放日志信息

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //qInstallMessageHandler(logOutput);
}

MainWindow::~MainWindow()
{
    delete ui;
}
