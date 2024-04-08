#include "MainWindow.h"

#include <QTranslator>
#include <QApplication>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    auto t = new QTranslator(qApp);
    t->load(":/test_zh_CN.qm");
    qApp->installTranslator(t);

    setWindowTitle(tr("Test Application"));
}

MainWindow::~MainWindow() {
}
