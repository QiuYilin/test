#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTextEdit>

#include "spdlog/spdlog.h"

class Widget;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() = default;
 
private:
    std::shared_ptr<spdlog::logger> _logger;
    Widget *_widget;

};

#endif // MAINWINDOW_H