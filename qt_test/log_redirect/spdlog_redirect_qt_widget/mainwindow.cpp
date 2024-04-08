#include "mainwindow.h"
#include "widget.h"
#include "spdlog/sinks/qt_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include <QLayout>
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{   
    setMinimumSize(640, 480);
    auto log_widget = new QTextEdit(this);
    setCentralWidget(log_widget);
    //int max_lines = 500; // keep the text widget to max 500 lines. remove old lines if needed.
    //auto logger = spdlog::qt_color_logger_mt("qt_logger", log_widget, max_lines);//>1.11.0
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>("logs/mylogfile", 1048576 * 5, 3));
    sinks.push_back(std::make_shared<spdlog::sinks::qt_sink_mt>(log_widget,"append"));
    _logger =std::make_shared<spdlog::logger>("combine", begin(sinks), end(sinks));
    //_logger->sinks()[0]->set_pattern("..");
    _logger->sinks()[0]->set_level(spdlog::level::trace);
    _logger->sinks()[1]->set_pattern(">>>>>>>>> %H:%M:%S %z %v <<<<<<<<<");
    _logger->sinks()[1]->should_log(spdlog::level::debug);
    spdlog::register_logger(_logger);

   _widget = new Widget(this);
   //this->layout()->addWidget(_widget);
   //this->layout()->addWidget(log_widget);
   
    _logger->warn("warn info");
    _logger->info("info info");
    _logger->trace("trace info");
    _logger->debug("debug info");
    _logger->error("error info");
    _logger->critical("critical info");
    _logger->flush();
}
