#include "widget.h"
#include "spdlog/spdlog.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
{
    spdlog::get("combine")->info("generate widget");
    //spdlog::get("combine")->info("widget_qt");
}

Widget::~Widget()
{
}


