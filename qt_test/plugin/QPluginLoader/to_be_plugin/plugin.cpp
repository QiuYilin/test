#include "plugin.h"

Plugin::Plugin(QObject *parent):QObject(parent){
    qDebug() << "constructor";
}

Plugin::~Plugin(){
    qDebug() << "desturctor";
}

void Plugin::SayHello(QWidget *parent){
    QMessageBox::information(parent,"plugin","hello,this is dynamically loaded.");
}

