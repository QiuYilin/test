#include "mainwindow.h"

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QTextStream>
#include <QDebug>

Q_LOGGING_CATEGORY(output, "test") // in .cpp



void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
 {
    // 加锁
        static QMutex mutex;
        mutex.lock();

        QByteArray localMsg = msg.toLocal8Bit();

        QString strMsg("");
        switch(type)
        {
        case QtDebugMsg:
            strMsg = QString("----Debug----");
            break;
        case QtInfoMsg:
            strMsg = QString("----Infos----");
            break;
        case QtWarningMsg:
            strMsg = QString("---Warning---");
            break;
        case QtCriticalMsg:
            strMsg = QString("---Critical---");
            break;
        case QtFatalMsg:
            strMsg = QString("----Fatal----");
            break;
        }

        // 设置输出信息格式
        QString strDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
        QString strMessage = QString("[%1] %5 %6\t-------------    ")
                 .arg(strDateTime).arg(strMsg).arg(localMsg.constData());



        //获取可执行程序所在目录
        QString localApplicationDirPath = QCoreApplication::applicationDirPath();
        QString log_file;
        QString log_dir=localApplicationDirPath+"/log";
        QDir dir(log_dir);
        //cmd中创建文件需要使用\，使用/报错
        //log_dir.replace("/","\\");
 
        if(!dir.exists())
        {
            //创建在可执行程序所在目录创建log文件
            system(QString(" mkdir %1").arg(log_dir).toStdString().data());
        }

        QDateTime current_date_time =QDateTime::currentDateTime();
        QString date = current_date_time.toString("yyyy-MM-dd");
        //在c++中\表示转义，\\才为字符"\"
        //log_file=log_dir+"\\"+"log_"+date+".txt";
        log_file=log_dir+"/"+"log_"+date+".txt";

        // 输出信息至文件中（读写、追加形式）
        QFile file(log_file);
        file.open(QIODevice::ReadWrite | QIODevice::Append);
        QTextStream stream(&file);
        stream << strMessage << "\r\n";
        file.flush();
        file.close();

        // 解锁
        mutex.unlock();
 }

int main(int argc, char *argv[])
{

    qInstallMessageHandler(myMessageOutput);

    QApplication a(argc, argv);
    MainWindow w;
    qCDebug(output)<<"This is a debug message.";
    qCInfo(output) <<"This is a info message.";
    qCWarning(output )<< "This is a warning message.";
    qCCritical(output)<<"This is a critical message.";
    w.show();
    return a.exec();
}
