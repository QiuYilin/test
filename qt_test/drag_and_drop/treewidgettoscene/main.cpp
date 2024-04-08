#include <QtWidgets/QApplication>
#include "MyScene.h"
#include "MyTree.h"
#include <QWidget>
#include <QHBoxLayout>
#include <QGraphicsView>
#include <QGraphicsScene>

int main(int argc, char *argv[]){
    QApplication app(argc,argv);
    QWidget mainWidget;
    QHBoxLayout *l = new QHBoxLayout(&mainWidget);

    MyTree * mytree = new MyTree(&mainWidget);
    mytree->setGeometry(0,0,180,500);
    mytree->init();

    MyScene* myscene = new MyScene;
    myscene->setSceneRect(QRect(0,0,300,500));


    QGraphicsView *view = new  QGraphicsView(&mainWidget);
    //view->setAcceptDrops(true);

    view ->setGeometry(200,0,300,500);
    view->setScene(myscene);

    l->addWidget(mytree);
    l->addWidget(view);

    mainWidget.showNormal();

    return app.exec();
}