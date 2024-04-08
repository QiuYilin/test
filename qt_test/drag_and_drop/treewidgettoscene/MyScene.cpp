#include "MyScene.h"
#include <QDebug>
MyScene::MyScene(QObject *parent):QGraphicsScene(parent)
{

}

void MyScene::dragMoveEvent(QGraphicsSceneDragDropEvent *event)
{
    //坑1 并不是在DragEnterEvent 开启接受权限 而是在这

    event->accept();
    //event->setAccepted(true);
    //不能调用父类的，会把权限关闭
  //  QGraphicsScene::dragMoveEvent(event);
}


void MyScene::dragEnterEvent(QGraphicsSceneDragDropEvent *event)
{
    event->accept();
    //event->setAccepted(true);
    QGraphicsScene::dragEnterEvent(event);
}

void MyScene::dropEvent(QGraphicsSceneDragDropEvent *event)
{
    if (event->mimeData()->hasFormat("Data/name"))
    {
        //获取拖拽的数据
        QByteArray itemData = event->mimeData()->data("Data/name");
        QDataStream dataStream(&itemData, QIODevice::ReadOnly);
        QString text;
        dataStream >>text;
        qDebug()<<text;

        //创建一个item
        QGraphicsRectItem  * item = new QGraphicsRectItem();
        //坑2 event->pos() 失效了 坐标一直是（0，0），直接用场景坐标
        item->setRect(event->scenePos().x(),event->scenePos().y(),100,50);
        this->addItem(item);


    }
    QGraphicsScene::dropEvent(event);


}