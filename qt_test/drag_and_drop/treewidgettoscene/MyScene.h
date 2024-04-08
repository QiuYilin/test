#ifndef MYSCENE_H
#define MYSCENE_H

#include <QGraphicsScene>
#include <QGraphicsSceneEvent>
#include <QGraphicsSceneDragDropEvent>
#include <QMimeData>
#include <QGraphicsRectItem>
#include <QObject>
#include <QDrag>
class MyScene:public QGraphicsScene
{
    Q_OBJECT
public:
    MyScene(QObject *parent = nullptr);

protected:

    //drag操作在窗体移动事件
    void dragMoveEvent(QGraphicsSceneDragDropEvent *event) override;

    //drag操作进入窗体
    void dragEnterEvent(QGraphicsSceneDragDropEvent* event) ;

    //drop释放事件
    void dropEvent(QGraphicsSceneDragDropEvent* event) ;


};

#endif // MYSCENE_H