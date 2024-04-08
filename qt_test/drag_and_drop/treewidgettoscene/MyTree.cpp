#include "MyTree.h"
#include <QMimeData>
#include <QWidget>
#include <QDrag>

MyTree::MyTree(QWidget *parent):QTreeWidget(parent)
{

}

void MyTree::init()
{
    //设置树形头部控件
    setHeaderLabel("节点拖动Demo");

    //创建树形
    //父节点1
    QTreeWidgetItem* pTree1 = new QTreeWidgetItem(this);
    pTree1->setText(0,QStringLiteral("父节点1"));

    //子节点
    QTreeWidgetItem* sTree1_1 = new QTreeWidgetItem(pTree1);
    sTree1_1->setText(0,QStringLiteral("子节点1"));

    QTreeWidgetItem* sTree1_2 = new QTreeWidgetItem(pTree1);
    sTree1_2->setText(0,QStringLiteral("子节点2"));
    pTree1->setExpanded(true);

    m_label = new QLabel(this);
    m_label->resize(100,50);
    m_label->hide();
}

void MyTree::mousePressEvent(QMouseEvent *event)
{
    m_selectItem = static_cast<QTreeWidgetItem*>(this->itemAt(event->pos()));
    if (event->button()==Qt::LeftButton&&m_selectItem){
        QByteArray dataItem;
        QDataStream dataStream(&dataItem, QIODevice::WriteOnly);
        dataStream << m_selectItem->text(0);
        m_label->setText(m_selectItem->text(0));

        //设置自定义数据
        QMimeData* mimeData = new QMimeData;
        mimeData->setData("Data/name", dataItem);

        //拖动设置
        QDrag* drag = new QDrag(this);
        drag->setPixmap(m_label->grab());
        drag->setMimeData(mimeData);
        drag->setHotSpot(QPoint(0,0));
        drag->exec();

//       Qt::DropAction dropAction = drag->exec(Qt::MoveAction|Qt::CopyAction,Qt::CopyAction);
//        if(dropAction == Qt::MoveAction)
//        {
//            //当成功拖动时清除原节点
//            delete m_selectItem;
//            m_selectItem = NULL;
//            qDebug()<<"成功";
//        }
    }
    QTreeWidget::mousePressEvent(event);
}