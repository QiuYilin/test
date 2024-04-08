#pragma once
#include <QTreeWidget>
#include <QLabel>
#include <QMouseEvent>

class MyTree:public QTreeWidget
{
    Q_OBJECT
public:
    MyTree(QWidget *parent = nullptr);

    void init();

    void mousePressEvent(QMouseEvent *event);

    QLabel*  m_label;

    QTreeWidgetItem*  m_selectItem;
};