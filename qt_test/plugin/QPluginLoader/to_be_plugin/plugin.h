#pragma once 
class Plugin : public QObject, public PluginInterface{//多重继承于QObject和抽象类
  Q_OBJECT
  Q_PLUGIN_METADATA(IID pluginInterface_iid FILE "plugin.json")//声明元数据，IID唯一 FILE可选 用于描述插件的相关数据信息
  Q_INTERFACES(PluginInterface) //将实现的插件接口通知给元类型系统
public:
  explicit Plugin(QObject *parent = 0);
  ~plugin();
  void SayHello(QWidget* parent) Q_DECL_OVERRIDE;//表示虚函数覆盖
}