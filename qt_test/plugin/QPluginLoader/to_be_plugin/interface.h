#pragma once
class PluginInterface{
public:
  virtual ~PluginInterface(){}
  virtual void SayHello(QWidget *parent) = 0;
};

#define pluginInterface_iid "io.qt.dynamicplugin"
Q_DECLARE_INTERFACE(PluginInterface, pluginInterface_iid)