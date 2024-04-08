#include "interface.h"
int main{
  PluginInterface* interface = NULL;

  QPluginLoader plugin_loader("plugin");

  QObject* plugin = plugin_loader.instance();

  if(plugin){
    interface = qobject_cast<PluginInterface*>(plugin);
    if(interface){
        interface->SayHello(this);
    }
    plugin_loader.unload();
  }
  
  return 0;
}