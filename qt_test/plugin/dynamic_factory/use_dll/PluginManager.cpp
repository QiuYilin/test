#include "PluginManager.h"

//......
typedef void *(*instance)();

PluginManager::PluginManager(QObject *parent) : QObject(parent)
{
}
PluginManager::~PluginManager()
{
  unloadAll();
}
void PluginManager::loadAll()
{
  QDir pluginDir(QCoreApplication::applicationDirPath() + R"(/../lib)");
  QStringList filters;
  filters << QString("*%1").arg(LIB_SUFFIX);
  pluginDir.setNameFilters(filters);

  auto entryInfoList = pluginDir.entryInfoList();
  for (const auto &info : entryInfoList)
  {
    auto lib = new QLibrary(QCoreApplication::applicationDirPath() + R"(/../lib/)" + info.fileName());
    if (lib->isLoaded())
    {
      LOGGING_WARN("%s is loaded.", info.fileName().toStdString().c_str());
      continue;
    }

    if (lib->load())
    {
      auto func = (instance)lib->resolve("getInstance");
      if (func)
      {
        auto plugin = (IPlugin *)func();
        if (plugin)
        {
          auto pluginName = plugin->getPluginName();
          if (plugins_.contains(pluginName))
          {
            LOGGING_WARN("%s repeated loading.", pluginName.toStdString().c_str());
            lib->unload();
            lib->deleteLater();
            lib = nullptr;
            continue;
          }

          plugins_.insert(pluginName, plugin);
          libs_.insert(pluginName, lib);

          LOGGING_DEBUG("%s version: %s", plugin->getPluginName().toStdString().c_str(),
                        plugin->getVersionString().toStdString().c_str());
        }
        else
        {
          LOGGING_ERROR("%s object create failed.", info.fileName().toStdString().c_str());
          lib->unload();
          lib->deleteLater();
          lib = nullptr;
        }
      }
      else
      {
        LOGGING_ERROR("%s cannot find symbol.", info.fileName().toStdString().c_str());
        lib->unload();
        lib->deleteLater();
        lib = nullptr;
      }
    }
    else
    {
      LOGGING_ERROR("%s load failed. error message: %s", info.fileName().toStdString().c_str(),
                    lib->errorString().toStdString().c_str());
      lib->deleteLater();
      lib = nullptr;
    }
  }
}
void PluginManager::unloadAll()
{
  for (auto &item : plugins_)
  {
    item->release();
  }

  for (auto &item : libs_)
  {
    item->unload();
    item->deleteLater();
    item = nullptr;
  }

  plugins_.clear();
  libs_.clear();
}
......
IPlugin *PluginManager::get(const QString &name)
{
  if (plugins_.contains(name))
  {
    return plugins_.value(name);
  }

  LOGGING_ERROR("%s is not found.", name.toStdString().c_str());
  return nullptr;
}
PluginManager *PluginManager::getInstance()
{
  static PluginManager w;
  return &w;
}
const char *PluginManager::getLibraryName()
{
  return PROJECT_NAME;
}
const char *PluginManager::getLibraryVersion()
{
  return PROJECT_VERSION;
}
}