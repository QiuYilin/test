do_pluginWork(const QString& msg, const QString& cmd)
{
  auto allPluginNames = PluginManager::getInstance()->getNames();
  for (const auto& name : allPluginNames)
  {
    auto pluginPtr = PluginManager::getInstance()->get(name);
    if (pluginPtr)
    {
      if (pluginPtr->getSupportCommandList().contains(cmd))//用于管理插件本身支持的函数
      {
        auto errorCode = pluginPtr->exec(msg);
        if (errorCode.value())
        {
          LOGGING_ERROR("[%s] result message: %s", pluginPtr->getPluginName().toStdString().c_str(),
                        errorCode.message().c_str());
        }
        LOGGING_DEBUG("%s is executed.", cmd.toStdString().c_str());
        return;
      }
    }
  }

  LOGGING_ERROR("cannot found match command.");
}