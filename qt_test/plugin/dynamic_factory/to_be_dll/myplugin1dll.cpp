//MyPlugin1.cpp
#define MY_PLUGIN_NAME "myplugin1"
MyPlugin1::MyPlugin1()
{
    //关键，这里追定了本插件的插件名，插件管理类对象PluginManager会根据这个找到自己
    supportCommandList_.push_back(MY_PLUGIN_NAME);
}
MyPlugin1::~MyPlugin1()
{
}

QString MyPlugin1::getVersionString() const
{
  return PROJECT_VERSION;
}
QString MyPlugin1::getPluginName() const
{
  return PROJECT_NAME;
}
IPlugin::pluginState_type MyPlugin1::getPluginState() const
{
  return IPlugin::starting_;
}
std::error_code MyPlugin1::exec(const QString& msge)
{
   //业务功能实现......
}