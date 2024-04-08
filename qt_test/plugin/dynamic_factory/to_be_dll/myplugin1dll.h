//myplugin1dll.h
extern "C" __declspec(dllexport) void *getInstance();

//myplugin1dll.cpp
void* getInstance()
{
  static MyPlugin1 w;
  return (void*)&w;
}

//MyPlugin1.h
class MyPlugin1: public IPlugin
{
public:
  MyPlugin1();
  ~MyPlugin1() override;

public:
  QString getVersionString() const override;
  //! \brief 获取业务名称
  //! \return
  QString getPluginName() const override;
  //! 获取插件的运行状态。
  //! \return
  pluginState_type getPluginState() const override;

  //! \brief 执行业务
  //! \param msg 由外部程序发送的通讯协议
  //! \return 错误信息
  std::error_code exec(const QString &msg) override;
  //! \brief 业务停止运行
  //! \return
  std::error_code stop() override;
  //! \brief 释放动态库资源
  //! \return 错误信息
  std::error_code release() override;
};