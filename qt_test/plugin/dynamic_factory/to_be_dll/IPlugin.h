#ifndef SERVICEPROJECT_IPLUGIN_H
#define SERVICEPROJECT_IPLUGIN_H

#include <QString>
#include <QStringList>

class IPlugin
{
public:
  IPlugin() = default;
  virtual ~IPlugin() = default;

  IPlugin(const IPlugin&) = delete;
  IPlugin& operator=(const IPlugin&) = delete;

public:
   //获取支持的插件方法
  virtual const QStringList& getSupportCommandList() const {return supportCommandList_; }


public:
  virtual QString getVersionString() const = 0;
  //! \brief 获取业务名称
  //! \return
  virtual QString getPluginName() const = 0;

  enum pluginState_type
  {
    idle_,
    starting_,
    started_,
    stopping_,
    stopped_,
    excepting_,
  };
  //! 获取插件的运行状态。
  //! \return
  virtual pluginState_type getPluginState() const = 0;
  //! 获取插件的运行状态的解释
  //! \param state
  //! \return
  static QString getPluginStateMessage(const pluginState_type& state)
  {
    QString msg = "no msg.";
    switch (state)
    {
      case pluginState_type::idle_:
        msg = "plugin is on idle statement.";
        break;
      case pluginState_type::starting_:
        msg = "plugin is on starting statement.";
        break;
      case pluginState_type::started_:
        msg = "plugin is on started statement.";
        break;
      case pluginState_type::stopping_:
        msg = "plugin is on stopping statement.";
        break;
      case pluginState_type::stopped_:
        msg = "plugin is on stopped statement.";
        break;
      case pluginState_type::excepting_:
        msg = "plugin is on excepting statement.";
        break;
    }

    return msg;
  }

  //! \brief 执行业务的主要接口方法
  //! \param msg 由外部程序发送的通讯协议（比如json报文）
  //! \return 错误信息
  virtual std::error_code exec(const QString& msg) = 0;
  //! \brief 业务停止运行
  //! \return
  virtual std::error_code stop() { return {}; }
  //! \brief 释放动态库资源
  //! \return 错误信息
  virtual std::error_code release() = 0;

protected:
  QStringList supportCommandList_;
};

#endif  // SERVICEPROJECT_IPLUGIN_H