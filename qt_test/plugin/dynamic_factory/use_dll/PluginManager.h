#include <QHash>
#include <QObject>

#if defined(PLUGIN_MANAGER_BUILD_SHARED)
#define PLUGIN_MANAGER_EXPORT __declspec(dllexport)
#else
#define PLUGIN_MANAGER_EXPORT __declspec(dllimport)
#endif

class IPlugin;
class QLibrary;

class PLUGIN_MANAGER_EXPORT PluginManager : public QObject{
  Q_OBJECT
private:
  explicit PluginManager(QObject *parent = nullptr);

public:
  static PluginManager* getInstance();
  ~PluginManager() override;

  static const char* getLibraryName();
  static const char* getLibraryVersion();

public:
  void loadAll();
  void unloadAll();
  std::error_code load(const QString& name);
  std::error_code unload(const QString& name);

  QStringList getNames() const {return plugins_.keys();}
  IPlugin* get(const QString& name);

private:
  QHash<QString,IPlugin*> plugins_;
  QHash<QString,QLibrary*> libs_;

}