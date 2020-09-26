#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <RosThread.h>

int main(int argc, char* argv[])
{
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

  QGuiApplication app(argc, argv);

  RosThread ros(argc, argv);
  ros.init();

  QQmlApplicationEngine engine;
  const QUrl url(QStringLiteral("qrc:/main.qml"));
  QObject::connect(
      &engine, &QQmlApplicationEngine::objectCreated, &app,
      [url](QObject* obj, const QUrl& objUrl) {
        if (!obj && url == objUrl)
          QCoreApplication::exit(-1);
      },
      Qt::QueuedConnection);

  engine.load(url);

  engine.rootContext()->setContextProperty("ros", &ros);

  QObject* mainWindow = engine.rootObjects().at(0);
  QObject::connect(&ros, SIGNAL(testSignal()), mainWindow, SLOT(testSlot()));

  return app.exec();
}
