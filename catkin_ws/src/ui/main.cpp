#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <RosThread.h>
#include <BusinessLogic.h>
#include <BottleModel.h>
int main(int argc, char* argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);

    RosThread ros(argc, argv);
    if (!ros.init()) {
        qDebug() << "roscore not running!";
    }

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(
                &engine, &QQmlApplicationEngine::objectCreated, &app,
                [url](QObject* obj, const QUrl& objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    },
    Qt::QueuedConnection);



    BusinessLogic business(&app, &ros, &engine);

    engine.rootContext()->setContextProperty("business", &business);
    engine.rootContext()->setContextProperty("ros", &ros);

    engine.load(url);

    return app.exec();
}
