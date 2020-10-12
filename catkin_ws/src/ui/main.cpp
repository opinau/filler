#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <RosThread.h>
#include <BusinessLogic.h>

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

    engine.load(url);

    engine.rootContext()->setContextProperty("ros", &ros);

    QObject* mainWindow = engine.rootObjects().at(0);
    //QObject::connect(&ros, SIGNAL(inkStatusChanged(QVariant)), mainWindow, SLOT(onInkStatusChanged(QVariant)));

    QObject::connect(&engine, &QQmlApplicationEngine::exit,
                     [&](int retCode){qDebug() << "quitting with" << retCode; app.exit(retCode);});

    BusinessLogic business(&app, &ros, (QQuickWindow*)mainWindow);

    engine.rootContext()->setContextProperty("business", &business);

    return app.exec();
}
