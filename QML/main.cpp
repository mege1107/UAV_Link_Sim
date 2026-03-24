#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDebug>
#include <QObject>
#include "backend.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    Backend backend;

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::warnings,
        [](const QList<QQmlError> &warnings) {
            for (const auto &w : warnings) {
                qWarning().noquote() << w.toString();
            }
        }
        );

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() {
            qWarning() << "QML object creation failed.";
            QCoreApplication::exit(-1);
        },
        Qt::QueuedConnection
        );

    engine.rootContext()->setContextProperty("backend", &backend);
    engine.loadFromModule("App", "Main");

    if (engine.rootObjects().isEmpty()) {
        qWarning() << "engine.rootObjects() is empty.";
        return -1;
    }

    return app.exec();
}
