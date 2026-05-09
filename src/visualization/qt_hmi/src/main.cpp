// qt_hmi/src/main.cpp
//
// Process entry point. Order of operations:
//   1. ros::init()              — must precede any ros::NodeHandle.
//   2. QApplication ctor        — must precede any QWidget ctor.
//   3. RosBridge ctor           — starts ros::AsyncSpinner internally.
//   4. MainWindow ctor          — wires bridge signals to widget slots.
//   5. Connect aboutToQuit      — stop spinner before destruction (race fix
//                                  per design §6.H).
//   6. app.exec()               — Qt event loop. Blocks until quit.
//   7. ros::shutdown()          — final cleanup.
#include <memory>

#include <QtCore/QCoreApplication>
#include <QtCore/QObject>
#include <QtWidgets/QApplication>

#include <ros/ros.h>

#include "qt_hmi/MainWindow.h"
#include "qt_hmi/RosBridge.h"

int main(int argc, char* argv[]) {
  // ros::init must be called before NodeHandle creation. argv parsing is
  // shared with QApplication, but ros::init only consumes ROS-specific
  // remappings (`name:=value`) and leaves Qt args alone.
  ros::init(argc, argv, "qt_hmi_node",
            ros::init_options::NoSigintHandler);

  QApplication app(argc, argv);
  QCoreApplication::setApplicationName(QStringLiteral("qt_hmi"));
  QCoreApplication::setOrganizationName(QStringLiteral("KATECH"));

  auto bridge = std::make_unique<RosBridge>();
  MainWindow w(bridge.get());
  w.show();

  // Race-safe shutdown: stop spinner BEFORE Qt destroys widgets / bridge.
  // Without this, an in-flight callback may emit into already-destructed
  // QObject instances → segfault. See design §6.H.
  QObject::connect(&app, &QCoreApplication::aboutToQuit,
                   [&bridge]() {
                     if (bridge) bridge->stop();
                   });

  const int rc = app.exec();
  bridge.reset();         // explicit destruct (spinner already stopped)
  ros::shutdown();
  return rc;
}
