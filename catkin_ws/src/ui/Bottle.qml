import QtQuick 2.0

Item {
    id: root
    property alias progressAlong: interpolator.progress

    x: interpolator.x
    y: interpolator.y

    height: 50

    z: 1

    Rectangle {
        height: parent.height
        width: height
        radius: width * 0.5
        color: "yellow"

    }

    PathInterpolator {
        id: interpolator
        path: Path {
            startX: conveyorStartX; startY: conveyorCentreY
            PathLine { x: conveyorStartX + 50; y: conveyorCentreY }
            PathArc { x: conveyorStartX + 100; y: conveyorCentreY; radiusX: 25; radiusY: 25 }
            PathLine { x: conveyorStartX + 150; y: conveyorCentreY }
            PathArc { x: conveyorStartX + 200; y: conveyorCentreY; radiusX: 25; radiusY: 25 }
            PathLine { x: conveyorStartX + 600; y: conveyorCentreY }
        }
    }
}
