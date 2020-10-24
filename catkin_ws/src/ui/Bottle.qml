import QtQuick 2.0

Item {
    id: root
    property alias progressAlong: interpolator.progress
    property alias path: interpolator.path

    x: interpolator.x
    y: interpolator.y

    height: 10

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
            id: bottlesPath
            startX: conveyorStartX; startY: conveyorCentreY
            PathLine { relativeX: phase1Length; y: conveyorCentreY }
            PathArc { id: phase2Arc; relativeX: phase2Length; y: conveyorCentreY; radiusX: phase2Radius; radiusY: phase2Radius; useLargeArc: true }
            PathLine { relativeX: phase3Length; y: conveyorCentreY }
            PathArc { relativeX: phase4Length; y: conveyorCentreY; radiusX: phase4Radius; radiusY: phase4Radius; useLargeArc: true }
            PathLine { relativeX: phase5Length; y: conveyorCentreY }
        }
    }
}
