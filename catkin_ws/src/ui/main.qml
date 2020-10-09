import QtQuick 2.12
import QtQuick.Window 2.12

Window {
    id: mainWindow

    visible: true
    width: 950
    height: 650
    title: "OPINAU-FILLER"
    color: "black"

//    onClosing :{
//        Qt.exit(0)
//    }

    //objectName: "ros"

    function onInkStatusChanged(labelPresent) {
        labelSensor.labelPresent = labelPresent
    }

    //    Connections {
    //        target: ros
    //        onInkStatusChanged: {
    //            labelSensor.labelPresent = labelPresent
    //        }
    //    }

    Text {
        id: name
        color: "white"
        text: "Bottle x: " + testBottle.x + " y: " + testBottle.y + " progress: " + bottlePath.progress
    }

    LabelSensor {
        id: labelSensor
        height: 20
        width: 20
    }

    Conveyor {
        id: conveyor
        speed: 0
        x: 50
        y: 300
        height: 90
        width: 700

        MouseArea {
            anchors.fill: parent
            onClicked:
            {
                bottlePathAnimation.start()
            }
        }
    }

    NumberAnimation {
        id: bottlePathAnimation
        target: bottlePath
        property: "progress"
        from: 0
        to: 1
        duration: 10000
    }

    PathInterpolator {
        id: bottlePath
        path: Path {
            startX: 0; startY: testBottle.conveyorCentreY
            PathLine { x: 50; y: testBottle.conveyorCentreY }
            PathArc { x: 100; y: testBottle.conveyorCentreY; radiusX: 25; radiusY: 25 }
            PathLine { x: 150; y: testBottle.conveyorCentreY }
            PathArc { x: 200; y: testBottle.conveyorCentreY; radiusX: 25; radiusY: 25 }
            PathLine { x: 600; y: testBottle.conveyorCentreY }
        }
    }

    Bottle {
        property int conveyorCentreY: (conveyor.y + (conveyor.height / 2)) - height / 2
        id: testBottle

        x: bottlePath.x
        y: bottlePath.y
    }

    LabellerStarwheel {
        x: 160
        y: 160
        height: 150
        width: 150
    }

    FillerStarwheel {
        x: 406
        y: 54
        height: 300
        width: 300
    }

    JogConsole {
        x: 0
        y: 400
        height: 300
        width: 300
    }

    Shortcut {
        sequence: "1"
        onActivated: {
            ros.test()
        }
    }
}
