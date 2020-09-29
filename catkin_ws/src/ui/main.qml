import QtQuick 2.12
import QtQuick.Window 2.12

Window {
    id: mainWindow

    visible: true
    width: 640
    height: 640
    title: qsTr("Hello World")
    color: "green"

    function testSlot() {
        console.log("in qml slot")
    }

    Text {
        id: name
        color: "white"
        text: "Bottle x: " + testBottle.x + " y: " + testBottle.y
    }

    Conveyor {
        x: 50
        y: 100
        height: 90
        width: 500

        MouseArea {
            anchors.fill: parent
            onClicked:
            {
                console.log("hello")
                bottlePath.start()
            }
        }
    }

    PathAnimation {
        id: bottlePath
        duration: 1000
        target: testBottle
        path: Path {
            startX: 0; startY: 300
            PathLine { x: 50; y: 300}
            PathArc { x: 100; y: 300; radiusX: 25; radiusY: 25}
            PathLine { x: 150; y: 300}
            PathArc { x: 200; y: 300; radiusX: 25; radiusY: 25}
            PathLine { x: 250; y: 300}
        }
    }

    Bottle {
        id: testBottle
        x: 0
        y: 300
    }

    LabellerStarwheel {
        x: 100
        y: 100
        height: 150
        width: 150
    }

    FillerStarwheel {
        x: 300
        y: 100
        height: 300
        width: 300
    }

    JogConsole {
        x: 0
        y: 500
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
