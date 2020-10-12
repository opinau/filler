import QtQuick 2.12
import QtQuick.Window 2.12

import QtQml.Models 2.1

import hr.openbook.opinau 1.0


Window {
    id: mainWindow

    visible: true
    width: 950
    height: 650
    title: "OPINAU-FILLER"
    color: "black"

    property double bottleSize: 25
    property double conveyorCentreY: (conveyor.y + (conveyor.height / 2)) - bottleSize
    property double conveyorStartX: conveyor.x

    Repeater {
        model: business.bottleModel
        delegate:         Bottle {
            progressAlong: progress
        }
    }

    //    onClosing :{
    //        Qt.exit(0)
    //    }

    function onInkStatusChanged(labelPresent) {
        labelSensor.labelPresent = labelPresent
    }

    Text {
        id: name
        color: "white"
        //text: "Bottle x: " + testBottle.x + " y: " + testBottle.y + " progress: " + bottlePath.progress
    }

    LabelSensor {
        id: labelSensor

        labelPresent: business.labelPresent

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

        opacity: 0.5

        MouseArea {
            anchors.fill: parent
            onClicked:
            {
                console.log(business.bottleModel)
            }
        }
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
