import QtQuick 2.12
import QtQuick.Window 2.12

import QtQml.Models 2.1

//import hr.openbook.opinau 1.0


Window {
    id: mainWindow

    visible: true
    width: 950
    height: 650
    title: "OPINAU-FILLER"
    color: "black"

    property double bottleSize: 5
    property double conveyorCentreY: (conveyor.y + (conveyor.height / 2)) - bottleSize
    property double conveyorStartX: conveyor.x

    property double phase1Length: 50
    property double phase2Length: 50
    property real phase2Radius: 100
    property double phase3Length: 250
    property double phase4Length: 100
    property real phase4Radius: 200

    property double phase5Length: 200


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
        x: conveyorStartX + phase1Length + (phase2Length / 2) - (width / 2)
        y: conveyorCentreY - 52 - (height / 2)
        height: 150
        width: height
    }

    FillerStarwheel {
        x: conveyorStartX + phase1Length + phase2Length + phase3Length + (phase4Length / 2) - (width / 2)
        y: conveyorCentreY - 157 - (height / 2)
        height: 300
        width: height
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
