import QtQuick 2.0

Item {
    property int speed: 50
    Rectangle {
        anchors.fill: parent
        clip: true
        Row {
            height: parent.height
            width: parent.width + 50

            NumberAnimation on x {
                running: true
                from: -50; to: 0
                loops: Animation.Infinite
                alwaysRunToEnd: false
                duration: 2000
            }

            Repeater {
                anchors.fill: parent
                model: parent.width / 50
                Rectangle {
                    height: parent.height
                    color: "white"
                    border.width: 1
                    width: 50
                }
            }
        }
    }
}
