import QtQuick 2.0

Item {
    property double progress: 0
    height: 50
    Rectangle {
        height: parent.height
        width: height
        radius: width * 0.5
        color: "yellow"
    }
}
