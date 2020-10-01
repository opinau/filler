import QtQuick 2.0

Item {
    property bool labelPresent: false
    Rectangle {
        anchors.fill: parent
        color: parent.labelPresent ? "red" : "green"
    }

}
