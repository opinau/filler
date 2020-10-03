import QtQuick 2.0

Item {
    property bool labelPresent: true
    Rectangle {
        anchors.fill: parent
        color: parent.labelPresent ? "red" : "green"
    }

}
