import QtQuick 2.0
import QtQuick.Layouts 1.12

Item {
    RowLayout {
        id: layout
        anchors.fill: parent
        spacing: 250

        MotorJog {
            index: 0
        }

        MotorJog {
            index: 1
        }

        MotorJog {
            index: 2
        }

        InkControl {

        }
    }
}
