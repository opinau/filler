import QtQuick 2.0
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12

Item {
    function sendInkCommand() {
        ros.messageInk(enabledSwitch.checked, bbdText.text)
    }

    height: 100

    ColumnLayout {
        Switch {
            id: enabledSwitch
            onToggled: {
                sendInkCommand();
            }
        }

        TextField {
            id: bbdText
            onTextChanged: {
                sendInkCommand()
            }
        }
    }
}
