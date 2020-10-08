import QtQuick 2.0
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12

Item {
    property int index: 0

    function sendRelayCommand() {
        ros.messageLabellerRelay(index, enabledSwitch.checked)
    }

    height: 100

    ColumnLayout {
        Switch {
            id: enabledSwitch
            onToggled: {
                sendRelayCommand();
            }
        }
    }
}
