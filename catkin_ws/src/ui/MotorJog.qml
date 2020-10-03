import QtQuick 2.0
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12

Item {
    property int index: 0

    function sendMotorCommand() {
        ros.messageLabellerMotor(index, enabledSwitch.checked, speedSlider.value)
    }

    height: 100

    ColumnLayout {
        Switch {
            id: enabledSwitch
            text: speedSlider.value
            onToggled: {
                sendMotorCommand();
            }
        }
        Slider {
            id: speedSlider
            from: -128
            to: 127
            stepSize: 1

            onValueChanged: {
                sendMotorCommand()
            }
        }
    }
}
