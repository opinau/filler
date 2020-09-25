import QtQuick 2.10
import QtGraphicalEffects 1.12

Item {
    Image {
        id: image
        source: "qrc:/assets/small_starwheel.png"
        anchors.fill: parent
        Colorize {
                anchors.fill: image
                source: image
                hue: 0.0
                saturation: 0.5
                lightness: -0.2
            }
        RotationAnimator {
                target: image
                from: 0
                to: 360
                duration: 1000
                running: true
            }
    }
}
