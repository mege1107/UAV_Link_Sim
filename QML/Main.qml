import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtCharts
import QtQuick.Dialogs

ApplicationWindow {
    id: window
    visible: true
    width: 1600
    height: 920
    minimumWidth: 1300
    minimumHeight: 760
    title: "无人机通信链路仿真系统(地面站)"
    color: "#f3f3f3"

    ButtonGroup {
        id: transferGroup
    }

    FileDialog {
        id: inputFileDialog
        title: "选择待发送文件"

        onAccepted: {
            backend.inputFilePath = selectedFile.toString()
            backend.sendFile()
        }
    }

    Component {
        id: constellationViewComponent

        Rectangle {
            color: "#fcfcfc"
            border.color: "#c8c8c8"
            border.width: 1

            Canvas {
                id: constellationCanvas
                anchors.fill: parent
                renderStrategy: Canvas.Threaded

                onPaint: {
                    var ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)

                    var xs = backend.constellationI
                    var ys = backend.constellationQ
                    var N = Math.min(xs.length, ys.length)

                    var left = 55
                    var right = width - 20
                    var top = 20
                    var bottom = height - 45
                    var plotW = right - left
                    var plotH = bottom - top

                    ctx.strokeStyle = "#d0d0d0"
                    ctx.lineWidth = 1
                    ctx.strokeRect(left, top, plotW, plotH)

                    if (N < 1) {
                        ctx.fillStyle = "#777777"
                        ctx.font = "18px sans-serif"
                        ctx.fillText("暂无星座图数据", width / 2 - 60, height / 2)
                        return
                    }

                    var xmin = xs[0], xmax = xs[0]
                    var ymin = ys[0], ymax = ys[0]

                    for (var i = 1; i < N; ++i) {
                        if (xs[i] < xmin) xmin = xs[i]
                        if (xs[i] > xmax) xmax = xs[i]
                        if (ys[i] < ymin) ymin = ys[i]
                        if (ys[i] > ymax) ymax = ys[i]
                    }

                    var maxAbs = Math.max(Math.abs(xmin), Math.abs(xmax), Math.abs(ymin), Math.abs(ymax))
                    if (maxAbs < 1e-6)
                        maxAbs = 1.0

                    var axisMin = -1.2 * maxAbs
                    var axisMax = 1.2 * maxAbs
                    var axisRange = axisMax - axisMin

                    function mapX(x) {
                        return left + (x - axisMin) / axisRange * plotW
                    }

                    function mapY(y) {
                        return bottom - (y - axisMin) / axisRange * plotH
                    }

                    ctx.strokeStyle = "#e0e0e0"
                    ctx.lineWidth = 1
                    for (var k = 0; k <= 4; ++k) {
                        var gx = left + k / 4 * plotW
                        var gy = top + k / 4 * plotH

                        ctx.beginPath()
                        ctx.moveTo(gx, top)
                        ctx.lineTo(gx, bottom)
                        ctx.stroke()

                        ctx.beginPath()
                        ctx.moveTo(left, gy)
                        ctx.lineTo(right, gy)
                        ctx.stroke()
                    }

                    var x0 = mapX(0)
                    var y0 = mapY(0)

                    ctx.strokeStyle = "#808080"
                    ctx.lineWidth = 1.2

                    ctx.beginPath()
                    ctx.moveTo(left, y0)
                    ctx.lineTo(right, y0)
                    ctx.stroke()

                    ctx.beginPath()
                    ctx.moveTo(x0, top)
                    ctx.lineTo(x0, bottom)
                    ctx.stroke()

                    ctx.fillStyle = "#2c7be5"
                    for (var p = 0; p < N; ++p) {
                        var px = mapX(xs[p])
                        var py = mapY(ys[p])

                        ctx.beginPath()
                        ctx.arc(px, py, 2.5, 0, 2 * Math.PI)
                        ctx.fill()
                    }

                    ctx.fillStyle = "#666666"
                    ctx.font = "12px sans-serif"
                    ctx.fillText("In-Phase", width / 2 - 22, height - 10)

                    ctx.save()
                    ctx.translate(18, height / 2 + 20)
                    ctx.rotate(-Math.PI / 2)
                    ctx.fillText("Quadrature", 0, 0)
                    ctx.restore()

                    ctx.font = "11px sans-serif"
                    for (var t = 0; t <= 4; ++t) {
                        var xv = axisMin + t / 4 * axisRange
                        var xp = left + t / 4 * plotW
                        ctx.fillText(xv.toFixed(2), xp - 12, bottom + 16)

                        var yv = axisMin + (4 - t) / 4 * axisRange
                        var yp = top + t / 4 * plotH
                        ctx.fillText(yv.toFixed(2), 5, yp + 4)
                    }
                }

                Connections {
                    target: backend
                    function onConstellationChanged() {
                        constellationCanvas.requestPaint()
                    }
                }
            }
        }
    }

    Component {
        id: controlAnalysisComponent

        ColumnLayout {
            anchors.fill: parent
            spacing: 0

            TabBar {
                id: analysisTab
                Layout.fillWidth: true
                currentIndex: 0

                TabButton { text: "时频分析" }
                TabButton { text: "波形图" }
                TabButton { text: "频谱图" }
                TabButton { text: "星座图" }
            }

            StackLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: analysisTab.currentIndex

                Rectangle {
                    color: "white"
                    border.color: "#c8c8c8"
                    border.width: 1

                    Canvas {
                        id: spectrogramCanvas
                        anchors.fill: parent
                        renderStrategy: Canvas.Threaded

                        function colorFromValue(v) {
                            var x = Math.max(0, Math.min(1, v))
                            var r = 0
                            var g = 0
                            var b = 0

                            if (x < 0.25) {
                                r = 0
                                g = Math.floor(4 * x * 255)
                                b = 255
                            } else if (x < 0.5) {
                                r = 0
                                g = 255
                                b = Math.floor((1 - 4 * (x - 0.25)) * 255)
                            } else if (x < 0.75) {
                                r = Math.floor(4 * (x - 0.5) * 255)
                                g = 255
                                b = 0
                            } else {
                                r = 255
                                g = Math.floor((1 - 4 * (x - 0.75)) * 255)
                                b = 0
                            }

                            return "rgb(" + r + "," + g + "," + b + ")"
                        }

                        onPaint: {
                            var ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)

                            var data = backend.spectrogramData
                            var specW = backend.spectrogramWidth
                            var specH = backend.spectrogramHeight

                            var left = 60
                            var right = width - 25
                            var top = 20
                            var bottom = height - 45
                            var plotW = right - left
                            var plotH = bottom - top

                            ctx.strokeStyle = "#b0b0b0"
                            ctx.lineWidth = 1
                            ctx.strokeRect(left, top, plotW, plotH)

                            if (specW > 0 && specH > 0 && data.length === specW * specH) {
                                var cellW = plotW / specW
                                var cellH = plotH / specH

                                for (var tx = 0; tx < specW; ++tx) {
                                    for (var fy = 0; fy < specH; ++fy) {
                                        var idx = tx * specH + fy
                                        var v = data[idx]
                                        var drawY = top + (specH - 1 - fy) * cellH

                                        ctx.fillStyle = colorFromValue(v)
                                        ctx.fillRect(left + tx * cellW, drawY, Math.ceil(cellW), Math.ceil(cellH))
                                    }
                                }
                            }

                            ctx.strokeStyle = "#909090"
                            ctx.lineWidth = 1
                            ctx.beginPath()
                            ctx.moveTo(left, bottom)
                            ctx.lineTo(right, bottom)
                            ctx.moveTo(left, bottom)
                            ctx.lineTo(left, top)
                            ctx.stroke()

                            ctx.fillStyle = "#666666"
                            ctx.font = "12px sans-serif"
                            ctx.fillText("Time (s)", width / 2 - 20, height - 10)

                            ctx.save()
                            ctx.translate(18, height / 2 + 25)
                            ctx.rotate(-Math.PI / 2)
                            ctx.fillText("Frequency (MHz)", 0, 0)
                            ctx.restore()

                            var tSpan = backend.spectrogramTimeSpan
                            var fMin = backend.spectrogramFreqMin
                            var fMax = backend.spectrogramFreqMax

                            for (var i = 0; i <= 4; ++i) {
                                var x = left + i / 4 * plotW
                                ctx.strokeStyle = "#d8d8d8"
                                ctx.beginPath()
                                ctx.moveTo(x, top)
                                ctx.lineTo(x, bottom)
                                ctx.stroke()

                                var tLabel = tSpan * i / 4
                                ctx.fillStyle = "#666666"
                                ctx.fillText(tLabel.toFixed(4), x - 12, bottom + 16)
                            }

                            for (var j = 0; j <= 4; ++j) {
                                var y = bottom - j / 4 * plotH
                                ctx.strokeStyle = "#d8d8d8"
                                ctx.beginPath()
                                ctx.moveTo(left, y)
                                ctx.lineTo(right, y)
                                ctx.stroke()

                                var fLabelHz = fMin + (fMax - fMin) * j / 4
                                var fLabelMHz = fLabelHz / 1e6
                                ctx.fillStyle = "#666666"
                                ctx.fillText(fLabelMHz.toFixed(2), 6, y + 4)
                            }
                        }

                        Connections {
                            target: backend
                            function onSpectrogramChanged() {
                                spectrogramCanvas.requestPaint()
                            }
                        }
                    }
                }

                Rectangle {
                    color: "#fcfcfc"
                    border.color: "#c8c8c8"
                    border.width: 1

                    Canvas {
                        id: waveformCanvas
                        anchors.fill: parent
                        renderStrategy: Canvas.Threaded

                        onPaint: {
                            var ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)

                            var data = backend.waveform
                            var N = data.length
                            if (N < 2)
                                return

                            var left = 20
                            var right = width - 20
                            var top = 15
                            var bottom = height - 15
                            var plotW = right - left
                            var plotH = bottom - top

                            var minv = backend.waveformMin
                            var maxv = backend.waveformMax
                            var range = maxv - minv
                            if (range <= 0)
                                range = 1

                            ctx.strokeStyle = "#d0d0d0"
                            ctx.lineWidth = 1
                            ctx.strokeRect(left, top, plotW, plotH)

                            if (minv <= 0 && maxv >= 0) {
                                var zeroY = bottom - (0 - minv) / range * plotH
                                ctx.strokeStyle = "#cccccc"
                                ctx.beginPath()
                                ctx.moveTo(left, zeroY)
                                ctx.lineTo(right, zeroY)
                                ctx.stroke()
                            }

                            ctx.strokeStyle = "#2c7be5"
                            ctx.lineWidth = 1
                            ctx.beginPath()

                            for (var i2 = 0; i2 < N; ++i2) {
                                var x2 = left + i2 / (N - 1) * plotW
                                var y2 = bottom - (data[i2] - minv) / range * plotH

                                if (i2 === 0)
                                    ctx.moveTo(x2, y2)
                                else
                                    ctx.lineTo(x2, y2)
                            }

                            ctx.stroke()
                        }

                        Connections {
                            target: backend
                            function onWaveformChanged() {
                                waveformCanvas.requestPaint()
                            }
                        }
                    }
                }

                Rectangle {
                    color: "#fcfcfc"
                    border.color: "#c8c8c8"
                    border.width: 1

                    Canvas {
                        id: spectrumCanvas
                        anchors.fill: parent

                        onPaint: {
                            var ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)

                            var freq = backend.spectrumFreq
                            var mag = backend.spectrumMag
                            var N = Math.min(freq.length, mag.length)

                            if (N < 2)
                                return

                            var left = 50
                            var right = width - 20
                            var top = 20
                            var bottom = height - 40
                            var plotW = right - left
                            var plotH = bottom - top

                            var xmin = freq[0]
                            var xmax = freq[N - 1]
                            var ymin = mag[0]
                            var ymax = mag[0]

                            for (var i3 = 1; i3 < N; ++i3) {
                                if (mag[i3] < ymin) ymin = mag[i3]
                                if (mag[i3] > ymax) ymax = mag[i3]
                            }

                            var xRange = xmax - xmin
                            var yRange = ymax - ymin
                            if (xRange <= 0) xRange = 1
                            if (yRange <= 0) yRange = 1

                            ctx.strokeStyle = "#d0d0d0"
                            ctx.lineWidth = 1
                            ctx.strokeRect(left, top, plotW, plotH)

                            ctx.fillStyle = "#666666"
                            ctx.font = "12px sans-serif"
                            ctx.fillText("Frequency (Hz)", width / 2 - 40, height - 10)
                            ctx.save()
                            ctx.translate(15, height / 2 + 30)
                            ctx.rotate(-Math.PI / 2)
                            ctx.fillText("Magnitude (dB)", 0, 0)
                            ctx.restore()

                            ctx.strokeStyle = "#2c7be5"
                            ctx.lineWidth = 1
                            ctx.beginPath()

                            for (var i4 = 0; i4 < N; ++i4) {
                                var x4 = left + (freq[i4] - xmin) / xRange * plotW
                                var y4 = bottom - (mag[i4] - ymin) / yRange * plotH

                                if (i4 === 0)
                                    ctx.moveTo(x4, y4)
                                else
                                    ctx.lineTo(x4, y4)
                            }

                            ctx.stroke()

                            ctx.fillStyle = "#666666"
                            ctx.font = "11px sans-serif"

                            for (var k = 0; k <= 4; ++k) {
                                var tx = left + k / 4 * plotW

                                ctx.strokeStyle = "#cccccc"
                                ctx.beginPath()
                                ctx.moveTo(tx, top)
                                ctx.lineTo(tx, bottom)
                                ctx.stroke()

                                var fLabel = xmin + k / 4 * xRange
                                ctx.fillText(fLabel.toFixed(0), tx - 12, bottom + 15)
                            }

                            for (var k2 = 0; k2 <= 4; ++k2) {
                                var ty = bottom - k2 / 4 * plotH

                                ctx.strokeStyle = "#cccccc"
                                ctx.beginPath()
                                ctx.moveTo(left, ty)
                                ctx.lineTo(right, ty)
                                ctx.stroke()

                                var mLabel = ymin + k2 / 4 * yRange
                                ctx.fillText(mLabel.toFixed(1), 5, ty + 4)
                            }
                        }

                        Connections {
                            target: backend
                            function onSpectrumChanged() {
                                spectrumCanvas.requestPaint()
                            }
                        }
                    }
                }

                Loader {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    sourceComponent: constellationViewComponent
                }
            }
        }
    }

    Component {
        id: telemetryAnalysisComponent

        ColumnLayout {
            anchors.fill: parent
            spacing: 0

            TabBar {
                id: telemetryAnalysisTab
                Layout.fillWidth: true
                currentIndex: 0

                TabButton { text: "接收端波形图" }
                TabButton { text: "接收端频谱图" }
                TabButton { text: "星座图" }
            }

            StackLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: telemetryAnalysisTab.currentIndex

                Rectangle {
                    color: "#fcfcfc"
                    border.color: "#c8c8c8"
                    border.width: 1

                    Canvas {
                        id: rxWaveformCanvas
                        anchors.fill: parent
                        renderStrategy: Canvas.Threaded

                        onPaint: {
                            var ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)

                            var data = backend.rxWaveform
                            var N = data.length
                            if (N < 2)
                                return

                            var left = 45
                            var right = width - 20
                            var top = 20
                            var bottom = height - 40
                            var plotW = right - left
                            var plotH = bottom - top

                            var minv = backend.rxWaveformMin
                            var maxv = backend.rxWaveformMax
                            var range = maxv - minv
                            if (range <= 0)
                                range = 1

                            ctx.strokeStyle = "#d0d0d0"
                            ctx.lineWidth = 1
                            ctx.strokeRect(left, top, plotW, plotH)

                            if (minv <= 0 && maxv >= 0) {
                                var zeroY = bottom - (0 - minv) / range * plotH
                                ctx.strokeStyle = "#cccccc"
                                ctx.beginPath()
                                ctx.moveTo(left, zeroY)
                                ctx.lineTo(right, zeroY)
                                ctx.stroke()
                            }

                            ctx.fillStyle = "#666666"
                            ctx.font = "12px sans-serif"
                            ctx.fillText("Amplitude", 6, 18)
                            ctx.fillText("Sample Index", width / 2 - 30, height - 10)

                            ctx.strokeStyle = "#2c7be5"
                            ctx.lineWidth = 1
                            ctx.beginPath()

                            for (var i5 = 0; i5 < N; ++i5) {
                                var x5 = left + i5 / (N - 1) * plotW
                                var y5 = bottom - (data[i5] - minv) / range * plotH

                                if (i5 === 0)
                                    ctx.moveTo(x5, y5)
                                else
                                    ctx.lineTo(x5, y5)
                            }

                            ctx.stroke()
                        }

                        Connections {
                            target: backend
                            function onRxWaveformChanged() {
                                rxWaveformCanvas.requestPaint()
                            }
                        }
                    }
                }

                Rectangle {
                    color: "#fcfcfc"
                    border.color: "#c8c8c8"
                    border.width: 1

                    Canvas {
                        id: rxSpectrumCanvas
                        anchors.fill: parent

                        onPaint: {
                            var ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)

                            var freq = backend.rxSpectrumFreq
                            var mag = backend.rxSpectrumMag
                            var N = Math.min(freq.length, mag.length)

                            if (N < 2)
                                return

                            var left = 50
                            var right = width - 20
                            var top = 20
                            var bottom = height - 40
                            var plotW = right - left
                            var plotH = bottom - top

                            var xmin = freq[0]
                            var xmax = freq[N - 1]
                            var ymin = mag[0]
                            var ymax = mag[0]

                            for (var i6 = 1; i6 < N; ++i6) {
                                if (mag[i6] < ymin) ymin = mag[i6]
                                if (mag[i6] > ymax) ymax = mag[i6]
                            }

                            var xRange = xmax - xmin
                            var yRange = ymax - ymin
                            if (xRange <= 0) xRange = 1
                            if (yRange <= 0) yRange = 1

                            ctx.strokeStyle = "#d0d0d0"
                            ctx.lineWidth = 1
                            ctx.strokeRect(left, top, plotW, plotH)

                            ctx.fillStyle = "#666666"
                            ctx.font = "12px sans-serif"
                            ctx.fillText("Frequency (Hz)", width / 2 - 40, height - 10)
                            ctx.save()
                            ctx.translate(15, height / 2 + 30)
                            ctx.rotate(-Math.PI / 2)
                            ctx.fillText("Magnitude (dB)", 0, 0)
                            ctx.restore()

                            ctx.strokeStyle = "#2c7be5"
                            ctx.lineWidth = 1
                            ctx.beginPath()

                            for (var i7 = 0; i7 < N; ++i7) {
                                var x7 = left + (freq[i7] - xmin) / xRange * plotW
                                var y7 = bottom - (mag[i7] - ymin) / yRange * plotH

                                if (i7 === 0)
                                    ctx.moveTo(x7, y7)
                                else
                                    ctx.lineTo(x7, y7)
                            }

                            ctx.stroke()
                        }

                        Connections {
                            target: backend
                            function onRxSpectrumChanged() {
                                rxSpectrumCanvas.requestPaint()
                            }
                        }
                    }
                }

                Loader {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    sourceComponent: constellationViewComponent
                }
            }
        }
    }

    component SectionBox : Rectangle {
        property alias title: titleText.text
        property alias contentItem: contentLoader.sourceComponent

        color: "white"
        border.color: "#b8b8b8"
        border.width: 1
        radius: 2

        ColumnLayout {
            anchors.fill: parent
            spacing: 0

            Rectangle {
                Layout.fillWidth: true
                height: 34
                color: "#f7f7f7"
                border.color: "#b8b8b8"
                border.width: 1

                Text {
                    id: titleText
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.left: parent.left
                    anchors.leftMargin: 10
                    font.pixelSize: 16
                    font.bold: true
                    color: "#444444"
                }
            }

            Loader {
                id: contentLoader
                Layout.fillWidth: true
                Layout.fillHeight: true
            }
        }
    }

    component StatusLamp : Rectangle {
        property color lampColor: "red"
        width: 22
        height: 22
        radius: 11
        color: lampColor
        border.color: "#a0a0a0"
        border.width: 1
    }

    component FrameBlock : Rectangle {
        property string label: ""
        property color blockColor: "#cccccc"

        width: 150
        height: 58
        radius: 2
        color: blockColor
        border.color: "#999999"
        border.width: 1

        Text {
            anchors.centerIn: parent
            text: parent.label
            color: "white"
            font.pixelSize: 16
            font.bold: true
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        Text {
            Layout.alignment: Qt.AlignHCenter
            text: "无人机通信链路仿真系统(地面站)"
            font.pixelSize: 34
            font.bold: true
            color: "black"
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10

            ColumnLayout {
                Layout.fillHeight: true
                Layout.preferredWidth: 470
                Layout.minimumWidth: 420
                Layout.maximumWidth: 520
                Layout.fillWidth: false
                spacing: 10

                TabBar {
                    id: modeTab
                    Layout.fillWidth: true

                    TabButton { text: "遥控" }
                    TabButton { text: "遥测" }
                    TabButton { text: "图传" }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 165
                    title: "参数设置"

                    contentItem: Component {
                        GridLayout {
                            anchors.fill: parent
                            anchors.margins: 14
                            columns: 4
                            rowSpacing: 12
                            columnSpacing: 10

                            Label { text: "调制方式"; font.pixelSize: 15 }
                            ComboBox {
                                Layout.preferredWidth: 130
                                model: ["MSK", "BPSK", "QPSK", "QAM", "OOK", "FSK", "FM"]
                                currentIndex: Math.max(0, model.indexOf(backend.selectedMode))
                                onActivated: backend.selectedMode = currentText
                            }

                            Label { text: "中心频率"; font.pixelSize: 15 }
                            RowLayout {
                                TextField {
                                    Layout.preferredWidth: 90
                                    text: backend.centerFreq
                                    onTextChanged: backend.centerFreq = text
                                }
                                Label { text: "MHz"; font.pixelSize: 15 }
                            }

                            Label { text: "跳频图案"; font.pixelSize: 15 }
                            ComboBox {
                                Layout.preferredWidth: 130
                                model: ["1", "2", "3", "4"]
                                currentIndex: Math.max(0, model.indexOf(backend.hopPattern))
                                onActivated: backend.hopPattern = currentText
                            }

                            Label { text: "信息速率"; font.pixelSize: 15 }
                            RowLayout {
                                ComboBox {
                                    Layout.preferredWidth: 110
                                    model: ["4", "8", "16", "32", "64", "128", "256", "512", "1024"]
                                    currentIndex: Math.max(0, model.indexOf(backend.infoRate))
                                    onActivated: backend.infoRate = currentText
                                }
                                Label { text: "Kbps"; font.pixelSize: 15 }
                            }

                            Item { Layout.columnSpan: 3 }
                            Button {
                                text: "应用参数"
                                Layout.preferredWidth: 110
                                onClicked: backend.applyParameterSettings()
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 95
                    title: "硬件(USRP B210)连接"

                    contentItem: Component {
                        RowLayout {
                            anchors.fill: parent
                            anchors.margins: 14
                            spacing: 16

                            Label { text: "连接"; font.pixelSize: 15 }

                            StatusLamp {
                                lampColor: backend.usrpConnected ? "#18c25c" : "red"
                            }

                            Label {
                                text: backend.usrpConnected ? "打开" : "关闭"
                                font.pixelSize: 15
                                color: backend.usrpConnected ? "#18a14c" : "#666666"
                            }

                            Switch {
                                checked: backend.usrpConnected
                                onToggled: backend.connectUsrp(checked)
                            }

                            Item { Layout.fillWidth: true }

                            Button {
                                text: "运行联调"
                                onClicked: backend.runSimulation()
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 165
                    title: "信息传输"

                    contentItem: Component {
                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 14
                            spacing: 12

                            RowLayout {
                                spacing: 12

                                RadioButton {
                                    id: textMode
                                    checked: false
                                    text: "字符"
                                    ButtonGroup.group: transferGroup
                                    onCheckedChanged: {
                                        if (checked) backend.setTransferSource("TEXT")
                                    }
                                }

                                TextField {
                                    Layout.fillWidth: true
                                    placeholderText: "请输入待发送文本"
                                    text: backend.inputText
                                    onTextChanged: backend.inputText = text
                                    enabled: textMode.checked
                                }

                                Button {
                                    text: "输入"
                                    enabled: textMode.checked
                                    onClicked: backend.sendText()
                                }
                            }

                            RowLayout {
                                spacing: 12

                                RadioButton {
                                    id: fileMode
                                    text: "文件"
                                    ButtonGroup.group: transferGroup
                                    onCheckedChanged: {
                                        if (checked) backend.setTransferSource("FILE")
                                    }
                                }

                                TextField {
                                    id: filePathField
                                    Layout.fillWidth: true
                                    placeholderText: "请输入或选择待发送文件"
                                    text: backend.inputFilePath
                                    readOnly: false
                                    enabled: fileMode.checked
                                    onEditingFinished: backend.inputFilePath = text
                                }

                                Button {
                                    text: "读取"
                                    enabled: fileMode.checked
                                    onClicked: inputFileDialog.open()
                                }
                            }

                            Label {
                                text: textMode.checked
                                      ? (backend.inputText === "" ? "等待输入文本" : "文本已就绪")
                                      : (backend.inputFilePath === "" ? "等待选择文件" : "文件已就绪")
                                color: "#ff5a5a"
                                font.pixelSize: 14
                                font.bold: true
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 105
                    title: "遥控信号发送"

                    contentItem: Component {
                        RowLayout {
                            anchors.fill: parent
                            anchors.margins: 14
                            spacing: 16

                            Label { text: "发送"; font.pixelSize: 15 }

                            StatusLamp {
                                lampColor: backend.sending ? "#18c25c" : "red"
                            }

                            Label {
                                text: backend.sending ? "打开" : "关闭"
                                font.pixelSize: 15
                                color: backend.sending ? "#18a14c" : "#666666"
                            }

                            Switch {
                                checked: backend.sending
                                onToggled: backend.startSending(checked)
                            }

                            Button {
                                text: "发送文件"
                                enabled: fileMode.checked && backend.inputFilePath !== ""
                                onClicked: {
                                    backend.sendFile()
                                    backend.startSending(true)
                                }
                            }

                            Item { Layout.fillWidth: true }

                            Label {
                                text: backend.sending ? "发送中" : "等待操作"
                                color: "#ff5a5a"
                                font.pixelSize: 17
                                font.bold: true
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 85
                    title: "结果"

                    contentItem: Component {
                        RowLayout {
                            anchors.fill: parent
                            anchors.margins: 14
                            spacing: 16

                            Text {
                                text: "BER:"
                                font.pixelSize: 22
                                font.bold: true
                            }

                            Text {
                                text: backend.berText
                                font.pixelSize: 22
                                color: "#d14b4b"
                                font.bold: true
                            }

                            Item { Layout.fillWidth: true }

                            Button {
                                text: "运行一次"
                                onClicked: backend.runSimulation()
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.minimumHeight: 160
                    title: "运行日志"

                    contentItem: Component {
                        ScrollView {
                            anchors.fill: parent
                            anchors.margins: 8
                            clip: true

                            TextArea {
                                id: logArea
                                width: parent.width
                                readOnly: true
                                wrapMode: TextArea.Wrap
                                text: backend.logText
                                font.pixelSize: 14

                                onTextChanged: {
                                    cursorPosition = length
                                }
                            }
                        }
                    }
                }
            }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumWidth: 700
                spacing: 10

                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 180
                    Layout.maximumHeight: 200
                    spacing: 10

                    SectionBox {
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        Layout.minimumWidth: 220
                        Layout.preferredWidth: 300
                        title: "系统示意"

                        contentItem: Component {
                            Rectangle {
                                anchors.fill: parent
                                anchors.margins: 10
                                color: "white"
                                border.color: "#b8b8b8"
                                border.width: 1
                                clip: true

                                Image {
                                    id: systemImage
                                    anchors.fill: parent
                                    source: "qrc:/qt/qml/App/images/groundstation_ui.png"
                                    fillMode: Image.PreserveAspectFit
                                    smooth: true
                                    cache: true
                                }

                                Text {
                                    anchors.centerIn: parent
                                    visible: systemImage.status === Image.Error
                                    text: "系统示意图加载失败"
                                    color: "#aa4444"
                                    font.pixelSize: 20
                                }
                            }
                        }
                    }

                    SectionBox {
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        Layout.minimumWidth: 480
                        Layout.preferredWidth: 700
                        title: "帧结构"

                        contentItem: Component {
                            ColumnLayout {
                                anchors.fill: parent
                                anchors.margins: 10
                                spacing: 8

                                Item { Layout.fillHeight: true }

                                RowLayout {
                                    Layout.alignment: Qt.AlignHCenter
                                    spacing: 6

                                    FrameBlock {
                                        label: "同步头"
                                        blockColor: "#ea812d"
                                        width: 130
                                        height: 50
                                    }

                                    FrameBlock {
                                        label: "数据"
                                        blockColor: "#446fc9"
                                        width: 300
                                        height: 50
                                    }

                                    FrameBlock {
                                        label: "补零"
                                        blockColor: "#79c341"
                                        width: 120
                                        height: 50
                                    }
                                }

                                RowLayout {
                                    Layout.alignment: Qt.AlignHCenter
                                    spacing: 90
                                    Label { text: "338samples"; font.pixelSize: 20 }
                                    Label { text: "99200samples"; font.pixelSize: 20 }
                                    Label { text: "264samples"; font.pixelSize: 20 }
                                }

                                Item { Layout.fillHeight: true }
                            }
                        }
                    }
                }

                SectionBox {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    title: modeTab.currentIndex === 1 ? "遥测信号分析" : "信号分析"

                    contentItem: modeTab.currentIndex === 1 ? telemetryAnalysisComponent : controlAnalysisComponent
                }
            }
        }
    }
}