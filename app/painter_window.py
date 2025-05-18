from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSlider, QFileDialog
)
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor
import cv2


class PainterWindow(QMainWindow):
    def __init__(self, detector):
        super().__init__()
        self.detector = detector

        # Initializing the colors
        self.colors = [
            QColor(255, 0, 0),  # Red
            QColor(0, 255, 0),  # Green
            QColor(0, 0, 255),  # Blue
            QColor(255, 255, 0),  # Yellow
            QColor(255, 0, 255),  # Magenta
            QColor(0, 255, 255),  # Cyan
        ]

        self.color = self.colors[0]  # Default to first color
        self.brush_size = 15
        self.last_point = None
        self.canvas_history = []

        self.camera_label = QLabel()
        self.canvas = QPixmap(800, 600)
        self.canvas_label = QLabel()

        # Setting up the UI
        self.initUI()
        self.initCamera()

    def initUI(self):
        self.setWindowTitle("AI Painter")
        self.setGeometry(100, 100, 1000, 800)

        # Main layout
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout()
        central.setLayout(layout)

        # Camera view

        self.camera_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.camera_label)

        # Canvas

        self.canvas.fill(Qt.white)

        self.canvas_label.setPixmap(self.canvas)
        self.canvas_label.setFixedSize(800, 600)
        layout.addWidget(self.canvas_label)

        # Toolbar
        toolbar = QHBoxLayout()

        # Clear button
        self.clear_btn = QPushButton("Clear Canvas")
        self.clear_btn.clicked.connect(self.clearCanvas)
        toolbar.addWidget(self.clear_btn)

        # Color buttons
        for color in self.colors:
            btn = QPushButton()
            btn.setFixedSize(32, 32)
            btn.setStyleSheet(f"background-color: {color.name()}")
            btn.clicked.connect(lambda _, c=color: self.setColor(c))
            toolbar.addWidget(btn)

        # Brush size slider
        toolbar.addWidget(QLabel("Brush:"))
        self.brush_slider = QSlider(Qt.Horizontal)
        self.brush_slider.setRange(5, 50)
        self.brush_slider.setValue(self.brush_size)
        self.brush_slider.valueChanged.connect(self.setBrushSize)
        toolbar.addWidget(self.brush_slider)

        # Save/Load buttons
        self.save_btn = QPushButton("Save")
        self.save_btn.clicked.connect(self.saveCanvas)
        toolbar.addWidget(self.save_btn)

        self.load_btn = QPushButton("Load")
        self.load_btn.clicked.connect(self.loadCanvas)
        toolbar.addWidget(self.load_btn)

        layout.addLayout(toolbar)

    def initCamera(self):
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.timer = QTimer()
        self.timer.timeout.connect(self.updateFrame)
        self.timer.start(30)  # ~30 FPS

    def updateFrame(self):
        ret, frame = self.cap.read()
        if ret:
            gesture, x, y, debug_frame = self.detector.detect(frame)

            # Scale coordinates
            canvas_x = int(x * (self.canvas.width() / frame.shape[1]))
            canvas_y = int(y * (self.canvas.height() / frame.shape[0]))

            # Handle gestures
            if gesture == "paint":
                self.drawOnCanvas(canvas_x, canvas_y)
            elif gesture == "erase":
                self.eraseOnCanvas(canvas_x, canvas_y)
            elif gesture == "full_erase":
                self.clearCanvas()  # Clear entire screen
            elif gesture == "change_color":
                self.cycleColor()

            # Display debug view
            debug_frame = cv2.cvtColor(debug_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = debug_frame.shape
            q_img = QImage(debug_frame.data, w, h, ch * w, QImage.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(q_img))

    def drawOnCanvas(self, x, y):
        # Save current state for undo
        if len(self.canvas_history) > 10:
            self.canvas_history.pop(0)
        self.canvas_history.append(self.canvas.copy())

        painter = QPainter(self.canvas)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(
            self.color,
            self.brush_size,
            Qt.SolidLine,
            Qt.RoundCap,
            Qt.RoundJoin
        ))

        current_point = QPoint(x, y)
        if self.last_point:
            painter.drawLine(self.last_point, current_point)
        else:
            painter.drawPoint(current_point)

        painter.end()
        self.canvas_label.setPixmap(self.canvas)
        self.last_point = current_point

    def eraseOnCanvas(self, x, y):
        painter = QPainter(self.canvas)
        painter.setPen(QPen(Qt.white, self.brush_size * 2))
        painter.drawPoint(QPoint(x, y))
        painter.end()
        self.canvas_label.setPixmap(self.canvas)
        self.last_point = None

    def cycleColor(self):
        current_idx = self.colors.index(self.color)
        self.color = self.colors[(current_idx + 1) % len(self.colors)]

    def setColor(self, color):
        self.color = color

    def setBrushSize(self, size):
        self.brush_size = size

    def clearCanvas(self):
        self.canvas.fill(Qt.white)
        self.canvas_label.setPixmap(self.canvas)
        self.last_point = None

    def saveCanvas(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Image", "", "PNG Files (*.png)"
        )
        if path:
            self.canvas.save(path)

    def loadCanvas(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Image", "", "PNG Files (*.png)"
        )
        if path:
            self.canvas = QPixmap(path)
            self.canvas_label.setPixmap(self.canvas)

    def closeEvent(self, event):
        self.cap.release()
        event.accept()