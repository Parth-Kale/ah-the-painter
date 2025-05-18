import sys
from PyQt5.QtWidgets import QApplication
from painter_window import PainterWindow
from gesture_detector import GestureDetector


def main():
    app = QApplication(sys.argv)

    # Initialize detector and window
    detector = GestureDetector()
    window = PainterWindow(detector)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()