import sys
import random
from dataclasses import dataclass, field
from typing import Dict, List

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
)


@dataclass
class RailData:
    name: str
    commanded: bool
    pg: bool
    flt: bool
    voltage: float
    current: float
    state: str


@dataclass
class EPSData:
    eps_state: str = "NORMAL"
    pack_voltage: float = 14.8
    cells: List[float] = field(default_factory=lambda: [3.70, 3.71, 3.69, 3.70])
    battery_temp: float = 24.5
    charger_status: str = "CHARGING"
    charger_vin: float = 18.2
    charge_current: float = 0.45
    charger_fault: bool = False
    rails: Dict[str, RailData] = field(default_factory=dict)
    faults: List[str] = field(default_factory=list)


class StatusLabel(QLabel):
    def __init__(self, text=""):
        super().__init__(text)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumHeight(28)
        self.setStyleSheet("font-weight: bold; border-radius: 8px; padding: 4px;")

    def set_status(self, text: str, level: str):
        colours = {
            "ok": "background-color: #d8f5d0; color: #145214;",
            "warn": "background-color: #fff3bf; color: #7a5200;",
            "fault": "background-color: #ffd6d6; color: #7a0000;",
            "off": "background-color: #e5e5e5; color: #555555;",
        }
        self.setText(text)
        self.setStyleSheet(
            f"font-weight: bold; border-radius: 8px; padding: 4px; {colours.get(level, colours['off'])}"
        )


class MetricCard(QGroupBox):
    def __init__(self, title: str):
        super().__init__(title)
        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.rows = {}

    def add_metric(self, key: str, label: str, row: int):
        name = QLabel(label)
        value = QLabel("--")
        value.setAlignment(Qt.AlignmentFlag.AlignRight)
        value.setStyleSheet("font-weight: bold;")
        self.layout.addWidget(name, row, 0)
        self.layout.addWidget(value, row, 1)
        self.rows[key] = value

    def set_metric(self, key: str, value: str):
        self.rows[key].setText(value)


class RailCard(QGroupBox):
    def __init__(self, rail_name: str):
        super().__init__(rail_name)
        layout = QGridLayout()
        self.setLayout(layout)

        self.state = StatusLabel()
        self.commanded = QLabel("--")
        self.pg = QLabel("--")
        self.flt = QLabel("--")
        self.voltage = QLabel("--")
        self.current = QLabel("--")

        layout.addWidget(self.state, 0, 0, 1, 2)
        layout.addWidget(QLabel("Commanded"), 1, 0)
        layout.addWidget(self.commanded, 1, 1)
        layout.addWidget(QLabel("Power Good"), 2, 0)
        layout.addWidget(self.pg, 2, 1)
        layout.addWidget(QLabel("Fault"), 3, 0)
        layout.addWidget(self.flt, 3, 1)
        layout.addWidget(QLabel("Voltage"), 4, 0)
        layout.addWidget(self.voltage, 4, 1)
        layout.addWidget(QLabel("Current"), 5, 0)
        layout.addWidget(self.current, 5, 1)

    def update_data(self, rail: RailData):
        if rail.flt:
            level = "fault"
        elif not rail.commanded:
            level = "off"
        elif rail.pg:
            level = "ok"
        else:
            level = "warn"

        self.state.set_status(rail.state, level)
        self.commanded.setText("ON" if rail.commanded else "OFF")
        self.pg.setText("YES" if rail.pg else "NO")
        self.flt.setText("YES" if rail.flt else "NO")
        self.voltage.setText(f"{rail.voltage:.2f} V")
        self.current.setText(f"{rail.current:.2f} A")


class EPSDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EPS Dashboard - Dummy Data")
        self.resize(1100, 720)

        self.data = self.create_dummy_data()

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout()
        central.setLayout(main_layout)

        header = QHBoxLayout()
        self.eps_state = StatusLabel()
        self.timestamp = QLabel("Telemetry: dummy live data")
        self.timestamp.setAlignment(Qt.AlignmentFlag.AlignRight)
        header.addWidget(QLabel("Electrical Power System Dashboard"))
        header.addWidget(self.eps_state)
        header.addWidget(self.timestamp)
        main_layout.addLayout(header)

        top_grid = QGridLayout()
        main_layout.addLayout(top_grid)

        self.battery_card = MetricCard("Battery Status")
        for i in range(4):
            self.battery_card.add_metric(f"cell_{i+1}", f"Cell {i+1}", i)
        self.battery_card.add_metric("pack", "Pack Voltage", 4)
        self.battery_card.add_metric("delta", "Cell Delta", 5)
        self.battery_card.add_metric("temp", "Battery Temp", 6)
        top_grid.addWidget(self.battery_card, 0, 0)

        self.charger_card = MetricCard("BQ25798 Charger")
        self.charger_card.add_metric("status", "Status", 0)
        self.charger_card.add_metric("vin", "Input Voltage", 1)
        self.charger_card.add_metric("current", "Charge Current", 2)
        self.charger_card.add_metric("fault", "Fault", 3)
        top_grid.addWidget(self.charger_card, 0, 1)

        self.fault_card = QGroupBox("Faults")
        fault_layout = QVBoxLayout()
        self.fault_card.setLayout(fault_layout)
        self.fault_status = StatusLabel()
        self.fault_text = QLabel("No active faults")
        self.fault_text.setWordWrap(True)
        fault_layout.addWidget(self.fault_status)
        fault_layout.addWidget(self.fault_text)
        top_grid.addWidget(self.fault_card, 0, 2)

        rail_box = QGroupBox("Power Rails / TPS259813A eFuses")
        rail_grid = QGridLayout()
        rail_box.setLayout(rail_grid)
        main_layout.addWidget(rail_box)

        self.rail_cards = {}
        rail_names = ["14.8 V Rail", "5 V Rail", "3.3 V Rail", "Payload", "ADCS", "Comms"]
        for index, name in enumerate(rail_names):
            card = RailCard(name)
            self.rail_cards[name] = card
            rail_grid.addWidget(card, index // 3, index % 3)

        controls = QHBoxLayout()
        self.freeze_button = QPushButton("Pause Dummy Data")
        self.freeze_button.setCheckable(True)
        self.freeze_button.clicked.connect(self.toggle_freeze)
        controls.addWidget(self.freeze_button)
        controls.addStretch()
        main_layout.addLayout(controls)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_dummy_data)
        self.timer.start(1000)

        self.update_dashboard()

    def create_dummy_data(self) -> EPSData:
        rails = {
            "14.8 V Rail": RailData("14.8 V Rail", True, True, False, 14.80, 0.52, "ON"),
            "5 V Rail": RailData("5 V Rail", True, True, False, 5.01, 0.31, "ON"),
            "3.3 V Rail": RailData("3.3 V Rail", True, True, False, 3.30, 0.18, "ON"),
            "Payload": RailData("Payload", False, False, False, 0.00, 0.00, "OFF"),
            "ADCS": RailData("ADCS", True, True, False, 5.02, 0.22, "ON"),
            "Comms": RailData("Comms", True, True, False, 3.29, 0.15, "ON"),
        }
        return EPSData(rails=rails)

    def refresh_dummy_data(self):
        self.data.pack_voltage = random.uniform(14.55, 15.05)
        base = self.data.pack_voltage / 4.0
        self.data.cells = [base + random.uniform(-0.025, 0.025) for _ in range(4)]
        self.data.battery_temp = random.uniform(22.0, 28.0)
        self.data.charger_vin = random.uniform(17.5, 18.6)
        self.data.charge_current = random.uniform(0.25, 0.85)
        self.data.charger_status = random.choice(["CHARGING", "CHARGING", "CHARGING", "IDLE"])
        self.data.charger_fault = False
        self.data.eps_state = random.choice(["NORMAL", "NORMAL", "NORMAL", "CHARGING"])

        for rail in self.data.rails.values():
            if rail.commanded:
                nominal = 14.8 if "14.8" in rail.name else 5.0 if "5" in rail.name or rail.name == "ADCS" else 3.3
                rail.voltage = random.uniform(nominal * 0.985, nominal * 1.015)
                rail.current = random.uniform(0.05, 0.65)
                rail.pg = True
                rail.flt = False
                rail.state = "ON"
            else:
                rail.voltage = 0.0
                rail.current = 0.0
                rail.pg = False
                rail.flt = False
                rail.state = "OFF"

        if random.random() < 0.08:
            rail = random.choice(list(self.data.rails.values()))
            rail.flt = True
            rail.pg = False
            rail.state = "FAULT"
            self.data.faults = [f"{rail.name} eFuse fault"]
            self.data.eps_state = "FAULT"
        else:
            self.data.faults = []

        self.update_dashboard()

    def update_dashboard(self):
        state_level = "fault" if self.data.eps_state == "FAULT" else "ok"
        self.eps_state.set_status(self.data.eps_state, state_level)

        for i, cell in enumerate(self.data.cells):
            self.battery_card.set_metric(f"cell_{i+1}", f"{cell:.3f} V")

        cell_delta_mv = (max(self.data.cells) - min(self.data.cells)) * 1000
        self.battery_card.set_metric("pack", f"{self.data.pack_voltage:.2f} V")
        self.battery_card.set_metric("delta", f"{cell_delta_mv:.0f} mV")
        self.battery_card.set_metric("temp", f"{self.data.battery_temp:.1f} °C")

        self.charger_card.set_metric("status", self.data.charger_status)
        self.charger_card.set_metric("vin", f"{self.data.charger_vin:.2f} V")
        self.charger_card.set_metric("current", f"{self.data.charge_current:.2f} A")
        self.charger_card.set_metric("fault", "YES" if self.data.charger_fault else "NO")

        if self.data.faults:
            self.fault_status.set_status("ACTIVE FAULT", "fault")
            self.fault_text.setText("\n".join(self.data.faults))
        else:
            self.fault_status.set_status("NO ACTIVE FAULTS", "ok")
            self.fault_text.setText("No active faults")

        for name, rail in self.data.rails.items():
            self.rail_cards[name].update_data(rail)

    def toggle_freeze(self):
        if self.freeze_button.isChecked():
            self.timer.stop()
            self.freeze_button.setText("Resume Dummy Data")
        else:
            self.timer.start(1000)
            self.freeze_button.setText("Pause Dummy Data")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EPSDashboard()
    window.show()
    sys.exit(app.exec())
