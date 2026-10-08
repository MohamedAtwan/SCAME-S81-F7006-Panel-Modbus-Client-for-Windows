"""S81-F7006 Modbus TCP/RTU commissioning client for Windows.

Designed for a PC Modbus TCP client connected through a Moxa MGate MB3170
to an S81-F7006 Modbus RTU slave. Runtime dependencies: Python standard
library only.
"""

from __future__ import annotations

import queue
import socket
import struct
import threading
from dataclasses import dataclass
from datetime import datetime
import tkinter as tk
from tkinter import messagebox, ttk


class ModbusError(Exception):
    """Raised for Modbus protocol and device exception responses."""


EXCEPTION_NAMES = {
    1: "Illegal Function",
    2: "Illegal Data Address",
    3: "Illegal Data Value",
    4: "Slave Device Failure",
    5: "Acknowledge",
    6: "Slave Device Busy",
    8: "Memory Parity Error",
    10: "Gateway Path Unavailable",
    11: "Gateway Target Device Failed to Respond",
}

PANEL_COMMAND_OFFSETS = {
    "Silence Panel": 0,
    "Silence Sounder": 1,
    "Reset Panel": 2,
    "Evacuate": 3,
}


@dataclass(frozen=True)
class PanelStatusPoint:
    description: str
    reference: int
    active_color: str

    @property
    def protocol_offset(self) -> int:
        return fc02_reference_to_offset(self.reference)


PANEL_STATUS_POINTS = (
    PanelStatusPoint("FIRE ALARM", 10001, "#d32f2f"),
    PanelStatusPoint("FIRE PREALARM", 10002, "#f57c00"),
    PanelStatusPoint("FAULT (SAFETY)", 10003, "#fbc02d"),
    PanelStatusPoint("DISABLED (SAFETY)", 10004, "#ff9800"),
    PanelStatusPoint("EXTERNAL FAULT (SAFETY)", 10005, "#fbc02d"),
    PanelStatusPoint("FIRE SOUNDER (SAFETY)", 10006, "#d32f2f"),
    PanelStatusPoint("GAS ALARM (SAFETY)", 10007, "#d32f2f"),
    PanelStatusPoint("SUPERVISORY (SAFETY)", 10008, "#ff9800"),
    PanelStatusPoint("FIRE 2 ALARM (SAFETY)", 10009, "#d32f2f"),
    PanelStatusPoint("OUTPUTS ACTIVATION (SAFETY)", 10010, "#388e3c"),
    PanelStatusPoint("ALARM (SECURITY)", 10011, "#d32f2f"),
    PanelStatusPoint("TAMPER (SECURITY)", 10012, "#ff9800"),
    PanelStatusPoint("FAULT (SECURITY)", 10013, "#fbc02d"),
    PanelStatusPoint("EXTERNAL TAMPER (SECURITY)", 10014, "#ff9800"),
    PanelStatusPoint("SOUNDER (SECURITY)", 10015, "#d32f2f"),
    PanelStatusPoint("CPU VOLTAGE OUT OF RANGE", 10021, "#fbc02d"),
    PanelStatusPoint("EARTH LEAKAGE FAULT", 10022, "#fbc02d"),
    PanelStatusPoint("PSU-1 FAULT", 10023, "#fbc02d"),
    PanelStatusPoint("PSU-2 FAULT", 10024, "#fbc02d"),
    PanelStatusPoint("BATTERY STATUS", 10025, "#388e3c"),
    PanelStatusPoint("CHARGE STATUS", 10026, "#388e3c"),
    PanelStatusPoint("SYSTEM FAULT", 10032, "#fbc02d"),
    PanelStatusPoint("CPU-1 FAILURE", 10033, "#fbc02d"),
    PanelStatusPoint("CPU-2 FAILURE", 10034, "#fbc02d"),
    PanelStatusPoint("PANEL SOUNDER FAULT", 10035, "#fbc02d"),
    PanelStatusPoint("REMOTE DISPLAY FAULT", 10036, "#fbc02d"),
    PanelStatusPoint("HOST TCP/IP COMM FAULT", 10037, "#fbc02d"),
    PanelStatusPoint("NETWORK COMM FAULT", 10038, "#fbc02d"),
    PanelStatusPoint("DISPLAY CARD FAILURE", 10039, "#fbc02d"),
    PanelStatusPoint("DEFAULT I/O CARD FAILURE", 10040, "#fbc02d"),
    PanelStatusPoint("CARD FAILURE (COMMON)", 10041, "#fbc02d"),
    PanelStatusPoint("LOOP CARDS: LOOP OPEN", 10042, "#fbc02d"),
    PanelStatusPoint("LOOP CARDS: LOOP SHORT CIRCUIT", 10043, "#fbc02d"),
    PanelStatusPoint("LOOP CARDS: LOGON FAULT", 10044, "#fbc02d"),
    PanelStatusPoint("PANEL RESET", 10045, "#1976d2"),
    PanelStatusPoint("PANEL BUZZER", 10046, "#1976d2"),
    PanelStatusPoint("EVACUATION", 10047, "#d32f2f"),
)


def fc02_reference_to_offset(reference: int) -> int:
    """Convert a 1xxxx discrete-input reference to its zero-based wire offset."""
    if not 10001 <= reference <= 19999:
        raise ValueError("FC02 reference must be from 10001 to 19999")
    return reference - 10001


MONITOR_REFERENCE_START = min(point.reference for point in PANEL_STATUS_POINTS)
MONITOR_REFERENCE_END = max(point.reference for point in PANEL_STATUS_POINTS)
MONITOR_PROTOCOL_START = fc02_reference_to_offset(MONITOR_REFERENCE_START)
MONITOR_QUANTITY = MONITOR_REFERENCE_END - MONITOR_REFERENCE_START + 1


def decode_panel_status_bits(bits: list[bool]) -> dict[int, bool]:
    """Return the defined panel-status reference values from a monitoring block."""
    if len(bits) < MONITOR_QUANTITY:
        raise ValueError(
            f"Panel status block requires {MONITOR_QUANTITY} bits, received {len(bits)}"
        )
    return {
        point.reference: bool(bits[point.reference - MONITOR_REFERENCE_START])
        for point in PANEL_STATUS_POINTS
    }


def panel_bases(addressing_mode: str, card_address: int) -> tuple[int, int, int]:
    """Return (V-IN, V-OUT, V-INSYS) bases from the ST-007 manual."""
    if addressing_mode == "standard":
        return 0, 0, 512
    if addressing_mode != "extended":
        raise ValueError("Addressing mode must be standard or extended")
    if not 1 <= card_address <= 5:
        raise ValueError("Extended mode card address must be from 1 to 5")
    offset = (card_address - 1) * 2000
    return offset, offset + 512, offset + 1024


def panel_command_address(
    addressing_mode: str, card_address: int, command_name: str
) -> int:
    if command_name not in PANEL_COMMAND_OFFSETS:
        raise ValueError("Unknown panel command")
    _, _, vinsys_base = panel_bases(addressing_mode, card_address)
    return vinsys_base + PANEL_COMMAND_OFFSETS[command_name]


@dataclass(frozen=True)
class ConnectionSettings:
    host: str
    port: int
    unit_id: int
    timeout: float


class ModbusTcpClient:
    """Synchronous Modbus TCP client supporting FC01, FC02, and FC05."""

    def __init__(self) -> None:
        self._socket: socket.socket | None = None
        self._settings: ConnectionSettings | None = None
        self._transaction_id = 0
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._socket is not None

    @property
    def settings(self) -> ConnectionSettings | None:
        return self._settings

    def connect(self, settings: ConnectionSettings) -> None:
        self.disconnect()
        sock = socket.create_connection(
            (settings.host, settings.port), timeout=settings.timeout
        )
        sock.settimeout(settings.timeout)
        self._socket = sock
        self._settings = settings

    def disconnect(self) -> None:
        sock, self._socket = self._socket, None
        self._settings = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def _recv_exact(self, size: int) -> bytes:
        if self._socket is None:
            raise ConnectionError("Not connected")
        data = bytearray()
        while len(data) < size:
            chunk = self._socket.recv(size - len(data))
            if not chunk:
                raise ConnectionError("The remote device closed the connection")
            data.extend(chunk)
        return bytes(data)

    def _request(self, function_code: int, payload: bytes) -> bytes:
        if self._socket is None or self._settings is None:
            raise ConnectionError("Not connected")

        with self._lock:
            self._transaction_id = (self._transaction_id + 1) & 0xFFFF
            transaction_id = self._transaction_id
            pdu = bytes([function_code]) + payload
            header = struct.pack(
                ">HHHB", transaction_id, 0, len(pdu) + 1, self._settings.unit_id
            )
            self._socket.sendall(header + pdu)

            response_header = self._recv_exact(7)
            rx_transaction, protocol_id, length, rx_unit = struct.unpack(
                ">HHHB", response_header
            )
            if rx_transaction != transaction_id:
                raise ModbusError(
                    f"Transaction ID mismatch: sent {transaction_id}, received {rx_transaction}"
                )
            if protocol_id != 0:
                raise ModbusError(f"Invalid protocol ID: {protocol_id}")
            if rx_unit != self._settings.unit_id:
                raise ModbusError(
                    f"Unit ID mismatch: sent {self._settings.unit_id}, received {rx_unit}"
                )
            if length < 2:
                raise ModbusError("Invalid response length")

            response_pdu = self._recv_exact(length - 1)
            rx_function = response_pdu[0]
            if rx_function == (function_code | 0x80):
                if len(response_pdu) < 2:
                    raise ModbusError("Malformed Modbus exception response")
                code = response_pdu[1]
                name = EXCEPTION_NAMES.get(code, "Unknown exception")
                raise ModbusError(f"Device exception {code}: {name}")
            if rx_function != function_code:
                raise ModbusError(
                    f"Function code mismatch: sent {function_code}, received {rx_function}"
                )
            return response_pdu[1:]

    def read_bits(self, function_code: int, start_address: int, quantity: int) -> list[bool]:
        if function_code not in (1, 2):
            raise ValueError("Bit-reading function code must be 1 or 2")
        if not 0 <= start_address <= 65535:
            raise ValueError("Start address must be from 0 to 65535")
        if not 1 <= quantity <= 2000:
            raise ValueError("Quantity must be from 1 to 2000")

        response = self._request(
            function_code, struct.pack(">HH", start_address, quantity)
        )
        if not response:
            raise ModbusError(f"Empty FC{function_code:02d} response")
        byte_count = response[0]
        bit_bytes = response[1:]
        expected_bytes = (quantity + 7) // 8
        if byte_count != len(bit_bytes) or byte_count < expected_bytes:
            raise ModbusError(f"Invalid FC{function_code:02d} byte count")

        return [
            bool(bit_bytes[index // 8] & (1 << (index % 8)))
            for index in range(quantity)
        ]

    def read_input_variables(self, start_address: int, quantity: int) -> list[bool]:
        return self.read_bits(1, start_address, quantity)

    def read_output_variables(self, start_address: int, quantity: int) -> list[bool]:
        return self.read_bits(2, start_address, quantity)

    def read_discrete_references(
        self, start_reference: int, quantity: int
    ) -> list[bool]:
        """Read FC02 values using conventional 1xxxx reference notation."""
        start_address = fc02_reference_to_offset(start_reference)
        return self.read_bits(2, start_address, quantity)

    def read_coils(self, start_address: int, quantity: int) -> list[bool]:
        """Backward-compatible name from the original version."""
        return self.read_input_variables(start_address, quantity)

    def write_single_input(self, address: int, value: bool) -> None:
        if not 0 <= address <= 65535:
            raise ValueError("Variable address must be from 0 to 65535")
        encoded_value = 0xFF00 if value else 0x0000
        request_payload = struct.pack(">HH", address, encoded_value)
        response = self._request(5, request_payload)
        if response != request_payload:
            raise ModbusError("FC05 response does not echo the requested address/value")

    def write_single_coil(self, address: int, value: bool) -> None:
        """Backward-compatible name from the original version."""
        self.write_single_input(address, value)


class ModbusApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("S81-F7006 Panel Modbus Client")
        self.geometry("1040x860")
        self.minsize(900, 740)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.client = ModbusTcpClient()
        self.results: queue.Queue[tuple[str, object]] = queue.Queue()
        self._working = False
        self.operation_buttons: list[ttk.Button] = []
        self.momentary_buttons: dict[str, ttk.Button] = {}
        self._active_momentary: tuple[str, int] | None = None
        self._momentary_release_event = threading.Event()
        self._pending_disconnect = False
        self._pending_close = False
        self._monitoring = False
        self._monitor_generation = 0
        self._monitor_stop_event: threading.Event | None = None
        self._last_monitor_values: dict[int, bool] | None = None
        self.status_led_items: dict[int, tuple[tk.Canvas, int, tk.StringVar]] = {}

        self.host_var = tk.StringVar(value="192.168.127.254")
        self.port_var = tk.StringVar(value="502")
        self.unit_var = tk.StringVar(value="1")
        self.timeout_var = tk.StringVar(value="3.0")
        self.addressing_mode_var = tk.StringVar(value="standard")
        self.card_address_var = tk.StringVar(value="1")
        self.bases_var = tk.StringVar()
        self.commands_armed_var = tk.BooleanVar(value=False)
        self.command_help_var = tk.StringVar()
        self.read_function_var = tk.StringVar(value="FC02 - V-OUT status")
        self.read_address_var = tk.StringVar(value="0")
        self.quantity_var = tk.StringVar(value="16")
        self.write_address_var = tk.StringVar(value="0")
        self.status_var = tk.StringVar(value="Disconnected")
        self.monitor_interval_var = tk.StringVar(value="1.0")
        self.monitor_state_var = tk.StringVar(value="Stopped")
        self.monitor_last_update_var = tk.StringVar(value="Last update: --")

        self._build_ui()
        self.addressing_mode_var.trace_add("write", self._refresh_panel_mapping)
        self.card_address_var.trace_add("write", self._refresh_panel_mapping)
        self.bind_all("<ButtonRelease-1>", self._global_button_release, add="+")
        self.bind_all("<Escape>", self._escape_release, add="+")
        self._refresh_panel_mapping()
        self.after(100, self._poll_results)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)

        connection = ttk.LabelFrame(self, text="Moxa MB3170 connection", padding=12)
        connection.grid(row=0, column=0, padx=12, pady=(12, 6), sticky="ew")
        for index in (1, 3, 5, 7):
            connection.columnconfigure(index, weight=1)

        ttk.Label(connection, text="Moxa IP address").grid(row=0, column=0, sticky="w")
        ttk.Entry(connection, textvariable=self.host_var, width=18).grid(
            row=0, column=1, padx=(6, 14), sticky="ew"
        )
        ttk.Label(connection, text="TCP port").grid(row=0, column=2, sticky="w")
        ttk.Entry(connection, textvariable=self.port_var, width=8).grid(
            row=0, column=3, padx=(6, 14), sticky="ew"
        )
        ttk.Label(connection, text="RTU Unit ID").grid(row=0, column=4, sticky="w")
        ttk.Entry(connection, textvariable=self.unit_var, width=7).grid(
            row=0, column=5, padx=(6, 14), sticky="ew"
        )
        ttk.Label(connection, text="Timeout (s)").grid(row=0, column=6, sticky="w")
        ttk.Entry(connection, textvariable=self.timeout_var, width=7).grid(
            row=0, column=7, padx=(6, 0), sticky="ew"
        )

        self.connect_button = ttk.Button(connection, text="Connect", command=self._connect)
        self.connect_button.grid(row=1, column=0, columnspan=2, pady=(12, 0), sticky="ew")
        self.disconnect_button = ttk.Button(
            connection, text="Disconnect", command=self._disconnect, state="disabled"
        )
        self.disconnect_button.grid(
            row=1, column=2, columnspan=2, padx=(8, 0), pady=(12, 0), sticky="ew"
        )
        ttk.Label(connection, textvariable=self.status_var).grid(
            row=1, column=4, columnspan=4, padx=(14, 0), pady=(12, 0), sticky="w"
        )

        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=1, column=0, padx=12, pady=6, sticky="nsew")
        command_tab = ttk.Frame(self.notebook, padding=12)
        monitor_tab = ttk.Frame(self.notebook, padding=12)
        read_tab = ttk.Frame(self.notebook, padding=12)
        manual_tab = ttk.Frame(self.notebook, padding=12)
        self.notebook.add(command_tab, text="Panel commands")
        self.notebook.add(monitor_tab, text="Panel status LEDs")
        self.notebook.add(read_tab, text="Read variables")
        self.notebook.add(manual_tab, text="Manual FC05")
        self._build_command_tab(command_tab)
        self._build_monitor_tab(monitor_tab)
        self._build_read_tab(read_tab)
        self._build_manual_tab(manual_tab)

        output = ttk.LabelFrame(self, text="Results and communication log", padding=8)
        output.grid(row=2, column=0, padx=12, pady=(6, 8), sticky="nsew")
        output.columnconfigure(0, weight=1)
        output.rowconfigure(0, weight=1)
        self.log_text = tk.Text(output, wrap="none", font=("Consolas", 10), state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        vertical = ttk.Scrollbar(output, orient="vertical", command=self.log_text.yview)
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(output, orient="horizontal", command=self.log_text.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        self.log_text.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)

        footer = ttk.Frame(self)
        footer.grid(row=3, column=0, padx=12, pady=(0, 12), sticky="ew")
        ttk.Label(
            footer,
            text="ST-007 addressing is zero-based. Verify the command and plant safety before writing.",
        ).pack(side="left")
        ttk.Button(footer, text="Clear log", command=self._clear_log).pack(side="right")

    def _build_command_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text="S81 addressing mode").grid(row=0, column=0, sticky="w")
        modes = ttk.Frame(parent)
        modes.grid(row=0, column=1, sticky="w")
        ttk.Radiobutton(
            modes,
            text="Standard (SW1-8 OFF)",
            variable=self.addressing_mode_var,
            value="standard",
        ).pack(side="left")
        ttk.Radiobutton(
            modes,
            text="Extended (SW1-8 ON)",
            variable=self.addressing_mode_var,
            value="extended",
        ).pack(side="left", padx=(16, 0))

        ttk.Label(parent, text="Card SW1 address (extended mode)").grid(
            row=1, column=0, pady=(10, 0), sticky="w"
        )
        ttk.Entry(parent, textvariable=self.card_address_var, width=8).grid(
            row=1, column=1, pady=(10, 0), sticky="w"
        )
        ttk.Label(parent, textvariable=self.bases_var).grid(
            row=2, column=0, columnspan=2, pady=(10, 0), sticky="w"
        )

        ttk.Separator(parent).grid(row=3, column=0, columnspan=2, pady=12, sticky="ew")
        arm = ttk.Checkbutton(
            parent,
            text="Arm momentary panel commands",
            variable=self.commands_armed_var,
            command=self._toggle_commands_armed,
        )
        arm.grid(row=4, column=0, columnspan=2, sticky="w")
        ttk.Label(
            parent,
            text="Press and hold a command to write 1 (FF00). Release it to write 0 (0000).",
        ).grid(row=5, column=0, columnspan=2, pady=(8, 0), sticky="w")

        command_list = ttk.Frame(parent)
        command_list.grid(row=6, column=0, columnspan=2, pady=(12, 0), sticky="ew")
        command_list.columnconfigure(0, weight=1)
        for row, command_name in enumerate(PANEL_COMMAND_OFFSETS):
            button = ttk.Button(command_list, text=command_name, state="disabled")
            button.grid(row=row, column=0, pady=3, sticky="ew", ipady=7)
            button.bind(
                "<ButtonPress-1>",
                lambda event, name=command_name: self._momentary_press(event, name),
                add="+",
            )
            button.bind(
                "<ButtonRelease-1>",
                lambda event, name=command_name: self._momentary_release(event, name),
                add="+",
            )
            self.momentary_buttons[command_name] = button

        ttk.Label(parent, textvariable=self.command_help_var).grid(
            row=7, column=0, columnspan=2, pady=(9, 0), sticky="w"
        )

    def _build_monitor_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(2, weight=1)

        controls = ttk.Frame(parent)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(7, weight=1)
        ttk.Label(controls, text="Polling interval (s)").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            controls,
            textvariable=self.monitor_interval_var,
            values=("0.5", "1.0", "2.0", "5.0"),
            width=7,
        ).grid(row=0, column=1, padx=(8, 12), sticky="w")
        self.monitor_start_button = ttk.Button(
            controls,
            text="Start monitoring",
            command=self._start_monitoring,
            state="disabled",
        )
        self.monitor_start_button.grid(row=0, column=2, padx=(0, 6), sticky="ew")
        self.monitor_stop_button = ttk.Button(
            controls,
            text="Stop monitoring",
            command=self._stop_monitoring,
            state="disabled",
        )
        self.monitor_stop_button.grid(row=0, column=3, padx=(0, 14), sticky="ew")
        ttk.Label(controls, textvariable=self.monitor_state_var).grid(
            row=0, column=4, padx=(0, 14), sticky="w"
        )
        ttk.Label(controls, textvariable=self.monitor_last_update_var).grid(
            row=0, column=5, columnspan=3, sticky="e"
        )

        ttk.Label(
            parent,
            text=(
                "FC02 references 10001-10047 are read as protocol offsets 0-46. "
                "Gray = OFF/no data; colored = ON; orange = communication error."
            ),
        ).grid(row=1, column=0, pady=(8, 8), sticky="w")

        table_frame = ttk.Frame(parent)
        table_frame.grid(row=2, column=0, sticky="nsew")
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)

        canvas = tk.Canvas(table_frame, highlightthickness=0, height=330)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scrollbar.set)

        rows = ttk.Frame(canvas)
        window_id = canvas.create_window((0, 0), window=rows, anchor="nw")
        rows.columnconfigure(1, weight=1)

        def refresh_scroll_region(_event=None) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))

        def resize_rows(event) -> None:
            canvas.itemconfigure(window_id, width=event.width)

        rows.bind("<Configure>", refresh_scroll_region)
        canvas.bind("<Configure>", resize_rows)
        canvas.bind(
            "<MouseWheel>",
            lambda event: canvas.yview_scroll(int(-event.delta / 120), "units"),
        )

        headings = ("LED", "Panel status", "Function", "Reference", "Wire offset", "State")
        for column, heading in enumerate(headings):
            ttk.Label(rows, text=heading, font=("TkDefaultFont", 9, "bold")).grid(
                row=0, column=column, padx=6, pady=(2, 5), sticky="w"
            )

        for row_number, point in enumerate(PANEL_STATUS_POINTS, start=1):
            led = tk.Canvas(rows, width=26, height=20, highlightthickness=0)
            led.grid(row=row_number, column=0, padx=6, pady=2)
            led_item = led.create_oval(4, 2, 22, 18, fill="#7f8c8d", outline="#4d4d4d")
            state_var = tk.StringVar(value="NO DATA")
            ttk.Label(rows, text=point.description).grid(
                row=row_number, column=1, padx=6, pady=2, sticky="w"
            )
            ttk.Label(rows, text="FC02").grid(
                row=row_number, column=2, padx=6, pady=2, sticky="w"
            )
            ttk.Label(rows, text=str(point.reference)).grid(
                row=row_number, column=3, padx=6, pady=2, sticky="e"
            )
            ttk.Label(rows, text=str(point.protocol_offset)).grid(
                row=row_number, column=4, padx=6, pady=2, sticky="e"
            )
            ttk.Label(rows, textvariable=state_var, width=11).grid(
                row=row_number, column=5, padx=6, pady=2, sticky="w"
            )
            self.status_led_items[point.reference] = (led, led_item, state_var)

    def _build_read_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text="Function / S81 data area").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            parent,
            textvariable=self.read_function_var,
            values=("FC01 - V-IN / V-INSYS", "FC02 - V-OUT status"),
            state="readonly",
            width=28,
        ).grid(row=0, column=1, padx=(10, 0), sticky="ew")
        ttk.Label(parent, text="Start address").grid(row=1, column=0, pady=(10, 0), sticky="w")
        ttk.Entry(parent, textvariable=self.read_address_var).grid(
            row=1, column=1, padx=(10, 0), pady=(10, 0), sticky="ew"
        )
        ttk.Label(parent, text="Quantity (1-512)").grid(row=2, column=0, pady=(10, 0), sticky="w")
        ttk.Entry(parent, textvariable=self.quantity_var).grid(
            row=2, column=1, padx=(10, 0), pady=(10, 0), sticky="ew"
        )
        buttons = ttk.Frame(parent)
        buttons.grid(row=3, column=0, columnspan=2, pady=(12, 0), sticky="ew")
        for column in (0, 1, 2, 3):
            buttons.columnconfigure(column, weight=1)
        read_button = ttk.Button(buttons, text="Read", command=self._read_variables, state="disabled")
        read_button.grid(row=0, column=0, padx=(0, 5), sticky="ew")
        self.operation_buttons.append(read_button)
        ttk.Button(buttons, text="Set V-IN base", command=lambda: self._load_base("vin")).grid(
            row=0, column=1, padx=5, sticky="ew"
        )
        ttk.Button(buttons, text="Set V-OUT base", command=lambda: self._load_base("vout")).grid(
            row=0, column=2, padx=5, sticky="ew"
        )
        ttk.Button(buttons, text="Set V-INSYS base", command=lambda: self._load_base("vinsys")).grid(
            row=0, column=3, padx=(5, 0), sticky="ew"
        )

    def _build_manual_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text="V-IN / V-INSYS address").grid(row=0, column=0, sticky="w")
        ttk.Entry(parent, textvariable=self.write_address_var).grid(
            row=0, column=1, padx=(10, 0), sticky="ew"
        )
        ttk.Label(
            parent,
            text="FC05 writes FF00 for ON and 0000 for OFF, as required by ST-007.",
        ).grid(row=1, column=0, columnspan=2, pady=(10, 0), sticky="w")
        controls = ttk.Frame(parent)
        controls.grid(row=2, column=0, columnspan=2, pady=(12, 0), sticky="ew")
        for column in (0, 1):
            controls.columnconfigure(column, weight=1)
        on_button = ttk.Button(
            controls,
            text="Write ON",
            command=lambda: self._manual_write(True),
            state="disabled",
        )
        on_button.grid(row=0, column=0, padx=(0, 5), sticky="ew")
        off_button = ttk.Button(
            controls,
            text="Write OFF",
            command=lambda: self._manual_write(False),
            state="disabled",
        )
        off_button.grid(row=0, column=1, padx=(5, 0), sticky="ew")
        self.operation_buttons.extend((on_button, off_button))

    def _settings(self) -> ConnectionSettings:
        host = self.host_var.get().strip()
        if not host:
            raise ValueError("Moxa IP address is required")
        port = int(self.port_var.get())
        unit_id = int(self.unit_var.get())
        timeout = float(self.timeout_var.get())
        if not 1 <= port <= 65535:
            raise ValueError("Port must be from 1 to 65535")
        if not 1 <= unit_id <= 127:
            raise ValueError("S81 RTU Unit ID must be from 1 to 127")
        if not 0.1 <= timeout <= 120:
            raise ValueError("Timeout must be from 0.1 to 120 seconds")
        return ConnectionSettings(host, port, unit_id, timeout)

    def _mapping(self) -> tuple[str, int, tuple[int, int, int]]:
        mode = self.addressing_mode_var.get()
        try:
            card_address = int(self.card_address_var.get())
        except ValueError as exc:
            raise ValueError("Card SW1 address must be an integer") from exc
        return mode, card_address, panel_bases(mode, card_address)

    def _refresh_panel_mapping(self, *_args) -> None:
        try:
            mode, card_address, bases = self._mapping()
            vin, vout, vinsys = bases
            self.bases_var.set(
                f"Calculated bases: V-IN={vin}, V-OUT={vout}, V-INSYS={vinsys}"
            )
            addresses = []
            for command_name, button in self.momentary_buttons.items():
                address = panel_command_address(mode, card_address, command_name)
                button.configure(text=f"{command_name}   [FC05 address {address}]")
                addresses.append(str(address))
            self.command_help_var.set("Momentary command addresses: " + ", ".join(addresses))
        except ValueError as exc:
            self.bases_var.set(str(exc))
            self.command_help_var.set("Momentary command addresses are invalid")
        self._refresh_momentary_buttons()

    def _load_base(self, area: str) -> None:
        try:
            _mode, _card, (vin, vout, vinsys) = self._mapping()
        except ValueError as exc:
            messagebox.showerror("Invalid addressing", str(exc))
            return
        address = {"vin": vin, "vout": vout, "vinsys": vinsys}[area]
        self.read_address_var.set(str(address))
        if area == "vout":
            self.read_function_var.set("FC02 - V-OUT status")
        else:
            self.read_function_var.set("FC01 - V-IN / V-INSYS")

    def _start_monitoring(self) -> None:
        if self._monitoring:
            return
        if not self.client.connected:
            messagebox.showerror("Not connected", "Connect to the Moxa gateway first")
            return
        try:
            interval = float(self.monitor_interval_var.get())
            if not 0.2 <= interval <= 60:
                raise ValueError("Polling interval must be from 0.2 to 60 seconds")
        except ValueError as exc:
            messagebox.showerror("Invalid polling interval", str(exc))
            return

        self._monitor_generation += 1
        generation = self._monitor_generation
        stop_event = threading.Event()
        self._monitor_stop_event = stop_event
        self._monitoring = True
        self._last_monitor_values = None
        self.monitor_state_var.set("Monitoring FC02...")
        self.monitor_last_update_var.set("Last update: waiting")
        self._set_monitor_controls()
        self._log(
            "Status monitoring started: FC02 references "
            f"{MONITOR_REFERENCE_START}-{MONITOR_REFERENCE_END}, "
            f"wire start={MONITOR_PROTOCOL_START}, quantity={MONITOR_QUANTITY}, "
            f"interval={interval:g}s"
        )
        threading.Thread(
            target=self._monitor_worker,
            args=(generation, stop_event, interval),
            daemon=True,
        ).start()

    def _stop_monitoring(self, clear_leds: bool = True, log: bool = True) -> None:
        was_monitoring = self._monitoring
        self._monitoring = False
        self._monitor_generation += 1
        if self._monitor_stop_event is not None:
            self._monitor_stop_event.set()
        self._monitor_stop_event = None
        self._last_monitor_values = None
        self.monitor_state_var.set("Stopped")
        self.monitor_last_update_var.set("Last update: --")
        if clear_leds:
            self._set_all_monitor_leds("#7f8c8d", "NO DATA")
        self._set_monitor_controls()
        if was_monitoring and log:
            self._log("Status monitoring stopped")

    def _monitor_worker(
        self, generation: int, stop_event: threading.Event, interval: float
    ) -> None:
        while not stop_event.is_set():
            try:
                bits = self.client.read_discrete_references(
                    MONITOR_REFERENCE_START, MONITOR_QUANTITY
                )
            except Exception as exc:
                if not stop_event.is_set():
                    self.results.put(("monitor_error", (generation, exc)))
                return
            self.results.put(("monitor_update", (generation, datetime.now(), bits)))
            if stop_event.wait(interval):
                return

    def _set_monitor_controls(self) -> None:
        if self._monitoring:
            self.monitor_start_button.configure(state="disabled")
            self.monitor_stop_button.configure(state="normal")
        else:
            self.monitor_start_button.configure(
                state="normal" if self.client.connected else "disabled"
            )
            self.monitor_stop_button.configure(state="disabled")

    def _set_all_monitor_leds(self, color: str, state: str) -> None:
        for led, led_item, state_var in self.status_led_items.values():
            led.itemconfigure(led_item, fill=color)
            state_var.set(state)

    def _apply_monitor_values(self, timestamp: datetime, bits: list[bool]) -> None:
        values = decode_panel_status_bits(bits)
        for point in PANEL_STATUS_POINTS:
            active = values[point.reference]
            led, led_item, state_var = self.status_led_items[point.reference]
            led.itemconfigure(led_item, fill=point.active_color if active else "#7f8c8d")
            state_var.set("ON" if active else "OFF")

        if self._last_monitor_values is None:
            active_count = sum(values.values())
            self._log(
                f"Status monitoring initialized: {active_count} of "
                f"{len(PANEL_STATUS_POINTS)} defined signals ON"
            )
        else:
            for point in PANEL_STATUS_POINTS:
                previous = self._last_monitor_values[point.reference]
                current = values[point.reference]
                if previous != current:
                    self._log(
                        f"STATUS CHANGE: {point.description} [{point.reference}] = "
                        f"{'ON' if current else 'OFF'}"
                    )
        self._last_monitor_values = values
        self.monitor_state_var.set("Monitoring FC02")
        self.monitor_last_update_var.set(
            "Last update: " + timestamp.strftime("%H:%M:%S")
        )

    def _show_monitor_error(self, exc: Exception) -> None:
        self._monitoring = False
        if self._monitor_stop_event is not None:
            self._monitor_stop_event.set()
        self._monitor_stop_event = None
        self._last_monitor_values = None
        self.monitor_state_var.set("Monitoring error")
        self.monitor_last_update_var.set("Last update: failed")
        self._set_all_monitor_leds("#ff8c00", "COMM ERROR")
        if self._active_momentary is not None:
            self._log("Monitoring error: releasing the active command for safety")
            self._momentary_release_event.set()
        self.commands_armed_var.set(False)
        self.client.disconnect()
        self.status_var.set("Disconnected after monitoring communication error")
        self.connect_button.configure(state="normal")
        self.disconnect_button.configure(state="disabled")
        self._set_operation_buttons("disabled")
        self._refresh_momentary_buttons()
        self._set_monitor_controls()
        self._log(f"ERROR during status monitoring: {type(exc).__name__}: {exc}")

    def _run(self, action: str, function) -> None:
        if self._working:
            return
        self._working = True
        self._set_operation_buttons("disabled")
        self._refresh_momentary_buttons()

        def worker() -> None:
            try:
                result = function()
                self.results.put((action, result))
            except Exception as exc:
                self.results.put(("error", (action, exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _connect(self) -> None:
        try:
            settings = self._settings()
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Invalid connection settings", str(exc))
            return
        self.status_var.set("Connecting...")
        self._log(
            f"Connecting to Moxa {settings.host}:{settings.port}, RTU Unit ID {settings.unit_id}"
        )
        self._run("connect", lambda: self.client.connect(settings))

    def _disconnect(self) -> None:
        if self._active_momentary is not None:
            self._pending_disconnect = True
            self.status_var.set("Releasing command before disconnect...")
            self._momentary_release_event.set()
            self._log("Disconnect requested: releasing the active command first")
            return
        self._disconnect_now()

    def _disconnect_now(self) -> None:
        self._stop_monitoring(clear_leds=True, log=False)
        self.client.disconnect()
        self._working = False
        self._pending_disconnect = False
        self.commands_armed_var.set(False)
        self.status_var.set("Disconnected")
        self.connect_button.configure(state="normal")
        self.disconnect_button.configure(state="disabled")
        self._set_operation_buttons("disabled")
        self._refresh_momentary_buttons()
        self._log("Disconnected")

    def _read_variables(self) -> None:
        try:
            address = int(self.read_address_var.get())
            quantity = int(self.quantity_var.get())
            if not 0 <= address <= 65535:
                raise ValueError("Start address must be from 0 to 65535")
            if not 1 <= quantity <= 512:
                raise ValueError("Quantity must be from 1 to 512 for the S81 card")
        except ValueError as exc:
            messagebox.showerror("Invalid read request", str(exc))
            return
        function_code = 2 if self.read_function_var.get().startswith("FC02") else 1
        area = "V-OUT" if function_code == 2 else "V-IN/V-INSYS"
        self._log(
            f"TX FC{function_code:02d}: {area}, start address={address}, quantity={quantity}"
        )
        self._run(
            "read",
            lambda: (
                function_code,
                address,
                self.client.read_bits(function_code, address, quantity),
            ),
        )

    def _manual_write(self, value: bool) -> None:
        try:
            address = int(self.write_address_var.get())
            if not 0 <= address <= 65535:
                raise ValueError("Variable address must be from 0 to 65535")
        except ValueError as exc:
            messagebox.showerror("Invalid FC05 request", str(exc))
            return
        state = "ON" if value else "OFF"
        if not messagebox.askyesno(
            "Confirm FC05 write",
            f"Write address {address} {state}?\n\nVerify plant safety before continuing.",
        ):
            return
        self._log(f"TX FC05: variable address={address}, value={state}")

        def command() -> tuple[str, int, str]:
            self.client.write_single_input(address, value)
            return "Manual", address, state

        self._run("write", command)

    def _toggle_commands_armed(self) -> None:
        if self.commands_armed_var.get():
            if not self.client.connected:
                self.commands_armed_var.set(False)
                messagebox.showerror("Not connected", "Connect to the Moxa gateway first")
            elif not messagebox.askyesno(
                "Arm panel commands",
                "Arm the four momentary panel-command buttons?\n\n"
                "Pressing a button can immediately operate the fire panel. "
                "Confirm that the test is authorized and safe.",
            ):
                self.commands_armed_var.set(False)
            else:
                self._log("Momentary panel commands armed")
        else:
            if self._active_momentary is not None:
                self._momentary_release_event.set()
            self._log("Momentary panel commands disarmed")
        self._refresh_momentary_buttons()

    def _momentary_press(self, _event, command_name: str):
        if not self.commands_armed_var.get() or not self.client.connected:
            self.bell()
            return "break"
        if self._working or self._active_momentary is not None:
            self.bell()
            return "break"
        try:
            mode, card_address, _bases = self._mapping()
            address = panel_command_address(mode, card_address, command_name)
        except ValueError as exc:
            messagebox.showerror("Invalid panel command", str(exc))
            return "break"

        self._active_momentary = (command_name, address)
        self._momentary_release_event.clear()
        self._working = True
        self._set_operation_buttons("disabled")
        self._refresh_momentary_buttons()
        self._log(
            f"BUTTON PRESSED: {command_name}, {mode} addressing, address={address}"
        )
        self._log(f"TX FC05: address={address}, value=ON (FF00)")

        release_event = self._momentary_release_event
        settings = self.client.settings

        def command() -> None:
            try:
                self.client.write_single_input(address, True)
                self.results.put(("momentary_on", (command_name, address)))
                release_event.wait()
                self.client.write_single_input(address, False)
                self.results.put(("momentary_done", (command_name, address)))
            except Exception as exc:
                recovery_error = None
                recovered_off = False
                if settings is not None:
                    recovery = ModbusTcpClient()
                    try:
                        recovery.connect(settings)
                        recovery.write_single_input(address, False)
                        recovered_off = True
                    except Exception as recovery_exc:
                        recovery_error = recovery_exc
                    finally:
                        recovery.disconnect()
                self.results.put(
                    (
                        "momentary_error",
                        (command_name, address, exc, recovered_off, recovery_error),
                    )
                )

        threading.Thread(target=command, daemon=True).start()
        return None

    def _momentary_release(self, _event, command_name: str):
        if self._active_momentary and self._active_momentary[0] == command_name:
            self._queue_momentary_release()
        return None

    def _global_button_release(self, _event=None) -> None:
        if self._active_momentary is not None:
            self._queue_momentary_release()

    def _escape_release(self, _event=None):
        if self._active_momentary is not None:
            self._queue_momentary_release()
            return "break"
        return None

    def _queue_momentary_release(self) -> None:
        if not self._momentary_release_event.is_set() and self._active_momentary:
            name, address = self._active_momentary
            self._log(f"BUTTON RELEASED: {name}")
            self._log(f"FC05 OFF queued for address {address}")
            self._momentary_release_event.set()

    def _poll_results(self) -> None:
        try:
            while True:
                action, result = self.results.get_nowait()
                if action == "connect":
                    self._working = False
                    self.status_var.set("Connected")
                    self.connect_button.configure(state="disabled")
                    self.disconnect_button.configure(state="normal")
                    self._set_operation_buttons("normal")
                    self._refresh_momentary_buttons()
                    self._set_monitor_controls()
                    self._log("Connected successfully")
                elif action == "read":
                    self._working = False
                    function_code, address, bits = result
                    values = ", ".join(
                        f"{address + index}={'ON' if value else 'OFF'}"
                        for index, value in enumerate(bits)
                    )
                    self._log(f"RX FC{function_code:02d}: {values}")
                    self._set_operation_buttons("normal")
                    self._refresh_momentary_buttons()
                elif action == "write":
                    self._working = False
                    name, address, state = result
                    self._log(f"RX FC05: {name} write confirmed, address {address} = {state}")
                    self._set_operation_buttons("normal")
                    self._refresh_momentary_buttons()
                elif action == "momentary_on":
                    name, address = result
                    self._log(
                        f"RX FC05: {name} ON acknowledged at address {address}; "
                        "holding value 1 until button release"
                    )
                elif action == "momentary_done":
                    name, address = result
                    self._log(f"RX FC05: {name} OFF acknowledged at address {address}")
                    self._active_momentary = None
                    self._working = False
                    self._set_operation_buttons("normal")
                    self._refresh_momentary_buttons()
                    if self._pending_disconnect:
                        self._disconnect_now()
                    if self._pending_close:
                        self.client.disconnect()
                        self.destroy()
                        return
                elif action == "momentary_error":
                    name, address, exc, recovered_off, recovery_error = result
                    self._log(
                        f"ERROR during momentary {name} at address {address}: "
                        f"{type(exc).__name__}: {exc}"
                    )
                    if recovered_off:
                        self._log("SAFETY RECOVERY: OFF was confirmed using a new TCP connection")
                    else:
                        self._log(
                            "WARNING: OFF could not be confirmed. Check the panel immediately. "
                            f"Recovery error: {recovery_error}"
                        )
                    self._active_momentary = None
                    self._working = False
                    self.commands_armed_var.set(False)
                    self._stop_monitoring(clear_leds=True, log=False)
                    self.client.disconnect()
                    self._set_monitor_controls()
                    self.status_var.set("Disconnected after command communication error")
                    self.connect_button.configure(state="normal")
                    self.disconnect_button.configure(state="disabled")
                    self._set_operation_buttons("disabled")
                    self._refresh_momentary_buttons()
                    if self._pending_disconnect:
                        self._disconnect_now()
                    if self._pending_close:
                        self.client.disconnect()
                        self.destroy()
                        return
                elif action == "monitor_update":
                    generation, timestamp, bits = result
                    if generation == self._monitor_generation and self._monitoring:
                        self._apply_monitor_values(timestamp, bits)
                elif action == "monitor_error":
                    generation, exc = result
                    if generation == self._monitor_generation and self._monitoring:
                        self._show_monitor_error(exc)
                elif action == "error":
                    self._working = False
                    source_action, exc = result
                    self._log(f"ERROR during {source_action}: {type(exc).__name__}: {exc}")
                    if source_action == "connect":
                        self.client.disconnect()
                        self.status_var.set("Connection failed")
                        self.connect_button.configure(state="normal")
                        self.disconnect_button.configure(state="disabled")
                        self._set_operation_buttons("disabled")
                        self._set_monitor_controls()
                    else:
                        self._set_operation_buttons(
                            "normal" if self.client.connected else "disabled"
                        )
                    self._refresh_momentary_buttons()
        except queue.Empty:
            pass
        self.after(100, self._poll_results)

    def _set_operation_buttons(self, state: str) -> None:
        for button in self.operation_buttons:
            button.configure(state=state)

    def _refresh_momentary_buttons(self) -> None:
        enabled = self.client.connected and self.commands_armed_var.get()
        active_name = self._active_momentary[0] if self._active_momentary else None
        for name, button in self.momentary_buttons.items():
            if active_name is not None:
                button.configure(state="normal" if name == active_name else "disabled")
            else:
                button.configure(state="normal" if enabled and not self._working else "disabled")

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _on_close(self) -> None:
        if self._active_momentary is not None:
            self._pending_close = True
            self.status_var.set("Releasing command before closing...")
            self._momentary_release_event.set()
            self._log("Close requested: releasing the active command first")
            return
        self._stop_monitoring(clear_leds=False, log=False)
        self.client.disconnect()
        self.destroy()


if __name__ == "__main__":
    ModbusApp().mainloop()
