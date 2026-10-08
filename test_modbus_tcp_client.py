import socket
import struct
import threading
import unittest

from modbus_tcp_client import (
    ConnectionSettings,
    MONITOR_PROTOCOL_START,
    MONITOR_QUANTITY,
    MONITOR_REFERENCE_END,
    MONITOR_REFERENCE_START,
    PANEL_STATUS_POINTS,
    ModbusTcpClient,
    decode_panel_status_bits,
    fc02_reference_to_offset,
    panel_bases,
    panel_command_address,
)


class FakeModbusServer:
    def __init__(self, request_count=3):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.request_count = request_count
        self.requests = []
        self.thread = threading.Thread(target=self._serve, daemon=True)

    def start(self):
        self.thread.start()

    @staticmethod
    def _recv_exact(conn, size):
        data = b""
        while len(data) < size:
            chunk = conn.recv(size - len(data))
            if not chunk:
                return b""
            data += chunk
        return data

    @staticmethod
    def _pack_bits(values):
        encoded = bytearray((len(values) + 7) // 8)
        for index, value in enumerate(values):
            if value:
                encoded[index // 8] |= 1 << (index % 8)
        return bytes(encoded)

    def _serve(self):
        conn, _ = self.listener.accept()
        with conn:
            for _ in range(self.request_count):
                header = self._recv_exact(conn, 7)
                if not header:
                    return
                tx, protocol, length, unit = struct.unpack(">HHHB", header)
                pdu = self._recv_exact(conn, length - 1)
                self.requests.append(pdu)
                if pdu[0] == 1:
                    response_pdu = bytes([1, 2, 0b01001101, 0b00000001])
                elif pdu[0] == 2:
                    start, quantity = struct.unpack(">HH", pdu[1:5])
                    if (start, quantity) == (200, 6):
                        values = [False, True, True, False, True, True]
                    else:
                        active_offsets = {0, 2, 20, 46}
                        values = [start + index in active_offsets for index in range(quantity)]
                    encoded = self._pack_bits(values)
                    response_pdu = bytes([2, len(encoded)]) + encoded
                else:
                    response_pdu = pdu
                response_header = struct.pack(
                    ">HHHB", tx, protocol, len(response_pdu) + 1, unit
                )
                conn.sendall(response_header + response_pdu)

    def close(self):
        self.listener.close()


class AddressingTests(unittest.TestCase):
    def test_standard_bases_and_commands(self):
        self.assertEqual(panel_bases("standard", 99), (0, 0, 512))
        self.assertEqual(panel_command_address("standard", 1, "Silence Panel"), 512)
        self.assertEqual(panel_command_address("standard", 1, "Silence Sounder"), 513)
        self.assertEqual(panel_command_address("standard", 1, "Reset Panel"), 514)
        self.assertEqual(panel_command_address("standard", 1, "Evacuate"), 515)

    def test_extended_bases_and_commands(self):
        self.assertEqual(panel_bases("extended", 1), (0, 512, 1024))
        self.assertEqual(panel_bases("extended", 3), (4000, 4512, 5024))
        self.assertEqual(panel_command_address("extended", 5, "Reset Panel"), 9026)

    def test_extended_card_address_validation(self):
        with self.assertRaises(ValueError):
            panel_bases("extended", 6)

    def test_fc02_reference_translation(self):
        self.assertEqual(fc02_reference_to_offset(10001), 0)
        self.assertEqual(fc02_reference_to_offset(10002), 1)
        self.assertEqual(fc02_reference_to_offset(10047), 46)
        with self.assertRaises(ValueError):
            fc02_reference_to_offset(10000)

    def test_panel_status_mapping(self):
        expected_references = {
            *range(10001, 10016),
            *range(10021, 10027),
            *range(10032, 10048),
        }
        self.assertEqual(len(PANEL_STATUS_POINTS), 37)
        self.assertEqual(
            {point.reference for point in PANEL_STATUS_POINTS}, expected_references
        )
        self.assertEqual(MONITOR_REFERENCE_START, 10001)
        self.assertEqual(MONITOR_REFERENCE_END, 10047)
        self.assertEqual(MONITOR_PROTOCOL_START, 0)
        self.assertEqual(MONITOR_QUANTITY, 47)
        self.assertEqual(len({point.description for point in PANEL_STATUS_POINTS}), 37)

    def test_panel_status_block_decoding_ignores_gaps(self):
        bits = [False] * MONITOR_QUANTITY
        bits[0] = True
        bits[15] = True  # Reference 10016 is an unused gap.
        bits[20] = True
        bits[46] = True
        decoded = decode_panel_status_bits(bits)
        self.assertTrue(decoded[10001])
        self.assertTrue(decoded[10021])
        self.assertTrue(decoded[10047])
        self.assertNotIn(10016, decoded)
        with self.assertRaises(ValueError):
            decode_panel_status_bits([False] * 46)


class ModbusClientTests(unittest.TestCase):
    def test_fc01_fc02_and_fc05(self):
        server = FakeModbusServer()
        server.start()
        client = ModbusTcpClient()
        try:
            client.connect(ConnectionSettings("127.0.0.1", server.port, 7, 1.0))
            inputs = client.read_input_variables(100, 9)
            self.assertEqual(
                inputs,
                [True, False, True, True, False, False, True, False, True],
            )
            outputs = client.read_output_variables(200, 6)
            self.assertEqual(outputs, [False, True, True, False, True, True])
            client.write_single_input(514, True)
            self.assertEqual(server.requests[0], bytes([1]) + struct.pack(">HH", 100, 9))
            self.assertEqual(server.requests[1], bytes([2]) + struct.pack(">HH", 200, 6))
            self.assertEqual(server.requests[2], bytes([5]) + struct.pack(">HH", 514, 0xFF00))
        finally:
            client.disconnect()
            server.close()

    def test_momentary_on_then_off_frames(self):
        server = FakeModbusServer(request_count=2)
        server.start()
        client = ModbusTcpClient()
        try:
            client.connect(ConnectionSettings("127.0.0.1", server.port, 1, 1.0))
            client.write_single_input(515, True)
            client.write_single_input(515, False)
            self.assertEqual(
                server.requests,
                [
                    bytes([5]) + struct.pack(">HH", 515, 0xFF00),
                    bytes([5]) + struct.pack(">HH", 515, 0x0000),
                ],
            )
        finally:
            client.disconnect()
            server.close()

    def test_fc02_reference_block_for_status_monitoring(self):
        server = FakeModbusServer(request_count=1)
        server.start()
        client = ModbusTcpClient()
        try:
            client.connect(ConnectionSettings("127.0.0.1", server.port, 2, 1.0))
            values = client.read_discrete_references(
                MONITOR_REFERENCE_START, MONITOR_QUANTITY
            )
            self.assertEqual(len(values), 47)
            self.assertTrue(values[0])
            self.assertTrue(values[2])
            self.assertTrue(values[20])
            self.assertTrue(values[46])
            self.assertEqual(
                server.requests[0],
                bytes([2]) + struct.pack(">HH", MONITOR_PROTOCOL_START, MONITOR_QUANTITY),
            )
        finally:
            client.disconnect()
            server.close()


if __name__ == "__main__":
    unittest.main()
