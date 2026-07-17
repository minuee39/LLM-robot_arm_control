import queue
import socket
import socketserver
import threading


DEFAULT_COMMAND_HOST = "127.0.0.1"
DEFAULT_COMMAND_PORT = 5055


class _CommandRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        raw_line = self.rfile.readline()
        command = raw_line.decode("utf-8", errors="replace").strip()
        if not command:
            self.wfile.write("ERR empty command\n".encode("utf-8"))
            return

        self.server.command_queue.put(command)
        self.wfile.write("OK queued\n".encode("utf-8"))


class _ThreadedCommandServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, handler_class, command_queue):
        self.command_queue = command_queue
        super().__init__(server_address, handler_class)


class RobotCommandServer:
    def __init__(self, host: str = DEFAULT_COMMAND_HOST, port: int = DEFAULT_COMMAND_PORT) -> None:
        self.host = host
        self.port = port
        self._queue: queue.Queue[str] = queue.Queue()
        self._server = _ThreadedCommandServer((host, port), _CommandRequestHandler, self._queue)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    @property
    def address(self) -> tuple[str, int]:
        return self._server.server_address

    def start(self) -> None:
        self._thread.start()

    def get_next_command(self) -> str | None:
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def shutdown(self) -> None:
        self._server.shutdown()
        self._server.server_close()


def send_command(command: str, host: str = DEFAULT_COMMAND_HOST, port: int = DEFAULT_COMMAND_PORT) -> str:
    with socket.create_connection((host, port), timeout=5.0) as sock:
        sock.sendall((command.strip() + "\n").encode("utf-8"))
        return sock.recv(1024).decode("utf-8", errors="replace").strip()
