from command_server import RobotCommandServer, send_command


def test_robot_command_server_queues_command():
    server = RobotCommandServer(port=0)
    server.start()
    host, port = server.address

    try:
        response = send_command("빨간 블럭을 파란 블럭 옆에 둬", host=host, port=port)

        assert response == "OK queued"
        assert server.get_next_command() == "빨간 블럭을 파란 블럭 옆에 둬"
    finally:
        server.shutdown()
