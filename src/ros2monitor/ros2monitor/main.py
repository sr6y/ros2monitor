import signal
from queue import Queue
from threading import Barrier
from threading import Thread

import rclpy
from ros2monitor.nodemanager import NodeManager
from ros2monitor.cli import Cli



def signal_handler(sig, frame):
    print('\nType "quit" to exit ros2monitor.\nPress "Enter" to continue ros2monitor.')

def main():
    rclpy.init()  # Init default context

    signal.signal(signal.SIGINT, signal_handler)  # Catch sigint signal

    safe_commands = Queue()  # Queue for sharing commands between threads
    command_barrier = Barrier(2)  # allows cli_thread wait for the other thread to complete the command to continue

    node_manager = NodeManager(safe_commands, command_barrier)  # NodeManager object
    cli = Cli(node_manager, safe_commands, command_barrier)  # CLI object

    cli_thread = Thread(target=cli.cmdloop)  # CLI thread
    spin_thread = Thread(target=rclpy.spin, args=[node_manager])  # rclpy spin thread

    cli_thread.start()
    spin_thread.start()
    # main thread wait for other threads to finish

if __name__ == '__main__':
    main()
