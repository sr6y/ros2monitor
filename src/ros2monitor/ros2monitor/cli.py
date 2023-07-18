import cmd
import rclpy


# Cmd subclass
class Cli(cmd.Cmd):
    def __init__(self, node_manager, safe_commands,
                 command_barrier):  # Generate the attributes that objects that are created should have
        cmd.Cmd.__init__(self)
        self.intro = 'Welcome to ros2monitor!\nType help or ? to list commands.'
        self.prompt = 'ros2monitor> '
        self.doc_header = 'Type "help command" to find out more about the command "command".\n' \
                          'Type "Ctrl+L" to clear-screen.'

        self.node_manager = node_manager
        self.safe_commands = safe_commands
        self.command_barrier = command_barrier

    # prevents that the last command entered from being repeated when the line is empty
    def emptyline(self):
        pass

    # default error message
    def default(self, line):
        self.stdout.write('%s: Unknown command\n' % (line,))

    # CLI commands
    def do_topiclist(self, args):
        """Shows available topics."""
        self.safe_commands.put(['topiclist'])
        self.command_barrier.wait()

    def do_monitoredtopics(self, args):
        """Shows topics that are being monitored."""
        self.safe_commands.put(['monitoredtopics'])
        self.command_barrier.wait()

    def do_multiple(self, args):
        """Monitor all available topics."""
        self.safe_commands.put(['multiple'])
        self.command_barrier.wait()

    def do_select(self, args):
        """Monitor one or more topics.
        Usage:
        select <topic>  Monitor one topic.
        select <topic> <topic>...    Monitor topic list."""
        self.safe_commands.put(['select', args])
        self.command_barrier.wait()

    def do_stop(self, args):
        """Stop monitoring topics.
        Usage: stop all   Stop monitoring all topics.
        stop <topic>   Stop monitoring one.
        stop <topic> <topic>...    Stop monitoring topic list."""
        self.safe_commands.put(['stop', args])
        self.command_barrier.wait()

    def do_logs(self, args):
        """Shows the logs of possible threats detected"""
        with open("src/ros2monitor/ros2monitor/logs/output.txt") as file:
            # read the file line by line
            line = file.readline()
            while line:
                print(line)
                line = file.readline()

    def do_quit(self, args):
        """Quit ros2monitor"""
        self.node_manager.destroy_node()  # Destroy node
        rclpy.try_shutdown()  # Shutdown default context
        return True

    def do_help(self, args):
        """List available commands with "help" or detailed help with "help command"."""
        cmd.Cmd.do_help(self, args)
