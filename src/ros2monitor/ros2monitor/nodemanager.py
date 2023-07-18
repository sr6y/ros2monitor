import yara
import functools
from datetime import datetime
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


# Node Manager class
class NodeManager(Node):
    def __init__(self, safe_commands, command_barrier):
        super().__init__('node_manager')
        self.topics_subscriptions = dict()
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.command_callback)
        self.rule = yara.compile('src/ros2monitor/ros2monitor/rules/rules.yar')  # yara rules file
        self.file = open('src/ros2monitor/ros2monitor/logs/output.txt', 'a')    # logs file
        self.safe_commands = safe_commands
        self.command_barrier = command_barrier

    # Subscribe to topic
    def subscribe(self, topic):
        # check if exist a subscription for this topic
        if topic[0] in self.topics_subscriptions:
            print('%s is already monitoring' % topic[0])
        else:
            msg_type = get_message(topic[1][0])
            subscription = self.create_subscription(
                msg_type,
                topic[0],
                functools.partial(self.listener_callback, topic=topic[0]),  # Topic value in listener_callback
                10,  # qos_profile
                raw=True  # raw binary representation
            )
            self.topics_subscriptions[topic[0]] = subscription  # save subscription
            print('Monitoring %s' % topic[0])

    # Unsubscribe a topic
    def unsubscribe(self, topic):
        if topic == 'all':
            if self.topics_subscriptions:
                # destroy all subscriptions
                for value in self.topics_subscriptions.values():
                    self.destroy_subscription(value)
                self.topics_subscriptions.clear()
                print('All topics stopped being monitored')
            else:
                print('You are not monitoring any topic')
        else:
            # destroy subscription if exist
            if topic in self.topics_subscriptions:
                self.destroy_subscription(self.topics_subscriptions[topic])
                self.topics_subscriptions.pop(topic)
                print('%s has been stopped monitoring' % topic)
            else:
                print('%s is not being monitored' % topic)

    # Get available topics
    def get_topics(self):
        topics = list()
        topics_aux = self.get_topic_names_and_types()
        for topic in topics_aux:
            topics.append(topic)
        return topics

    # Get current subscriptions
    def get_subscriptions(self):
        subcriptions_aux = list()
        for k in self.topics_subscriptions.keys():
            subcriptions_aux.append(k)
        return subcriptions_aux

    # Commands
    # topiclist command
    def topiclist_command(self):
        topics = self.get_topics()  # Get available topic list
        if topics:
            print('Available topics:')
            for topic in topics:
                print(topic[0])
        else:
            print('No topics available')

    def monitoredtopics_command(self):
        subscriptions_aux = self.get_subscriptions()
        if subscriptions_aux:
            print('You are monitoring:')
            for sub in subscriptions_aux:
                print(sub)
        else:
            print('You are not monitoring any topic')

    def multiple_command(self):
        topics = self.get_topics()
        if topics:
            for topic in topics:
                self.subscribe(topic)
        else:
            print('No topics available')

    def select_command(self, args):
        if args:
            topics_aux = args.split()
            topics = self.get_topics()
            if topics:
                for t_name in topics_aux:
                    exist = False
                    for topic in topics:
                        if t_name == topic[0]:
                            exist = True
                            self.subscribe(topic)
                    if not exist:
                        print('%s does not exist or is not available' % t_name)
            else:
                print('No topics available to monitor')
        else:
            print('Enter the topic that you want to monitor')

    def stop_command(self, args):
        if args:
            topics_aux = args.split()
            for t_name in topics_aux:
                self.unsubscribe(t_name)
        else:
            print('Enter topic/s you want to stop monitoring')

    # Callbacks
    # receives commands and carries out appropriate actions
    def command_callback(self):
        if not self.safe_commands.empty():
            command = self.safe_commands.get()  # get and remove the command
            match command[0]:
                case 'topiclist':
                    self.topiclist_command()
                case 'monitoredtopics':
                    self.monitoredtopics_command()
                case 'multiple':
                    self.multiple_command()
                case 'select':
                    self.select_command(command[1])
                case 'stop':
                    self.stop_command(command[1])
            self.command_barrier.wait()

    # received messages are analyzed with yara
    def listener_callback(self, msg, topic):
        matches = self.rule.match(data=msg)
        if matches:
            output_string = '{}  {} in {} in message -> {}\n'.format(datetime.now(), matches, topic, msg)
            self.file.write(output_string)  # monitoring logs
            self.file.flush()
