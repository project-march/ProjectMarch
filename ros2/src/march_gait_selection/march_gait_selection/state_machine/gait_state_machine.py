import rclpy
from rclpy.node import Node
from .gait_state_machine_error import GaitStateMachineError
from .home_gait import HomeGait


class GaitStateMachine(object):
    UNKNOWN = 'unknown'

    def __init__(self, gait_selection, trajectory_scheduler, state_input, update_rate):
        """Generates a state machine from given gaits and resets it to UNKNOWN state.

        In order to start the state machine see `run`.

        :param GaitSelection gait_selection: Selection of loaded gaits to build from
        :param TrajectoryScheduler trajectory_scheduler: Scheduler interface for scheduling trajectories
        :param StateMachineInput state_input: Input interface for controlling the states
        :param float update_rate: update rate in Hz
        """

        super(GaitStateMachine, self).__init__('gait_state_machine')
        self._gait_selection = gait_selection
        self._trajectory_scheduler = trajectory_scheduler
        self._input = state_input
        self._update_rate = update_rate

        self._home_gaits = {}
        self._idle_transitions = {}
        self._gait_transitions = {}
        self._generate_graph()

        self._transition_callbacks = []
        self._gait_callbacks = []
        self._stop_accepted_callbacks = []

        self._current_state = self.UNKNOWN
        self._current_gait = None
        self._is_idle = True
        self._shutdown_requested = False
        self._should_stop = False

    def create_publishers(self):
        self.current_state_pub = self._gate_selection.create_publisher(CurrentState, '/march/gait_selection/current_state', 10)
        self.current_gait_pub = self._gate_selection.create_publisher(CurrentGait, '/march/gait_selection/current_gait', 10)

        def current_state_cb(state, next_is_idle):
            self.current_state_pub.publish(CurrentState(header=Header(stamp=self._gate_selection.get_clock().now().to_msg()),
                                                        state=state,
                                                        state_type=CurrentState.IDLE if next_is_idle else CurrentState.GAIT)

        def current_gait_cb(gait_name, subgait_name, version, duration, gait_type):
            current_gait_pub.publish(CurrentGait(header=Header(stamp=self.get_clock().now().to_msg()), gait=gait_name,
                                                 subgait=subgait_name, version=version,
                                                 duration=rclpy.Duration(seconds=duration), gait_type=gait_type)

        self.add_transition_callback(current_state_cb)
        self.add_gait_callback(current_gait_cb)

    def create_subscriptions(self):
        self._gate_selection.create_subscription(Error, '/march/error', lambda msg: self.error_cb(msg))

    def create_services(self):
        self._gate_selection.create_service(PossibleGaits, '/march/gait_selection/get_possible_gaits',
                                      lambda msg: PossibleGaits.Response(gaits=self.get_possible_gaits()))

    def create_sounds(self):
        if self.get_parameter_or('~sounds', alternative_value=False):
            sounds = Sounds(['start', 'gait_start', 'gait_end', 'gait_stop'])

            def play_gait_sound(_state, next_is_idle):
                if next_is_idle:
                    sounds.play('gait_end')
                else:
                    sounds.play('gait_start')

            self.add_transition_callback(play_gait_sound)
            self.add_stop_accepted_callback(lambda: sounds.play('gait_stop'))

            # Short sleep is necessary to wait for the sound topic to initialize
            sleep(0.5)
            sounds.play('start')

    def error_cb(self, msg):
        if msg.type == Error.NON_FATAL:
            self.stop()
        elif msg.type == Error.FATAL:
            self.request_shutdown()

    def get_possible_gaits(self):
        """Returns possible names of gaits that can be executed.

        :returns List of names, or empty list when a gait is executing.
        """
        if self._is_idle:
            return list(self._idle_transitions[self._current_state])
        else:
            return []

    def add_transition_callback(self, cb):
        """Adds a callback function that will be called when a transition to a state happens.

        The given method should be running as shortly as possible, since they
        will be called from within the main loop.

        :param cb: method that accepts a name of the state and a boolean if it is an idle state.
        """
        self._add_callback(self._transition_callbacks, cb)

    def add_gait_callback(self, cb):
        """Adds a callback function that will be called when a trajectory of a gait is being scheduled.

        The given method should be running as shortly as possible, since they
        will be called from within the main loop.

        :param cb: Callable method that accepts 5 args: gait name, subgait name, version, duration and gait type.
        """
        self._add_callback(self._gait_callbacks, cb)

    def add_stop_accepted_callback(self, cb):
        """Adds a callback function that will be called when a gait accepts the stop command.

        The given method should be running as shortly as possible, since they
        will be called from within the main loop.

        :param cb: Callable method that accepts no arguments and returns None.
        """
        self._add_callback(self._stop_accepted_callbacks, cb)

    def run(self):
        """Runs the state machine until shutdown is requested."""
        rate = self._gait_selection.create_rate(self._update_rate)
        last_update_time = self._gait_selection.get_clock().now()
        while not self._shutdown_requested:
            now = self._gait_seleciton.get_clock().now()
            elapsed_time = now - last_update_time
            last_update_time = now
            if self._is_idle:
                self._process_idle_state()
            else:
                self._process_gait_state(elapsed_time.to_sec())
            rate.sleep()

    def request_shutdown(self):
        """Requests shutdown, which will terminate the state machine as soon as possible."""
        self._shutdown_requested = True

    def stop(self):
        """Requests a stop from the current executing gait, but keeps the state machine running."""
        if not self._is_idle:
            self._should_stop = True

    def _process_idle_state(self):
        if self._input.gait_requested():
            gait_name = self._input.gait_name()
            self._gait_selection.get_logger().info('Requested gait `{0}`'.format(gait_name))
            if gait_name in self._idle_transitions[self._current_state]:
                self._current_state = gait_name
                self._is_idle = False
                self._should_stop = False
                self._input.gait_accepted()
                self._call_transition_callbacks()
                self._gait_selection.get_logger().info('Accepted gait `{0}`'.format(gait_name))
            else:
                self._input.gait_rejected()
                self._gait_selection.get_logger().info('Cannot execute gait `{0}` from idle state `{1}`'.format(gait_name,
                                                                                                self._current_state))
        elif self._input.unknown_requested():
            self._input.gait_accepted()
            self._transition_to_unknown()
            self._input.gait_finished()
            self._call_transition_callbacks()

    def _process_gait_state(self, elapsed_time):
        if self._current_gait is None:
            if self._current_state in self._home_gaits:
                self._current_gait = self._home_gaits[self._current_state]
            else:
                self._current_gait = self._gait_selection[self._current_state]
            self._gait_selection.get_logger().info('Executing gait `{0}`'.format(self._current_gait.name))
            trajectory = self._current_gait.start()
            if trajectory is not None:
                self._call_gait_callbacks()
                self._gait_selection.get_logger().info('Scheduling {subgait}'.format(subgait=self._current_gait.subgait_name))
                self._trajectory_scheduler.schedule(trajectory)
            elapsed_time = 0.0

        if self._trajectory_scheduler.failed():
            self._trajectory_scheduler.reset()
            self._current_gait.end()
            self._current_gait = None
            self._transition_to_unknown()
            self._input.gait_finished()
            return

        self._handle_input()

        trajectory, should_stop = self._current_gait.update(elapsed_time)
        # schedule trajectory if any
        if trajectory is not None:
            self._call_gait_callbacks()
            self._gait_selection.get_logger().loginfo('Scheduling {subgait}'.format(subgait=self._current_gait.subgait_name))
            self._trajectory_scheduler.schedule(trajectory)

        if should_stop:
            self._current_state = self._gait_transitions[self._current_state]
            self._is_idle = True
            self._current_gait.end()
            self._input.gait_finished()
            self._call_transition_callbacks()
            self._gait_selection.get_logger().info('Finished gait `{0}`'.format(self._current_gait.name))
            self._current_gait = None

    def _handle_input(self):
        if self._input.stop_requested() or self._should_stop:
            self._should_stop = False
            if self._current_gait.stop():
                self._gait_selection.get_logger().info('Gait `{0}` responded to stop'.format(self._current_gait.name))
                self._input.stop_accepted()
                self._call_callbacks(self._stop_accepted_callbacks)
            else:
                self._gait_selection.get_logger.info('Gait `{0}` does not respond to stop'.format(self._current_gait.name))
                self._input.stop_rejected()

        if self._input.transition_requested():
            request = self._input.get_transition_request()
            self._input.reset()
            if self._current_gait.transition(request):
                self._gait_selection.get_logger().info('Gait `{0}` responded to transition request `{1}`'.format(self._current_gait.name,
                                                                                                 request.name))
            else:
                self._gait_selection.get_logger().info('Gait `{0}` does not respond to transition request `{1}`'.format(self._current_gait.name,
                                                                                                        request.name))

    def _call_transition_callbacks(self):
        self._call_callbacks(self._transition_callbacks, self._current_state, self._is_idle)

    def _call_gait_callbacks(self):
        if self._current_gait is not None:
            self._call_callbacks(self._gait_callbacks, self._current_gait.name, self._current_gait.subgait_name,
                                 self._current_gait.version, self._current_gait.duration, self._current_gait.gait_type)

    def _transition_to_unknown(self):
        self._current_state = self.UNKNOWN
        self._is_idle = True
        self._gait_selection.get_logger().info('Transitioned to `{0}`'.format(self.UNKNOWN))

    def _generate_graph(self):
        self._idle_transitions = {}
        self._gait_transitions = {}
        idle_positions = self._gait_selection.positions
        for gait in self._gait_selection:
            gait_name = gait.name
            starting_position = gait.starting_position
            from_idle_name = next(
                (name for name, position in idle_positions.items() if position['joints'] == starting_position), None)
            if from_idle_name is None:
                from_idle_name = 'unknown_idle_{0}'.format(len(idle_positions))
                self._gait_selection.get_logger().warn('No named position given for starting position of gait `{gn}`, creating `{n}`'
                                       .format(gn=gait_name, n=from_idle_name))
                idle_positions[from_idle_name] = {'gait_type': '', 'joints': starting_position}
            if from_idle_name in self._idle_transitions:
                self._idle_transitions[from_idle_name].add(gait_name)
            else:
                self._idle_transitions[from_idle_name] = {gait_name}

            final_position = gait.final_position
            to_idle_name = next(
                (name for name, position in idle_positions.items() if position['joints'] == final_position), None)
            if to_idle_name is None:
                to_idle_name = 'unknown_idle_{0}'.format(len(idle_positions))
                self._gait_selection.get_logger().warn(
                    'No named position given for final position of gait `{gn}`, creating `{n}`'
                    .format(gn=gait_name, n=to_idle_name))
                idle_positions[to_idle_name] = {'gait_type': '', 'joints': final_position}
            self._gait_transitions[gait_name] = to_idle_name

        self._validate_transitions()

        self._generate_home_gaits(idle_positions)

    def _validate_transitions(self):
        for idle in self._gait_transitions.values():
            if idle not in self._idle_transitions:
                self._gait_selection.get_logger().warn('{0} does not have transitions'.format(idle))

    def _generate_home_gaits(self, idle_positions):
        self._idle_transitions[self.UNKNOWN] = set()
        self._home_gaits = {}
        for idle_name, position in idle_positions.items():
            home_gait = HomeGait(idle_name, position['joints'], position['gait_type'])
            home_gait_name = home_gait.name
            self._home_gaits[home_gait_name] = home_gait
            if home_gait_name in self._gait_transitions:
                raise GaitStateMachineError('Gaits cannot have the same name as home gait `{0}`'.format(home_gait_name))
            self._gait_transitions[home_gait_name] = idle_name
            self._idle_transitions[self.UNKNOWN].add(home_gait_name)

    @staticmethod
    def _add_callback(callbacks, cb):
        """Adds a method to a list if it is callable."""
        if callable(cb):
            callbacks.append(cb)

    @staticmethod
    def _call_callbacks(callbacks, *args):
        """Calls multiple methods with same set of arguments."""
        for cb in callbacks:
            cb(*args)
