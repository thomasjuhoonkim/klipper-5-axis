import logging
import math
import stepper


class CoreXYRotUWKinematics:
    def __init__(self, toolhead, config):
        # Setup translation axis rails
        self.rails = [
            stepper.LookupMultiRail(config.getsection("stepper_" + n)) for n in "xyz"
        ]
        # Setup rotation axes u and w
        self.rotors = [
            stepper.LookupMultiRail(config.getsection("stepper_" + n)) for n in "uw"
        ]

        # something related to adding endstops on x and y
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)

        # c memory injection for xyzuw
        self.rails[0].setup_itersolve("corexy_stepper_alloc", b"+")
        self.rails[1].setup_itersolve("corexy_stepper_alloc", b"-")
        self.rails[2].setup_itersolve("cartesian_stepper_alloc", b"z")
        self.rotors[0].setup_itersolve("rotuw_stepper_alloc", b"u")
        self.rotors[1].setup_itersolve("rotuw_stepper_alloc", b"w")

        # event handler for steppers & emergency stop
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )

        # setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", max_velocity, above=0.0, maxval=max_velocity
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", max_accel, above=0.0, maxval=max_accel
        )

        # normalized arbitrary limits for xyzuw
        self.limits = [(1.0, -1.0)] * 5

        # axes min and max for xyzuw
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.0)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.0)

    # get all steppers
    def get_steppers(self):
        return [s for rail in self.rails + self.rotors for s in rail.get_steppers()]

    # idk yet
    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails + self.rotors]
        return [
            0.5 * (pos[0] + pos[1]),
            0.5 * (pos[0] - pos[1]),
            pos[2],
            pos[3],
            pos[4],
        ]

    # set positions of all axes to new position + optionally home?
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails + self.rotors):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    # something
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)

    # homing
    def home(self, homing_state):
        # Each axis is homed independently and in order
        # extend homing to rails and rotors
        all_systems = self.rails + self.rotors
        for system in homing_state.get_axes():
            system_to_home = all_systems[system]
            # Determine movement
            position_min, position_max = system_to_home.get_range()
            hi = system_to_home.get_homing_info()
            homepos = [None, None, None, None, None, None]
            homepos[system] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[system] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[system] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([system_to_home], forcepos, homepos)

    # shut off motors by defining limits? (internal)
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 5

    # boundary check, fails if not homed yet (internal)
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in range(5):
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0] or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    # boundary check for move to be done
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < limits[0][0]
            or xpos > limits[0][1]
            or ypos < limits[1][0]
            or ypos > limits[1][1]
        ):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    # current status, nothing special
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyzuw", self.limits) if l <= h]
        return {
            "homed_axes": "".join(axes),
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
        }


def load_kinematics(toolhead, config):
    return CoreXYRotUWKinematics(toolhead, config)
