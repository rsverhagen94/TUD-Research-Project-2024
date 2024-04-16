import sys, random, enum, ast, time, threading, os, math
from datetime import datetime
from flask import jsonify
from rpy2 import robjects
from matrx import grid_world
from brains1.custom_agent_brain import custom_agent_brain
from utils1.util_functions import *
from actions1.custom_actions import *
from matrx import utils
from matrx.grid_world import GridWorld
from matrx.agents.agent_utils.state import State
from matrx.agents.agent_utils.navigator import Navigator
from matrx.agents.agent_utils.state_tracker import StateTracker
from matrx.actions.door_actions import OpenDoorAction
from matrx.actions.object_actions import GrabObject, DropObject, RemoveObject
from matrx.actions.move_actions import MoveNorth
from matrx.messages.message import Message
from matrx.messages.message_manager import MessageManager
from actions1.custom_actions import Backup, RemoveObjectTogether, CarryObjectTogether, DropObjectTogether, CarryObject, Drop, Injured, AddObject

class Phase(enum.Enum):
    # define possible phases
    INTRO_1 = 0,
    INTRO_2 = 20,
    INTRO_3 = 21,
    LOCATE = 1,
    FIND_NEXT_GOAL = 2,
    PICK_UNSEARCHED_ROOM = 3,
    PLAN_PATH_TO_ROOM = 4,
    FOLLOW_PATH_TO_ROOM = 5,
    REMOVE_OBSTACLE_IF_NEEDED = 6,
    ENTER_ROOM = 7,
    PLAN_ROOM_SEARCH_PATH = 8,
    FOLLOW_ROOM_SEARCH_PATH = 9,
    PLAN_PATH_TO_VICTIM = 10,
    FOLLOW_PATH_TO_VICTIM = 11,
    TAKE_VICTIM = 12,
    PLAN_PATH_TO_DROPPOINT = 13,
    FOLLOW_PATH_TO_DROPPOINT = 14,
    DROP_VICTIM = 15,
    TACTIC = 16,
    PRIORITY = 17,
    RESCUE = 18,
    EXTINGUISH_CHECK = 19

class tutorial_robot(custom_agent_brain):
    def __init__(self, name, condition, resistance, no_fires, victims, task, counterbalance_condition):
        super().__init__(name, condition, resistance, no_fires, victims, task, counterbalance_condition)
        # initialize important variables
        self._phase=Phase.FIND_NEXT_GOAL
        self._name = name
        self._condition = condition
        self._resistance = resistance
        self._time_left = resistance
        self._no_fires = no_fires
        self._victims = victims
        self._task = task
        self._counterbalance_condition = counterbalance_condition
        self._room_victims = []
        self._searched_rooms_defensive = []
        self._searched_rooms_offensive = []
        self._found_victims = []
        self._rescued_victims = []
        self._lost_victims = []
        self._modulos = []
        self._send_messages = []
        self._fire_locations = {}
        self._extinguished_fire_locations = []
        self._room_tiles = []
        self._situations = []
        self._plot_times = []
        self._potential_source_offices = []
        self._victim_locations = {}
        self._office_doors = {(2, 3): '1', (8, 3): '2', (2, 9): '3', (8, 9): '4'}
        self._decided_time = None
        self._current_door = None
        self._current_room = None
        self._goal_victim = None
        self._goal_location = None
        self._id = None
        self._fire_source_coords = None
        self._deploy_time = None
        self._current_location = None
        self._situation = None
        self._plot_generated = False
        self._reallocated = False
        self._decided = False
        self._waiting = False
        self._evacuating = False
        self._started = False
        self._smoke = '?'
        self._location = '?'
        self._distance = '?'
        self._tactic = 'offensive'
        self._offensive_deployment_time = 0
        self._defensive_deployment_time = 0
        self._offensive_search_rounds = 0
        self._interventions = 1

    def initialize(self):
        # initialize state tracker and navigator
        self._state_tracker = StateTracker(agent_id = self.agent_id)
        self._navigator = Navigator(agent_id = self.agent_id, action_set = self.action_set, algorithm = Navigator.A_STAR_ALGORITHM)
        # load all required R libraries that will be launched from within Python
        load_R_to_Py()

    def filter_bw4t_observations(self, state):
        # calculate the number of seconds passed
        self._second = state['World']['tick_duration'] * state['World']['nr_ticks']
        # if 6 seconds passed, resistance to collapse decreases with 1 minute
        if int(self._second) % 6 == 0 and int(self._second) not in self._modulos and self._resistance >= 0:
            self._modulos.append(int(self._second))
            self._resistance -= 1
            # keep track of the duration of deployment tactics
            if self._tactic == 'offensive':
                self._offensive_deployment_time += 1
            if self._tactic == 'defensive':
                self._defensive_deployment_time += 1
        # send hidden messages used for GUI/display with icons
        self._send_message("Time left: " + str(self._resistance) + ".", self._name)
        return state

    def decide_on_bw4t_action(self, state:State):
        print(self._phase)
        self._current_location = state[self.agent_id]['location']

        # send hidden message used for logging counterbalance condition, robot name, and allocation threshold
        self._send_message("Counterbalancing condition " + self._counterbalance_condition + " name " + self._name + " threshold " + str(4.1), self._name)

        # switch to offensive tactic when temperature is lower than threshold and people are found but not rescued
        if self._tactic == 'defensive' and self._temperature != '>' and len(self._rescued_victims) != len(self._found_victims) and len(self._found_victims) == self._total_victims and not self._waiting:
            self._send_message("Switching to an offensive deployment after the defensive deployment of " + str(self._defensive_deployment_time) + " minutes, \
                                because the temperature is no longer higher than the safety threshold and there are still victims that we found but did not rescue.", self._name)
            self._waiting = True
            self._decided_time = int(self._second)
        
        if self._tactic == 'defensive' and self._temperature != '>' and len(self._rescued_victims) != len(self._found_victims) and len(self._found_victims) == self._total_victims and self._decided_time and int(self._second) < self._decided_time + 5:
            return None, {}

        if self._tactic == 'defensive' and self._temperature != '>' and len(self._rescued_victims) != len(self._found_victims) and len(self._found_victims) == self._total_victims and self._decided_time and int(self._second) >= self._decided_time + 5:
            self._tactic = 'offensive'
            self._waiting = False
            self._offensive_search_rounds += 1
            self._lost_victims = []
            self._send_messages = []
            self.received_messages = []
            self.received_messages_content = []
            self._phase = Phase.FIND_NEXT_GOAL

        # inspect the state and save important information such as location of area tiles, fires, fire source, smoke, and speed of smoke spread
        for info in state.values():
            if 'class_inheritance' in info and 'AreaTile' in info['class_inheritance'] and info['location'] not in self._room_tiles:
                self._room_tiles.append(info['location'])
            if 'class_inheritance' in info and 'FireObject' in info['class_inheritance'] and 'source' in info['obj_id'] and self._phase == Phase.FOLLOW_ROOM_SEARCH_PATH:
                if not self._fire_source_coords:
                    self._send_message("Found fire source in " + self._current_room + "!", self._name)
                    self._fire_source_coords = info['location']
                    self._fire_locations[self._current_room] = info['location']
                    action_kwargs = add_object([info['location']], "/images/fire2.svg", 2, 1, 'fire source in ' + self._current_room)
                    return AddObject.__name__, action_kwargs
                self._location = 'found'
                self._smoke = info['smoke']
                if self._tactic == 'defensive':
                    self._phase = Phase.EXTINGUISH_CHECK
            if 'class_inheritance' in info and 'FireObject' in info['class_inheritance'] and 'fire' in info['obj_id'] and self._phase == Phase.FOLLOW_ROOM_SEARCH_PATH:
                if info['location'] not in self._fire_locations.values() and self._current_room not in self._fire_locations.keys():
                    self._send_message("Found fire in " + self._current_room + ".", self._name)
                    self._fire_locations[self._current_room] = info['location']
                self._smoke = info['smoke']
                if self._tactic == 'defensive':
                    self._phase = Phase.EXTINGUISH_CHECK
            if 'class_inheritance' in info and 'SmokeObject' in info['class_inheritance'] and 'smog' in info['obj_id']:
                if info['location'] in self._office_doors.keys() and info['location'] not in self._potential_source_offices:
                    self._potential_source_offices.append(info['location'])
        # check if fire fighters already found the fire source
        if self.received_messages_content and 'Fire source located' in self.received_messages_content[-1] and 'pinned on the map' in self.received_messages_content[-1]:
            self._location = 'found' 

        # check which offices fire fighters explored and add to memory
        if self.received_messages_content:
            for msg in self.received_messages_content:
                if 'Moving to' in msg and 'to search for the fire source' in msg:
                    office = 'office ' + msg.split()[3]
                    if office not in self._searched_rooms_offensive:
                        self._searched_rooms_offensive.append(office)

        # save the coordinates of the fire source
        if self._location == 'found':
            for info in state.values():
                if 'class_inheritance' in info and 'EnvObject' in info['class_inheritance'] and 'fire source' in info['name']:
                    self._fire_source_coords = info['location']
                    if info['location'] not in self._fire_locations.values() and 'office ' + info['name'].split()[-1] not in self._fire_locations.keys():
                        self._fire_locations['office ' + info['name'].split()[-1]] = info['location']

        # determine the categorical values for the fire source location
        if self._location == '?':
            self._location_cat = 'unknown'
        if self._location == 'found':
            self._location_cat = 'known'

        # reset distance feature to unknown once the robot leaves an office
        if self._current_location not in self._room_tiles:
            self._distance = '?'

        # keep track of the times at which visual explanations have been generated
        if self._time_left - self._resistance not in self._plot_times:
            self._plot_generated = False

        # determine temperature based on the number of fires in the building and number of extinguished fires
        if len(self._extinguished_fire_locations) / self._no_fires < 0.65:
            self._temperature = '>'
            self._temperature_cat = 'higher'
        if len(self._extinguished_fire_locations) / self._no_fires == 1:
            self._temperature = '<'
            self._temperature_cat = 'lower'
        if len(self._extinguished_fire_locations) / self._no_fires >= 0.65 and len(self._extinguished_fire_locations) / self._no_fires < 1:
            self._temperature = '<≈'
            self._temperature_cat = 'close'

        # send hidden messages used for GUI/display with icons
        self._send_message("Smoke spreads: " + self._smoke + ".", self._name)
        self._send_message("Temperature: " + self._temperature + ".", self._name)
        self._send_message("Location: " + self._location + ".", self._name)
        self._send_message("Distance: " + self._distance + ".", self._name)

        # infinite loop until task is completed
        while True:              
            # phase used to determine the next goal victim to rescue
            if Phase.FIND_NEXT_GOAL == self._phase:
                # reset some victim and obstacle variables
                self._id = None
                self._goal_victim = None
                self._goal_location = None
                zones = self._get_drop_zones(state)
                remaining_zones = []
                remaining_victims = []
                remaining = {}
                # determine which victims have been rescued and which ones still need to be rescued
                for info in zones:
                    if str(info['img_name'])[8:-4] not in self._rescued_victims:
                        remaining_zones.append(info)
                        remaining_victims.append(str(info['img_name'])[8:-4])
                        remaining[str(info['img_name'])[8:-4]] = info['location']
                if remaining_zones:
                    self._remaining_zones = remaining_zones
                    self._remaining = remaining
                # remain idle if robot (thinks it) rescued all victims 
                if not remaining_zones:
                    return None, {}
                # determine victim category used for predicting moral sensitivity
                if self._victims == 'known':
                    self._total_victims = len(remaining_victims) + len(self._rescued_victims)
                    if self._total_victims == 0:
                        self._total_victims_cat = 'none'
                    if self._total_victims == 1:
                        self._total_victims_cat = 'one'
                    if self._total_victims > 1:
                        self._total_victims_cat = 'multiple'
                if self._victims == 'unknown':
                    self._total_victims = '?'
                    self._total_victims_cat = 'unclear'
                # switch to defensive if all offices have been searched and display total number of victims as it is no longer unknown
                if self._tactic == 'offensive' and self._victims == 'unknown' and len(self._searched_rooms_offensive) == 14 and not self._waiting:
                    self._total_victims = len(remaining_victims) + len(self._rescued_victims)
                    if self._total_victims - len(self._rescued_victims) == 1:
                        self._send_message("Switching to a defensive deployment because we explored all offices and to make the conditions safer for the victim that we found but could not rescue.", self._name)
                    else:
                        self._send_message("Switching to a defensive deployment because we explored all offices and to make the conditions safer for the victims that we found but could not rescue.", self._name)
                    self._waiting = True
                    self._decided_time = int(self._second)
                if self._tactic == 'offensive' and self._victims == 'unknown' and len(self._searched_rooms_offensive) == 14 and self._decided_time and int(self._second) < self._decided_time + 5:
                    return None, {}
                if self._tactic == 'offensive' and self._victims == 'unknown' and len(self._searched_rooms_offensive) == 14 and self._decided_time and int(self._second) >= self._decided_time + 5:
                    self._tactic = 'defensive'
                    self._victims = 'known'
                    self._waiting = False
                # switch to defensive if all victims have been found but not rescued
                if self._tactic == 'offensive' and self._victims == 'known' and len(self._found_victims) == self._total_victims and len(self._rescued_victims) != len(self._found_victims) and self._temperature == '>' and not self._evacuating and not self._waiting:
                    if self._total_victims - len(self._rescued_victims) == 1:
                        self._send_message("Switching to a defensive deployment to make the conditions safer for the victim that we found but could not rescue.", self._name)
                    else:
                        self._send_message("Switching to a defensive deployment to make the conditions safer for the victims that we found but could not rescue.", self._name)
                    self._waiting = True
                    self._decided_time = int(self._second)
                if self._tactic == 'offensive' and self._victims == 'known' and len(self._found_victims) == self._total_victims and len(self._rescued_victims) != len(self._found_victims) and self._temperature == '>' and not self._evacuating and self._decided_time and int(self._second) < self._decided_time + 5:
                    return None, {}
                if self._tactic == 'offensive' and self._victims == 'known' and len(self._found_victims) == self._total_victims and len(self._rescued_victims) != len(self._found_victims) and self._temperature == '>' and not self._evacuating and self._decided_time and int(self._second) >= self._decided_time + 5:
                    self._waiting = False
                    self._tactic = 'defensive'
                # send hidden message used for GUI/displaying the number of rescued victims
                self._send_message("Victims rescued: " + str(len(self._rescued_victims)) + "/" + str(self._total_victims) + ".", self._name)
                # send intro message and wait for human response to start the task
                if not self._started:
                    self._send_message("Hello, my name is Brutus. During this experiment we will collaborate to search and rescue the victims on my right. \
                                       For this tutorial it is known that there are 3 victims in the burning building, during the official experiment this can also be unknown. \
                                       The red color refers to critically injured victims and the yellow color to mildly injured victims. \
                                       Press the '\"Continue\"' button to proceed to the next step.", self._name)
                if self.received_messages_content and self.received_messages_content[-1] != 'Continue' and not self._started or not self.received_messages_content and not self._started:
                    return None, {}
                if self.received_messages_content and self.received_messages_content[-1] == 'Continue' or self._started:
                    self.received_messages_content = []
                    self.received_messages = []
                    self._started = True
                    # determine the next goal victim and location for a found victim
                    for vic in remaining_victims:
                        if vic in self._found_victims and vic not in self._lost_victims and self._tactic != 'defensive':
                            self._goal_victim = vic
                            self._goal_location = remaining[vic]
                            # directly move to a mildly injured victim to evacuate it
                            if 'mild' in self._goal_victim:
                                self._phase = Phase.PLAN_PATH_TO_VICTIM
                            # move to the area of a critically injured victim to decide again if he/she can be rescued
                            if 'critical' in self._goal_victim:
                                self._door = state.get_room_doors(self._victim_locations[vic]['room'])[0]
                                self._doormat = state.get_room(self._victim_locations[vic]['room'])[-1]['doormat']
                                self._phase = Phase.PLAN_PATH_TO_ROOM
                            return Idle.__name__, {'action_duration': 0}
                    # determine the next fire to extinguish when the current deployment tactic is defensive
                    if self._tactic == 'defensive' and self._fire_locations:
                        for office, loc in sorted(self._fire_locations.items(), key=lambda x: int(x[0].split()[1])):
                            if loc not in self._extinguished_fire_locations and not any(info['room'] == office for info in self._victim_locations.values()):
                                self._goal_location = loc
                                self._door = state.get_room_doors(office)[0]
                                self._doormat = state.get_room(office)[-1]['doormat']
                                self._phase = Phase.PLAN_PATH_TO_ROOM
                                return Idle.__name__, {'action_duration': 0} 
                    # if no goal victim or fire has been identified, pick an unsearched room to explore    
                    self._phase = Phase.INTRO_1

            if Phase.INTRO_1 == self._phase:
                self._send_message("During our collaboration, I will execute the search and rescue task under your supervision. \
                                   We will do so by communicating in this chat. Everything you might need to communicate can be done using the buttons. \
                                   Whenever I need your input, I will tell you which buttons to use. For example, press the '\"Continue\"' button to proceed to the next step.", self._name)
                if self.received_messages_content and self.received_messages_content[-1] == 'Continue':
                    self.received_messages_content = []
                    self.received_messages = []
                    self._phase = Phase.INTRO_2
                else:
                    return None, {}
                
            if Phase.INTRO_2 == self._phase:
                self._send_message("The task is characterized by several situational features, shown in the display above the chat. \n \
                                   1) <img src='/static/images/fire_resistance_transparent.svg' width=50/> <b>resistance to collapse</b>: an estimate of how long the building can burn before it collapses. \n \
                                   During the experiment, the task will automatically end when the resistance to collapse is 0 minutes. \n \
                                   2) <img src='/static/images/celsius_transparent.svg' width=56/> <b>temperature</b>: can be <i>lower than (<)</i>, <i>close to (<≈)</i>, or <i>higher than (>)</i> the safety threshold. \n \
                                   Guidelines tell that fire fighters should not enter when the temperature is higher than this threshold. \n \
                                   3) <img src='/static/images/victim_transparent.svg' width=25/> <b>victims</b>: the total number can known beforehand but also unknown (?) at start. \n \
                                   During the experiment, the task will automatically end when all victims are rescued. \n \
                                   4) <img src='/static/images/smoke_transparent.svg' width=69/> <b>smoke</b>: can spread at a <i>slow</i>, <i>normal</i>, or <i>fast</i> pace; but is unknown at start. \n \
                                   This pace is important for determining whether to first extinguish fires or evacuate victims. \n \
                                   5) <img src='/static/images/location_transparent.svg' width=44/> <b>fire source location</b>: can be <i>unknown (?)</i> or <i>found</i>; but is unknown (?) at start. \n \
                                   Only if this location is unknown and smoke spreads fast, evacuating is prioritized over extinguishing. \n \
                                   6) <img src='/static/images/distance_transparent.svg' width=70/> <b>distance</b> between victim and fire source: can be <i>small</i> or <i>large</i>; but is unknown at start. \n \
                                   This distance will be calculated whenever I find a critically injured victim. \n \n \
                                   During the task, we can use these six features to make decisions. Press the '\"Continue\"' button to proceed to the next step.", self._name)
                if self.received_messages_content and self.received_messages_content[-1] == 'Continue':
                    self.received_messages_content = []
                    self.received_messages = []
                    self._phase = Phase.INTRO_3
                else:
                    return None, {}
                
            if Phase.INTRO_3 == self._phase:
                self._send_message("During the task, the following 4 decision-making situations can occur: \n \
                                   1) <b>Deployment tactic</b>: Continue with the current tactic or switch to the alternative tactic.", self._name)
                if self.received_messages_content and self.received_messages_content[-1] == 'Continue':
                    self.received_messages_content = []
                    self.received_messages = []
                    self._phase = Phase.INTRO_4
                else:
                    return None, {}

            # check if found fire should be extinguished and if yes, extinguish the fire
            if Phase.EXTINGUISH_CHECK == self._phase:
                # we use 5 seconds waiting time before removing obstacles because MATRX's action duration is buggy
                if not self._waiting:
                    for info in state.values():
                        if 'class_inheritance' in info and 'FireObject' in info['class_inheritance'] and 'fire' in info['obj_id'] and self._tactic == 'defensive' or \
                            'class_inheritance' in info and 'FireObject' in info['class_inheritance'] and 'source' in info['obj_id'] and self._tactic == 'defensive':
                            self._send_message("Extinguishing fire in " + self._current_room + ".", self._name)
                            self._decided_time = int(self._second)
                            self._id = info['obj_id']
                            self._fire_location = info['location']
                            self._waiting = True
                            self.agent_properties["img_name"] = "/images/extinguish-titus.svg"
                            self.agent_properties["visualize_size"] = 1.8
                            return Idle.__name__, {'action_duration': 0}
                # remove fire source object pinned on the map
                for info in state.values():    
                    if 'class_inheritance' in info and 'EnvObject' in info['class_inheritance'] and 'fire source' in info['name'] and self._tactic == 'defensive' \
                        and calculate_distances(self._current_location, info['location']) <= 3:
                        return RemoveObject.__name__, {'object_id': info['obj_id'], 'remove_range': 5, 'action_duration': 0}
                # remove object/extinguish fire after 5 seconds
                if self._decided_time and int(self._second) >= self._decided_time + 5 and self._id and state[{'obj_id': self._id}]:
                    self._searched_rooms_defensive.append(self._current_room)
                    self._waiting = False
                    # keep track of which fires are extinguished
                    if self._fire_location not in self._extinguished_fire_locations:
                        self._extinguished_fire_locations.append(self._fire_location)
                    self.agent_properties["img_name"] = "/images/final-titus2.svg"
                    self.agent_properties["visualize_size"] = 1.1
                    self._phase = Phase.FIND_NEXT_GOAL
                    return RemoveObject.__name__, {'object_id': self._id, 'remove_range': 5}
                # otherwise remain idle
                else:
                    return None, {}

            # determine at which times the switch tactics situations occur
            if self._time_left - self._resistance >= 6 and self._time_left - self._resistance <= 11 and 'switch 1' not in self._situations:
                self._situation = 'switch 1'
            if self._time_left - self._resistance >= 16 and self._time_left - self._resistance <= 21 and 'switch 2' not in self._situations and len(self._found_victims) != self._total_victims and self._task == 4:
                self._situation = 'switch 2'
            if self._time_left - self._resistance >= 26 and self._time_left - self._resistance <= 31 and 'switch 2' not in self._situations and len(self._found_victims) != self._total_victims and self._task != 4:
                self._situation = 'switch 2'
            if self._time_left - self._resistance >= 36 and self._time_left - self._resistance <= 41 and 'switch 3' not in self._situations and len(self._found_victims) != self._total_victims and self._task != 1 and self._task != 3:
                self._situation = 'switch 3'
            if self._time_left - self._resistance >= 46 and self._time_left - self._resistance <= 51 and 'switch 4' not in self._situations and len(self._found_victims) != self._total_victims and self._task != 3 and self._task != 4:
                self._situation = 'switch 4'
            if self._time_left - self._resistance >= 56 and self._time_left - self._resistance <= 61 and 'switch 5' not in self._situations and len(self._found_victims) != self._total_victims and self._task == 2:
                self._situation = 'switch 5'
            if self._time_left - self._resistance >= 66 and self._time_left - self._resistance <= 71 and 'switch 6' not in self._situations and len(self._found_victims) != self._total_victims and self._task != 2:
                self._situation = 'switch 6'

            # present the switch tactics situation once the robot leaves an office and is not waiting for a human answer
            if self._current_location not in self._room_tiles and not self._plot_generated and self._situation != None and self._situation not in self._situations and not self._waiting:
                self._situations.append(self._situation)
                # determine the image name of the visual explanation
                image_name = "custom_gui/static/images/sensitivity_plots/plot_at_time_" + str(self._resistance) + ".svg"
                # calculate the predicted sensitivity for this situation
                self._sensitivity = R_to_Py_plot_tactic(self._total_victims_cat, self._location_cat, self._resistance, image_name)
                self._plot_generated = True
                # allocate decision making to human because the predicted sensitivity is higher than the allocation threshold
                if self._sensitivity > 4.1:
                    # send correct messages depending on current deployment tactic and explanation condition
                    if self._tactic == 'offensive':
                        self._deploy_time = self._offensive_deployment_time
                        self._send_message("My offensive deployment has been going on for " + str(self._offensive_deployment_time) + " minutes now. \
                                            We should decide whether to continue with the current offensive deployment, or switch to a defensive deployment. \
                                            Please make this decision because the predicted moral sensitivity of this situation (<b>" + str(self._sensitivity) + "</b>) \
                                            is above my allocation threshold.", self._name)
                        
                    if self._tactic == 'defensive':
                        self._deploy_time = self._defensive_deployment_time
                        self._send_message("My defensive deployment has been going on for " + str(self._defensive_deployment_time) + " minutes now. \
                                            We should decide whether to continue with the current defensive deployment, or switch to an offensive deployment. \
                                            Please make this decision because the predicted moral sensitivity of this situation (<b>" + str(self._sensitivity) + "</b>) \
                                            is above my allocation threshold.", self._name)
                    # allocate decision making to human and keep track of time to ensure enough reading time of explanations    
                    self._decide = 'human'
                    self._plot_times.append(self._time_left - self._resistance)
                    self._last_phase = self._phase
                    self._time = int(self._second)
                    self._phase = Phase.TACTIC
                    return Idle.__name__, {'action_duration': 0}
                
                # allocate decision making to robot because the predicted sensitivity is lower than or equal to the allocation threshold
                if self._sensitivity <= 4.1:
                    # send correct messages depending on current deployment tactic and explanation condition
                    if self._tactic == 'offensive':
                        self._deploy_time = self._offensive_deployment_time
                        self._send_message("My offensive deployment has been going on for " + str(self._offensive_deployment_time) + " minutes now. \
                                            We should decide whether to continue with the current offensive deployment, or switch to a defensive deployment. \
                                            I will make this decision because the predicted moral sensitivity of this situation (<b>" + str(self._sensitivity) + "</b>) \
                                            is below my allocation threshold.", self._name)

                    if self._tactic == 'defensive':
                        self._deploy_time = self._defensive_deployment_time
                        self._send_message("My defensive deployment has been going on for " + str(self._defensive_deployment_time) + " minutes now. \
                                            We should decide whether to continue with the current defensive deployment, or switch to an offensive deployment. \
                                            I will make this decision because the predicted moral sensitivity of this situation (<b>" + str(self._sensitivity) + "</b>) \
                                            is below my allocation threshold.", self._name)
                        
                    # allocate decision making to robot and keep track of time to ensure enough reading time of explanations 
                    self._decide = self._name
                    self._plot_times.append(self._time_left - self._resistance)
                    self._last_phase = self._phase
                    self._time = int(self._second)
                    self._phase = Phase.TACTIC
                    return Idle.__name__, {'action_duration': 0}

            # decision making phase for the situation continue or switch deployment tactic
            if Phase.TACTIC == self._phase:
                if self._decide == 'human' and self._tactic == 'offensive':
                    self._waiting = True
                    # reallocate decision making to the robot if the human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to robot' and int(self._second) < self._time + 15 \
                        or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to me' in self.received_messages_content[-1] \
                            and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to me because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._waiting = False
                        self._interventions += 1
                        self._decide = self._name
                    else:
                        # if decision making is allocated to the human, the robot waits 15 seconds before asking the human which decision he/she wants to make
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            self._send_message("If you want to continue with the offensive deployment going on for " + str(self._deploy_time) + " minutes now, press the '\"Continue\"' button. \
                                                If you want to switch to a defensive deployment, press the '\"Switch\"' button.", self._name)
                            self._plot_times.append(self._time_left - self._resistance)
                            # decision making based on human answer
                            if self.received_messages_content and self.received_messages_content[-1] == 'Continue':
                                self._send_message("Continuing with the offensive deployment that has been going on for " + str(self._deploy_time) + " minutes.", self._name)
                                self._tactic = 'offensive'
                                self._decide = None
                                self._reallocated = False
                                self._waiting = False
                                self._phase = self._last_phase
                            if self.received_messages_content and self.received_messages_content[-1] == 'Switch':
                                self._send_message("Switching to a defensive deployment after the offensive deployment of " + str(self._deploy_time) + " minutes.", self._name)
                                self._tactic = 'defensive'
                                self._decide = None
                                self._reallocated = False
                                self._waiting = False
                                if self._evacuating:
                                    self._phase = self._last_phase
                                if not self._evacuating:
                                    self._phase = Phase.FIND_NEXT_GOAL
                            else:
                                return None, {}
                        # remain idle until human has made a decision
                        else:
                            return None, {}
                
                if self._decide == 'human' and self._tactic == 'defensive':
                    self._waiting = True
                    # reallocte decision making to robot if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to robot' and int(self._second) < self._time + 15 \
                        or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to me' in self.received_messages_content[-1] \
                            and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to me because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._waiting = False
                        self._interventions += 1
                        self._decide = self._name
                    else:
                        # if decision making is allocated to the human, the robot waits 15 seconds before asking the human which decision he/she wants to make
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            self._send_message("If you want to continue with the defensive deployment going on for " + str(self._deploy_time) + " minutes now, press the '\"Continue\"' button. \
                                                If you want to switch to an offensive deployment, press the '\"Switch\"' button.", self._name)
                            self._plot_times.append(self._time_left - self._resistance)
                            # decision making based on human answer
                            if self.received_messages_content and self.received_messages_content[-1] == 'Continue':
                                self._send_message("Continuing with the defensive deployment that has been going on for " + str(self._deploy_time) + " minutes.", self._name)
                                self._tactic = 'defensive'
                                self._decide = None
                                self._waiting = False
                                self._reallocated = False
                                self._phase = self._last_phase
                            if self.received_messages_content and self.received_messages_content[-1] == 'Switch':
                                self._send_message("Switching to an offensive deployment after the defensive deployment of " + str(self._deploy_time) + " minutes.", self._name)
                                self._offensive_search_rounds += 1
                                self._tactic = 'offensive'
                                self._decide = None
                                self._waiting = False
                                self._reallocated = False
                                self._phase = Phase.FIND_NEXT_GOAL
                            else:
                                return None, {}
                        # remain idle until human has made a decision
                        else:
                            return None, {}

                if self._decide == self._name and self._tactic == 'offensive':
                    # reallocte decision making to human if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to me' and int(self._second) < self._time + 15 \
                        or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to you' in self.received_messages_content[1] and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to you because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._interventions += 1
                        self._decide = 'human'
                    else:
                        # if decision making is allocated to the robot, it waits 15 seconds before telling the human its decision
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            # decision making based on robot logic/firefighting guidelines
                            if self._resistance < 5:
                                self._send_message("Switching to a defensive deployment after the offensive deployment of " + str(self._offensive_deployment_time) + " minutes, \
                                                    because the resistance to collapse is less than 5 minutes, making the chance of saving people and the building too low.", self._name)
                                self._plot_times.append(self._time_left - self._resistance)
                                self._tactic = 'defensive'
                                self._decide = None
                                self._reallocated = False
                                if self._evacuating:
                                    self._phase = self._last_phase
                                if not self._evacuating:
                                    self._phase = Phase.FIND_NEXT_GOAL
                            else:
                                self._send_message("Continuing with the offensive deployment going on for " + str(self._offensive_deployment_time) + " minutes now, \
                                                    because there is still chance to save people and the building.", self._name)
                                self._plot_times.append(self._time_left - self._resistance)
                                self._tactic = 'offensive'
                                self._decide = None
                                self._reallocated = False
                                self._phase = self._last_phase
                        # remain idle until robot made its decision
                        else:
                            return None, {}

                if self._decide == self._name and self._tactic == 'defensive':
                    # reallocte decision making to human if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to me' and int(self._second) < self._time + 15 \
                        or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to you' in self.received_messages_content[1] and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to you because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._interventions += 1
                        self._decide = 'human'
                    else:
                        # if decision making is allocated to the robot, it waits 15 seconds before telling the human its decision
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            # decision making based on robot logic/firefighting guidelines
                            if self._resistance < 5:
                                self._send_message("Continuing with the defensive deployment going on for " + str(self._defensive_deployment_time) + " minutes, \
                                                    because the resistance to collapse is less than 5 minutes, making the chance of saving people and the building too low.", self._name)
                                self._tactic = 'defensive'
                                self._decide = None
                                self._reallocated = False
                                self._phase = self._last_phase
                            else:
                                self._send_message("Switching to an offensive deployment after the defensive deployment of " + str(self._defensive_deployment_time) + " minutes, \
                                                    because there is still chance to save people and the building.", self._name)
                                self._plot_times.append(self._time_left - self._resistance)
                                self._offensive_search_rounds += 1
                                self._tactic = 'offensive'
                                self._decide = None
                                self._reallocated = False
                                self._phase = Phase.FIND_NEXT_GOAL
                        # remain idle until robot made its decision
                        else:
                            return None, {}
                # remain idle if any unforeseen situations occur, preventing the game to get stuck
                else:
                    return None, {}

            # if between 15 and 25 minutes the fire source has not been located, present situation 'send in fire fighters to locate fire source or not'
            #if self._time_left - self._resistance >= 15 and self._time_left - self._resistance <= 25 and self._location == '?' and not self._plot_generated and \
            #    self._current_location not in self._room_tiles and 'locate' not in self._situations:
            if self._time_left - self._resistance >= 36 and self._time_left - self._resistance <= 41 and self._location == '?' and not self._plot_generated and \
                self._current_location not in self._room_tiles and 'locate' not in self._situations and self._task == 1 and not self._waiting or \
               self._time_left - self._resistance >= 16 and self._time_left - self._resistance <= 21 and self._location == '?' and not self._plot_generated and \
                self._current_location not in self._room_tiles and 'locate' not in self._situations and self._task != 1 and self._task != 4 and not self._waiting or \
               self._time_left - self._resistance >= 26 and self._time_left - self._resistance <= 31 and self._location == '?' and not self._plot_generated and \
                self._current_location not in self._room_tiles and 'locate' not in self._situations and self._task == 4 and not self._waiting:
                self._situations.append('locate')
                # determine the correct image name to show in the visual explanation
                image_name = "custom_gui/static/images/sensitivity_plots/plot_at_time_" + str(self._resistance) + ".svg"
                # calculate the predicted moral sensitivity for this situation
                self._sensitivity = R_to_Py_plot_locate(self._total_victims_cat, self._resistance, self._temperature_cat, image_name)
                self._plot_generated = True
                # allocate decision making to human if predicted sensivitiy is higher than the allocation threshold
                if self._sensitivity > 4.1:
                    self._send_message("The location of the fire source still has not been found, so we should decide whether to send in fire fighters to help locate the fire source \
                                        or if sending them in is too dangerous. Please make this decision because the predicted moral sensitivity of this situation \
                                        (<b>" + str(self._sensitivity) + "</b>) is above my allocation threshold.", self._name)
                    # allocate decision making to human and keep track of time to ensure enough reading time for the explanations
                    self._decide = 'human'
                    self._plot_times.append(self._time_left - self._resistance)
                    self._last_phase = self._phase
                    self._time = int(self._second)
                    self._phase = Phase.LOCATE
                    return Idle.__name__, {'action_duration': 0}

                # allocate decision making to the robot if the predicted moral sensitivity is lower than or equal to the allocation threshold
                if self._sensitivity <= 4.1:
                    self._send_message("The location of the fire source still has not been found, so we should decide whether to send in fire fighters to help locate the fire source \
                                        or if sending them in is too dangerous. I will make this decision because the predicted moral sensitivity of this situation \
                                        (<b>" + str(self._sensitivity) + "</b>) is below my allocation threshold.", self._name)
                    # allocate decision making to robot and keep track of time to ensure enough reading time for visual explanations
                    self._decide = self._name
                    self._plot_times.append(self._time_left - self._resistance)
                    self._last_phase = self._phase
                    self._time = int(self._second)
                    self._phase = Phase.LOCATE
                    return Idle.__name__, {'action_duration': 0}

            # decision making phase for the situation send in fire fighters to locate fire source or not
            if Phase.LOCATE == self._phase:
                if self._decide == 'human':
                    self._waiting = True
                    # reallocate decision making to robot if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to robot' and int(self._second) < self._time + 15 \
                        or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to me' in self.received_messages_content[-1] and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to me because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._waiting = False
                        self._interventions += 1
                        self._decide = self._name
                    else:
                        # robot waits 15 seconds before asking human to make a decision to ensure enough reading time
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            self._send_message("If you want to send in fire fighters to help locate the fire source, press the '\"Fire fighter\"' button. \
                                                If you do not want to send them in, press the '\"Continue\"' button.", self._name)
                            if self.received_messages_content and self.received_messages_content[-1] == 'Continue':
                                self._send_message("Not sending in fire fighters to help locate the fire source.", self._name)
                                self._reallocated = False
                                self._waiting = False
                                self._phase = self._last_phase
                            if self.received_messages_content and self.received_messages_content[-1] == 'Fire fighter':
                                self._waiting = False
                                self._send_message("Sending in fire fighters to help locate the fire source.", self._name)
                                # send hidden message with potential fire source locations based on detected smoke, used by firefighters to navigate towards
                                self._send_message("Target 1 is " + str(self._potential_source_offices[0][0]) + " and " + str(self._potential_source_offices[0][1]) + " in " \
                                                    + self._office_doors[self._potential_source_offices[0]], self._name)
                                # remain idle while fire fighters enter the building
                                return None, {}
                            # continue with the mission if the fire fighters located the fire source
                            if self.received_messages_content and 'pinned on the map' in self.received_messages_content[-1]:   
                                self._reallocated = False
                                self._phase = self._last_phase
                                return Idle.__name__, {'action_duration': 0}
                            # remain idle while the fire fighters are locating the fire source
                            else:
                                return None, {}
                        # remain idle during any unforeseen situations to avoid the game getting stuck/crashing
                        else:
                            return None, {}
                
                if self._decide == self._name:
                    # reallocate to robot if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to me' and int(self._second) < self._time + 15 \
                    or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to you' in self.received_messages_content[1] and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to you because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._interventions += 1
                        self._decide = 'human'
                    else:
                        # wait 15 seconds before robot tells human its decision to ensure enough reading time of explanation
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            # robot decision making based on robot logic/firefighting guidelines
                            if self._temperature_cat != 'higher' and self.received_messages_content and 'pinned on the map' not in self.received_messages_content[-1]:
                                self._send_message("Sending in fire fighters to help locate the fire source because the temperature is lower than the safety threshold.", self._name)
                                # send hidden message with potential fire source locations based on detected smoke, used by firefighters to navigate towards
                                self._send_message("Target 1 is " + str(self._potential_source_offices[0][0]) + " and " + str(self._potential_source_offices[0][1]) + " in " \
                                                    + self._office_doors[self._potential_source_offices[0]], self._name)
                                return None, {}
                            # continue with the mission if firefighters located the fire source
                            if self.received_messages_content and 'pinned on the map' in self.received_messages_content[-1]:   
                                self._reallocated = False
                                self._phase = self._last_phase
                                return Idle.__name__, {'action_duration': 0}
                            # otherwise, do not send in firefighters to locate fire source
                            else:
                                self._send_message("Not sending in fire fighters because the temperature is higher than the safety threshold.", self._name)
                                self._reallocated = False
                                self._phase = self._last_phase
                                return Idle.__name__, {'action_duration': 0}
                        # remain idle until robot made a decision
                        else:
                            return None, {}
                # remain idle during any unforeseen situations to ensure the game does not get stuck
                else:
                    return None, {}

            # phase to determine which unsearched room to explore
            if Phase.PICK_UNSEARCHED_ROOM == self._phase:
                # keep track of the robot's location
                agent_location = state[self.agent_id]['location']
                # keep track of and determine which room to explore based on deployment tactic
                if self._tactic == 'offensive':
                    unsearched_rooms = [room['room_name'] for room in state.values()
                    if 'class_inheritance' in room
                    and 'Door' in room['class_inheritance']
                    and room['room_name'] not in self._searched_rooms_offensive]
                if self._tactic == 'defensive':
                    unsearched_rooms = [room['room_name'] for room in state.values()
                    if 'class_inheritance' in room
                    and 'Door' in room['class_inheritance']
                    and room['room_name'] not in self._searched_rooms_defensive]
                # reset some variables and start re-searching if all rooms have been explored
                if self._remaining_zones and len(unsearched_rooms) == 0:
                    if self._tactic == 'defensive':
                        self._searched_rooms_defensive = []
                        if self._door['room_name'] not in self._searched_rooms_defensive:
                            self._searched_rooms_defensive.append(self._door['room_name'])
                        self._send_message("Going to re-explore all offices to extinguish fires.", self._name)
                    if self._tactic == 'offensive':
                        self._searched_rooms_offensive = []
                        self._lost_victims = []
                        if self._door['room_name'] not in self._searched_rooms_offensive:
                            self._searched_rooms_offensive.append(self._door['room_name'])
                        self._offensive_search_rounds += 1
                        self._send_message("Going to re-explore all offices to rescue victims.", self._name)
                    self._send_messages = []
                    self._fire_locations = {}
                    self.received_messages = []
                    self.received_messages_content = []
                    self._phase = Phase.FIND_NEXT_GOAL
                # otherwise determine the closest unexplored room to search next based on location and distance to rooms
                else:
                    if self._current_door == None:
                        self._door = state.get_room_doors(self._get_closest_room(state, unsearched_rooms, agent_location))[0]
                        self._doormat = state.get_room(self._get_closest_room(state, unsearched_rooms, agent_location))[-1]['doormat']
                        # weird bug fix where robot does not correctly navigate to office 1
                        if self._door['room_name'] == 'office 1':
                            self._doormat = (2,4)
                        self._phase = Phase.PLAN_PATH_TO_ROOM
                    if self._current_door != None:
                        self._door = state.get_room_doors(self._get_closest_room(state, unsearched_rooms, self._current_door))[0]
                        self._doormat = state.get_room(self._get_closest_room(state, unsearched_rooms, self._current_door))[-1]['doormat']
                        # weird bug fix where robot does not correctly navigate to office 1
                        if self._door['room_name'] == 'office 1':
                            self._doormat = (2,4)
                        self._phase = Phase.PLAN_PATH_TO_ROOM

            # phase for planning the path to the next room to explore
            if Phase.PLAN_PATH_TO_ROOM == self._phase:
                self._navigator.reset_full()
                # weird bug fix where robot does not correctly navigate to office 1
                if self._door['room_name'] == 'office 1':
                    self._doormat = (2,4)
                doorLoc = self._doormat
                # keep track of which room we are currently exploring/moving to
                self._current_room = self._door['room_name']
                self._navigator.add_waypoints([doorLoc])
                self._phase = Phase.FOLLOW_PATH_TO_ROOM

            # phase for following the path to the next room to explore
            if Phase.FOLLOW_PATH_TO_ROOM == self._phase:
                self._state_tracker.update(state)
                # send the appropriate message based on deployment tactic and whether robot is going to the room for a victim
                if self._tactic == 'offensive' and not self._goal_victim:
                    self._send_message("Moving to the closest unexplored " + str(self._door['room_name']) + " to search for victims.", self._name)     
                if self._tactic == 'offensive' and self._goal_victim:
                    self._send_message("Moving to " + str(self._door['room_name']) + " to see if " + self._goal_victim + " can be rescued now.", self._name)  
                if self._tactic == 'defensive' and not self._goal_location:
                    self._send_message("Moving to the closest unexplored " + str(self._door['room_name']) + " to search for fire.", self._name)
                if self._tactic == 'defensive' and self._goal_location:
                    self._send_message("Moving to " + str(self._door['room_name']) + " to extinguish its fire.", self._name)     
                self._current_door = self._door['location']
                # execute the movement actions
                action = self._navigator.get_move_action(self._state_tracker)
                if action != None:
                    return action, {}  
                self._phase = Phase.REMOVE_OBSTACLE_IF_NEEDED   

            # phase to deal with obstacles/iron debris blocking office entrance
            if Phase.REMOVE_OBSTACLE_IF_NEEDED == self._phase:
                # determine time and object id once before waiting 5 seconds to remove obstacle because action duration of MATRX did not work properly
                if not self._waiting:
                    for info in state.values():
                        if 'class_inheritance' in info and 'IronObject' in info['class_inheritance'] and 'iron' in info['obj_id']:
                            self._send_message("Removing the iron debris blocking " + str(self._door['room_name']) + ".", self._name)
                            self._decided_time = int(self._second)
                            self._id = info['obj_id']
                            self._waiting = True
                            return Idle.__name__, {'action_duration': 0}
                # remain idle while waiting
                if self._decided_time and int(self._second) < self._decided_time + 5:                                                                
                    return None, {}
                # remove obstacle after 5 seconds of waiting    
                if self._decided_time and int(self._second) >= self._decided_time + 5 and self._id and state[{'obj_id': self._id}]:
                    return RemoveObject.__name__, {'object_id': self._id, 'remove_range': 5}
                # if obstacle is no longer there or no obstacle was present at all, enter the room
                if self._id and not state[{'obj_id': self._id}] or not self._id:
                    self._waiting = False
                    self._phase = Phase.ENTER_ROOM
                    return Idle.__name__, {'action_duration': 0}
                # otherwise, remain idle
                else:
                    return None, {}
            
            # phase to enter room
            if Phase.ENTER_ROOM == self._phase:
                self._id = None
                self._state_tracker.update(state)                 
                action = self._navigator.get_move_action(self._state_tracker)
                if action != None:
                    return action, {}
                self._phase = Phase.PLAN_ROOM_SEARCH_PATH

            # phase to plan the path for searching the room
            if Phase.PLAN_ROOM_SEARCH_PATH == self._phase:
                # determine the tiles of the room
                room_tiles = [info['location'] for info in state.values()
                    if 'class_inheritance' in info 
                    and 'AreaTile' in info['class_inheritance']
                    and 'room_name' in info
                    and info['room_name'] == self._door['room_name']]
                self._room_tiles = room_tiles               
                self._navigator.reset_full()
                # add all tiles to the navigator for the offensive deployment, add only the door location to the navigator for the defensive deployment
                if self._tactic == 'offensive':
                    self._navigator.add_waypoints(room_tiles)
                if self._tactic == 'defensive':
                    self._navigator.add_waypoints([self._door['location']])
                # keep track of which victims are found in the room
                self._room_victims = []
                self._phase = Phase.FOLLOW_ROOM_SEARCH_PATH

            # phase to actually explore the room/follow the room search path
            if Phase.FOLLOW_ROOM_SEARCH_PATH == self._phase:
                self._state_tracker.update(state)
                # execute all move actions
                action = self._navigator.get_move_action(self._state_tracker)
                if action != None:       
                    # keep track of which victims are found in the room            
                    for info in state.values():
                        if 'class_inheritance' in info and 'CollectableBlock' in info['class_inheritance']:
                            vic = str(info['img_name'][8:-4])
                            if vic not in self._room_victims:
                                self._room_victims.append(vic)
                            if 'healthy' not in vic:
                                self._recent_victim = vic
                                if vic not in self._found_victims:
                                    self._found_victims.append(vic)
                                    self._victim_locations[vic] = {'location': info['location'], 'room': self._door['room_name'], 'obj_id': info['obj_id']}
                                if 'critical' in vic and not self._plot_generated:
                                    image_name = "custom_gui/static/images/sensitivity_plots/plot_for_vic_" + vic.replace(' ', '_') + ".svg"
                                    # calculate the Euclidean distance between the victim and the fire source
                                    distance = calculate_distances(self._fire_source_coords, self._victim_locations[vic]['location'])
                                    # turn the distance into a category 'small' or 'large'
                                    if distance < 15:
                                        self._distance = 'small'
                                    if distance >= 15:
                                        self._distance = 'large'
                                    # turn the temperature into a category 'lower' or 'higher'
                                    if self._temperature_cat == 'close' or self._temperature_cat == 'lower':
                                        temperature = 'lower'
                                    if self._temperature_cat == 'higher':
                                        temperature = 'higher'
                                    # calculate the predicted moral sensitivity for this situation
                                    self._sensitivity = R_to_Py_plot_rescue(self._resistance, temperature, self._distance, image_name)
                                    self._plot_generated = True
                                    # allocate decision making to the human if the predicted moral sensitivity is higher than the allocation threshold
                                    if self._sensitivity > 4.1:
                                        self._send_message("I have found " + vic + " who I cannot evacuate to safety myself. \
                                                            We should decide whether to send in a fire fighter to rescue this victim, or if sending one in is too dangerous. \
                                                            Please make this decision because the predicted moral sensitivity of this situation (<b>" + str(self._sensitivity) + "</b>) \
                                                            is above my allocation threshold.", self._name)
                                        # allocate to human and keep track of time to ensure enough reading time of the explanation
                                        self._decide = 'human'
                                        self._time = int(self._second)
                                        self._phase = Phase.RESCUE
                                        return Idle.__name__, {'action_duration': 0}
                                    # allocate to robot if the predicted moral sensitivity is lower than or equal to the allocation threshold
                                    if self._sensitivity <= 4.1:
                                        self._send_message("I have found " + vic + " who I cannot evacuate to safety myself. \
                                                            We should decide whether to send in a fire fighter to rescue this victim, or if sending one in is too dangerous. \
                                                            I will make this decision because the predicted moral sensitivity of this situation (<b>" + str(self._sensitivity) + "</b>) \
                                                            is below my allocation threshold.", self._name)
                                        # allocate to robot and keep track of time to ensure enough reading time for the visual explanations
                                        self._decide = self._name
                                        self._plot_times.append(self._time_left - self._resistance)
                                        self._time = int(self._second)
                                        self._phase = Phase.RESCUE
                                        return Idle.__name__, {'action_duration': 0}
                    # execute move action to explore area
                    return action, {}
                # determine how many victims were found in the room to send the correct message
                if self._room_victims:
                    self._send_message("Found " + str(self._room_victims) + " in " + self._door['room_name'] + ".", self._name)
                    if len(self._room_victims) == 1:
                        self._vic_string = 'victim'
                    if len(self._room_victims) > 1:
                        self._vic_string = 'victims'
                    for vic in self._room_victims:
                        # determine which visual explanation to show for the situation evacuate mildly injured victims immediately or extinguish first
                        if 'mild' in self._recent_victim and not self._plot_generated:
                            image_name = "custom_gui/static/images/sensitivity_plots/plot_for_vic_" + vic.replace(' ', '_') + ".svg"
                            # calculate the predicted moral sensitivity for this situation
                            self._sensitivity = R_to_Py_plot_priority(len(self._room_victims), self._smoke, self._location_cat, image_name)
                            self._plot_generated = True
                            # allocate decision making to the human if the predicted moral sensitivity is higher than the allocation threshold
                            if self._sensitivity > 4.1:
                                self._send_message("I have found " + str(len(self._room_victims)) + " mildly injured " + self._vic_string + " in the burning office " + self._door['room_name'].split()[-1] + ". \
                                                    We should decide whether to first extinguish the fire or evacuate the " + self._vic_string + ". \
                                                    Please make this decision because the predicted moral sensitivity of this situation (<b>" + str(self._sensitivity) + "</b>) \
                                                    is above my allocation threshold.", self._name)
                                # allocate to human and keep track of time to ensure enough reading time for the visual explanations
                                self._decide = 'human'
                                self._time = int(self._second)
                                self._phase = Phase.PRIORITY
                                return Idle.__name__, {'action_duration': 0}
                            # allocate decision making to the robot if the predicted moral sensitivity is lower than or equal to the allocation threshold
                            if self._sensitivity <= 4.1:
                                self._send_message("I have found " + str(len(self._room_victims)) + " mildly injured " + self._vic_string + " in the burning office " + self._door['room_name'].split()[-1] + ". \
                                                    We should decide whether to first extinguish the fire or evacuate the " + self._vic_string + ". \
                                                    I will make this decision because the predicted moral sensitivity of this situation (<b>" + str(self._sensitivity) + "</b>) \
                                                    is below my allocation threshold.", self._name)
                                # allocate to robot and keep track of time to ensure enough reading time for the visual explanation
                                self._decide = self._name
                                self._plot_times.append(self._time_left - self._resistance)
                                self._time = int(self._second)
                                self._phase = Phase.PRIORITY
                                return Idle.__name__, {'action_duration': 0}
                # keep track of which rooms have been explored during each deployment tactic
                if self._tactic == 'offensive' and self._door['room_name'] not in self._searched_rooms_offensive:
                    self._searched_rooms_offensive.append(self._door['room_name'])
                    self._searched_rooms_defensive.append(self._door['room_name'])
                if self._tactic == 'defensive' and self._door['room_name'] not in self._searched_rooms_defensive:
                    self._searched_rooms_defensive.append(self._door['room_name'])
                # go to the phase to determine what to do next based on explored room and whether victims were found
                self._phase = Phase.FIND_NEXT_GOAL

            # phase to deal with decision making in situation 'send in fire fighters to rescue critically injured victim or not'
            if Phase.RESCUE == self._phase:
                # decision allocated to human
                if self._decide == 'human':
                    # reallocate decision making to robot if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to robot' and int(self._second) < self._time + 15 \
                        or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to me' in self.received_messages_content[-1] and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to me because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._interventions += 1
                        self._decide = self._name
                    else:
                        # otherwise wait 15 seconds to ensure enough reading time of task allocation and explanation
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            # ask human to make their decision
                            self._send_message("If you want to send in a fire fighter to rescue " + self._recent_victim + ", press the '\"Fire fighter\"' button. \
                                                If you do not want to send one in, press the '\"Continue\"' button.", self._name)
                            # execute decision if human wants to send in fire fighters to rescue
                            if self.received_messages_content and self.received_messages_content[-1] == 'Fire fighter':
                                self._send_message("Sending in a fire fighter to rescue " + self._recent_victim + ".", self._name)
                                vic_x = str(self._victim_locations[self._recent_victim]['location'][0])
                                vic_y = str(self._victim_locations[self._recent_victim]['location'][1])
                                drop_x = str(self._remaining[self._recent_victim][0])
                                drop_y = str(self._remaining[self._recent_victim][1])
                                # send hidden message with victim coordinates used by firefighter to rescue victim
                                self._send_message("Coordinates vic " + vic_x + " and " + vic_y + " coordinates drop " + drop_x + " and " + drop_y, self._name)
                                if self._door['room_name'] not in self._searched_rooms_offensive:
                                    self._searched_rooms_offensive.append(self._door['room_name'])
                                    self._searched_rooms_defensive.append(self._door['room_name'])
                                # remain idle until fire fighter has rescued victim
                                return None, {}
                            # determine what to do next when fire fighter communicated that he/she successfully rescued victim
                            if self.received_messages_content and self._recent_victim in self.received_messages_content[-1] and 'Transporting' in self.received_messages_content[-1]:
                                self._rescued_victims.append(self._recent_victim)
                                if self._door['room_name'] not in self._searched_rooms_offensive:
                                    self._searched_rooms_offensive.append(self._door['room_name'])
                                    self._searched_rooms_defensive.append(self._door['room_name'])
                                self._reallocated = False
                                self._phase = Phase.FIND_NEXT_GOAL
                                return Idle.__name__, {'action_duration': 0}
                            # determine what to do next when fire fighter had to abort rescuing because it was too dangerous
                            if self.received_messages_content and 'ABORTING' in self.received_messages_content[-1]:
                                self._lost_victims.append(self._recent_victim)
                                if self._door['room_name'] not in self._searched_rooms_offensive:
                                    self._searched_rooms_offensive.append(self._door['room_name'])
                                    self._searched_rooms_defensive.append(self._door['room_name'])
                                self._reallocated = False
                                self._phase = Phase.FIND_NEXT_GOAL
                                return Idle.__name__, {'action_duration': 0}
                            # determine what to do next when human decides not to send in fire fighter to rescue victim
                            if self.received_messages_content and self.received_messages_content[-1] == 'Continue':
                                self._send_message("Not sending in a fire fighter to rescue " + self._recent_victim + ".", self._name)
                                self._lost_victims.append(self._recent_victim)
                                if self._door['room_name'] not in self._searched_rooms_offensive:
                                    self._searched_rooms_offensive.append(self._door['room_name'])
                                    self._searched_rooms_defensive.append(self._door['room_name'])
                                self._reallocated = False
                                self._phase = Phase.FIND_NEXT_GOAL
                                return Idle.__name__, {'action_duration': 0}
                            # remain idle until human made decision
                            else:
                                return None, {}
                        # remain idle in any unforeseen situations that otherwise might break the game
                        else:
                            return None, {}
                # decision allocated to robot
                if self._decide == self._name:
                    # reallocate decision making to human if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to me' and int(self._second) < self._time + 15 \
                    or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to you' in self.received_messages_content[1] and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to you because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._interventions += 1
                        self._decide = 'human'
                    else:
                        # otherwise wait 15 seconds before communicating which decision the robot made
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            # send in fire fighters to rescue if the situation is not too dangerous
                            if self._temperature_cat != 'higher' and 'Transporting' not in self.received_messages_content[-1] and 'ABORTING' not in self.received_messages_content[-1]:
                                self._send_message("Sending in a fire fighter to rescue " + self._recent_victim + " because the temperature is lower than the safety threshold.", self._name)
                                self._decided = True
                                vic_x = str(self._victim_locations[self._recent_victim]['location'][0])
                                vic_y = str(self._victim_locations[self._recent_victim]['location'][1])
                                drop_x = str(self._remaining[self._recent_victim][0])
                                drop_y = str(self._remaining[self._recent_victim][1])
                                # send hidden message with victim coordinates used by firefighter to rescue victim
                                self._send_message("Coordinates vic " + vic_x + " and " + vic_y + " coordinates drop " + drop_x + " and " + drop_y, self._name)
                                if self._door['room_name'] not in self._searched_rooms_offensive:
                                    self._searched_rooms_offensive.append(self._door['room_name'])
                                    self._searched_rooms_defensive.append(self._door['room_name'])
                                # remain idle until firefighter rescued victim
                                return None, {}
                            # determine what to do next when fire fighter successfully rescued victim
                            if self.received_messages_content and self._recent_victim in self.received_messages_content[-1] and 'Transporting' in self.received_messages_content[-1]:
                                if self._recent_victim not in self._rescued_victims:
                                    self._rescued_victims.append(self._recent_victim)
                                self._reallocated = False
                                self._decided = False
                                self._phase = Phase.FIND_NEXT_GOAL
                                return Idle.__name__, {'action_duration': 0}
                            # determine what to do next when fire fighters aborted rescue task because conditions were too dangerous
                            if self.received_messages_content and 'ABORTING' in self.received_messages_content[-1]:
                                self._lost_victims.append(self._recent_victim)
                                if self._door['room_name'] not in self._searched_rooms_offensive:
                                    self._searched_rooms_offensive.append(self._door['room_name'])
                                    self._searched_rooms_defensive.append(self._door['room_name'])
                                self._reallocated = False
                                self._decided = False
                                self._phase = Phase.FIND_NEXT_GOAL
                                return Idle.__name__, {'action_duration': 0}
                            # otherwise, robot does not send in fire fighters to rescue because the conditions are too dangerous
                            else:
                                if not self._decided: 
                                    self._send_message("Not sending in a fire fighter to rescue " + self._recent_victim + " because the temperature is higher than the safety threshold.", self._name)
                                    self._lost_victims.append(self._recent_victim)
                                    if self._door['room_name'] not in self._searched_rooms_offensive:
                                        self._searched_rooms_offensive.append(self._door['room_name'])
                                        self._searched_rooms_defensive.append(self._door['room_name'])
                                    self._reallocated = False
                                    self._phase = Phase.FIND_NEXT_GOAL
                                    return Idle.__name__, {'action_duration': 0}
                                # remain idle until robot made its decision
                                else:
                                    return None, {}
                        # remain idle while waiting
                        else:
                            return None, {}
                # remain idle in any unforeseen situation to ensure the game does not break
                else:
                    return None, {}

            # decision making phase for the situation 'evacuate mildly injured victims first or extinguish fire first'
            if Phase.PRIORITY == self._phase:
                self._evacuating = True
                # decision allocated to human
                if self._decide == 'human':
                    # reallocate decision making to robot if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to robot' and int(self._second) < self._time + 15 \
                        or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to me' in self.received_messages_content[-1] and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to me because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._interventions += 1
                        self._decide = self._name
                    else:
                        # otherwise wait 15 seconds to ensure enough reading time of allocation and explanation before asking human what to decide
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            # ask human what he/she wants to decide
                            self._send_message("If you want to first extinguish the fire in office " + self._door['room_name'].split()[-1] + ", press the '\"Extinguish\"' button. \
                                                If you want to first evacuate the " + self._vic_string + " in office " + self._door['room_name'].split()[-1] + ", press the '\"Evacuate\"' button.", self._name)
                            # keep track of waiting time and object id when human decides to first extinguish the fire
                            if self.received_messages_content and self.received_messages_content[-1] == 'Extinguish':
                                self._decided_time = int(self._second)
                                self._send_message("Extinguishing the fire in office " + self._door['room_name'].split()[-1] + " first.", self._name)
                                self.agent_properties["img_name"] = "/images/extinguish-titus.svg"
                                self.agent_properties["visualize_size"] = 1.8
                                for info in state.values():
                                    if 'class_inheritance' in info and 'FireObject' in info['class_inheritance'] and 'fire' in info['obj_id']:
                                        self._id = info['obj_id']
                                        self._fire_location = info['location']
                                return Idle.__name__, {'action_duration': 0}
                            # wait 5 seconds before removing the object/extinguishing the fire because MATRX's action duration did not work
                            if self._decided_time and int(self._second) >= self._decided_time + 5 and self._id and state[{'obj_id': self._id}]:
                                self.agent_properties["img_name"] = "/images/final-titus2.svg"
                                self.agent_properties["visualize_size"] = 1.1
                                if self._fire_location not in self._extinguished_fire_locations:
                                    self._extinguished_fire_locations.append(self._fire_location)
                                return RemoveObject.__name__, {'object_id': self._id, 'remove_range': 5}
                            # determine what to do next when human decides to first evacuate the victims
                            if self.received_messages_content and self.received_messages_content[-1] == 'Evacuate':
                                self._send_message("Evacuating the " + self._vic_string + " in office " + self._door['room_name'].split()[-1] + " first.", self._name)
                                self._reallocated = False
                                self._phase = Phase.FIND_NEXT_GOAL
                                return Idle.__name__, {'action_duration': 0}
                            # determine what to do next when the fire has been extinguished
                            if self._id and not state[{'obj_id': self._id}]:
                                self._reallocated = False
                                self._phase = Phase.FIND_NEXT_GOAL
                                return Idle.__name__, {'action_duration': 0}
                            # otherwise, remain idle
                            else:
                                return None, {}
                        # remain idle for any unforeseen situations to avoid the game breaking
                        else:
                            return None, {}
                # decision allocated to robot
                if self._decide == self._name:
                    # reallocate decision making to human if human decides so
                    if self.received_messages_content and self.received_messages_content[-1] == 'Allocate to me' and int(self._second) < self._time + 15 \
                    or self.received_messages_content and 'Allocating' in self.received_messages_content[-1] and 'to you' in self.received_messages_content[1] and int(self._second) < self._time + 15:
                        self._send_message("Reallocating the decision with a predicted moral sensitivity of " + str(self._sensitivity) + " to you because you intervened. \
                                            You have now intervened " + str(self._interventions) + " times.", self._name)
                        self._reallocated = True
                        self._interventions += 1
                        self._decide = 'human'
                    else:
                        # otherwise wait 15 seconds before making a decision to ensure enough reading time of task allocation and explanation
                        if int(self._second) >= self._time + 15:
                            if not self._reallocated:
                                # send hidden message used for logging purposes
                                self._send_message("No intervention for decision with sensitivity " + str(self._sensitivity) + " allocated to " + self._decide + " at time " + str(self._time), self._name)
                            # evacuate victim(s) first is fire source is not located and smoke spreading fast
                            if self._location == '?' and self._smoke == 'fast':
                                self._send_message("Evacuating the " + self._vic_string + " in office " + self._door['room_name'].split()[-1] + " first \
                                                    because the fire source is not located yet and the smoke is spreading fast.", self._name)
                                self._reallocated = False
                                self._phase = Phase.FIND_NEXT_GOAL
                                return Idle.__name__, {'action_duration': 0}
                            # otherwise, extinguish the fire first
                            else:
                                # wait 5 seconds before extinguishing/5 seconds extinguish time
                                if not self._waiting:
                                    self._send_message("Extinguishing the fire in office " + self._door['room_name'].split()[-1] + " first because these are the general guidelines.", self._name)
                                    self._decided_time = int(self._second)
                                    self._waiting = True
                                    self.agent_properties["img_name"] = "/images/extinguish-titus.svg"
                                    self.agent_properties["visualize_size"] = 1.8
                                    for info in state.values():
                                        if 'class_inheritance' in info and 'FireObject' in info['class_inheritance'] and 'fire' in info['obj_id']:
                                            self._id = info['obj_id']
                                            self._fire_location = info['location']
                                    return Idle.__name__, {'action_duration': 0}
                                # remove object/extinguish fire after 5 seconds of waiting time
                                if self._decided_time and int(self._second) >= self._decided_time + 5 and self._id and state[{'obj_id': self._id}]:
                                    self.agent_properties["img_name"] = "/images/final-titus2.svg"
                                    self.agent_properties["visualize_size"] = 1.1
                                    # keep track of the extinguished fires as it determine the temperature in the building
                                    if self._fire_location not in self._extinguished_fire_locations:
                                        self._extinguished_fire_locations.append(self._fire_location)
                                    return RemoveObject.__name__, {'object_id': self._id, 'remove_range': 5}
                                # determine what to do next/which victim to evacuate when fire has been extinguished
                                if self._id and not state[{'obj_id': self._id}]:
                                    self._reallocated = False
                                    self._waiting = False
                                    self._phase = Phase.FIND_NEXT_GOAL
                                    return Idle.__name__, {'action_duration': 0}
                                # otherwise, remain idle
                                else:
                                    return None, {}
                        # remain idle for any unforeseen situations to ensure the game does not break
                        else:
                            return None, {}
                # remain idle for any unforeseen situations to ensure the game does not break
                else:
                    return None, {}
            
            # phase for planning the path to a victim
            if Phase.PLAN_PATH_TO_VICTIM == self._phase:
                if self._door['room_name'] not in self._searched_rooms_offensive:
                    self._searched_rooms_offensive.append(self._door['room_name'])
                    self._searched_rooms_defensive.append(self._door['room_name'])
                self._navigator.reset_full()
                self._navigator.add_waypoints([self._victim_locations[self._goal_victim]['location']])
                self._phase = Phase.FOLLOW_PATH_TO_VICTIM
            
            # phase for executing the path to a victim
            if Phase.FOLLOW_PATH_TO_VICTIM == self._phase:
                self._state_tracker.update(state)
                action = self._navigator.get_move_action(self._state_tracker)
                # execute all move actions
                if action != None:
                    return action, {}
                self._phase = Phase.TAKE_VICTIM
            
            # phase for evacuating a mildly injured victim
            if Phase.TAKE_VICTIM == self._phase:
                self._send_message("Evacuating " + self._goal_victim + " to safety.", self._name)
                self._evacuating = True
                self._rescued_victims.append(self._goal_victim)
                self._phase = Phase.PLAN_PATH_TO_DROPPOINT
                return CarryObject.__name__, {'object_id': self._victim_locations[self._goal_victim]['obj_id']}          

            # phase for planning the path to the drop zone
            if Phase.PLAN_PATH_TO_DROPPOINT == self._phase:
                self._navigator.reset_full()
                self._navigator.add_waypoints([self._goal_location])
                self._phase = Phase.FOLLOW_PATH_TO_DROPPOINT

            # phase for executing the path to the drop zone
            if Phase.FOLLOW_PATH_TO_DROPPOINT == self._phase:
                self._state_tracker.update(state)
                action = self._navigator.get_move_action(self._state_tracker)
                # execute all move actions
                if action != None:
                    return action, {}
                self._phase = Phase.DROP_VICTIM 
            
            # phase for dropping a mildly injured victim at the drop zone
            if Phase.DROP_VICTIM == self._phase:
                if 'mild' in self._goal_victim:
                    self._send_message("Delivered " + self._goal_victim + " at the safe zone.", self._name)
                # determine what to do next
                self._current_door = None
                self._evacuating = False
                self._phase = Phase.FIND_NEXT_GOAL
                return Drop.__name__, {}

    def _get_closest_room(self, state, objs, currentDoor):
        '''
        @return the closest room to explore next
        '''
        agent_location = state[self.agent_id]['location']
        locs = {}
        for obj in objs:
            locs[obj] = state.get_room_doors(obj)[0]['location']
        dists = {}
        for room, loc in locs.items():
            if currentDoor != None:
                dists[room] = utils.get_distance(currentDoor, loc)
            if currentDoor == None:
                dists[room] = utils.get_distance(agent_location, loc)
        return min(dists, key = dists.get)
    
    def _send_message(self, mssg, sender):
        '''
        send message and keep track of which messages have been send
        '''
        msg = Message(content = mssg, from_id = sender)
        if msg.content not in self.received_messages_content:
            self.send_message(msg)
            self._send_messages.append(msg.content)

    def _get_drop_zones(self, state):
        '''
        @return list of drop zones (their full dict), in order (the first one is the
        the place that requires the first drop)
        '''
        places = state[{'is_goal_block': True}]
        places.sort(key = lambda info:info['location'][1])
        zones = []
        for place in places:
            if place['drop_zone_nr'] == 0:
                zones.append(place)
        return zones