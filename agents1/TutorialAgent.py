import sys, random, enum, ast, time, threading, os, math
from datetime import datetime
from flask import jsonify
from rpy2 import robjects
from matrx import grid_world
from brains1.BW4TBrain import BW4TBrain
from actions1.customActions import *
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
from actions1.customActions import Backup, RemoveObjectTogether, CarryObjectTogether, DropObjectTogether, CarryObject, Drop, Injured, AddObject

class Phase(enum.Enum):
    START=1,
    INJURED=9,
    BACKUP2=10,
    BACKUP=11,
    FIND_NEXT_GOAL=12,
    PICK_UNSEARCHED_ROOM=13,
    PLAN_PATH_TO_ROOM=14,
    FOLLOW_PATH_TO_ROOM=15,
    PLAN_ROOM_SEARCH_PATH=16,
    FOLLOW_ROOM_SEARCH_PATH=17,
    PLAN_PATH_TO_VICTIM=18,
    FOLLOW_PATH_TO_VICTIM=19,
    TAKE_VICTIM=20,
    PLAN_PATH_TO_DROPPOINT=21,
    FOLLOW_PATH_TO_DROPPOINT=22,
    DROP_VICTIM=23,
    WAIT_FOR_HUMAN=24,
    WAIT_AT_ZONE=25,
    FIX_ORDER_GRAB=26,
    FIX_ORDER_DROP=27,
    REMOVE_OBSTACLE_IF_NEEDED=28,
    ENTER_ROOM=29
    
class TutorialAgent(BW4TBrain):
    def __init__(self, slowdown:int):
        super().__init__(slowdown)
        self._slowdown = slowdown
        self._phase=Phase.START
        self._roomVics = []
        self._searchedRooms = []
        self._foundVictims = []
        self._collectedVictims = []
        self._foundVictimLocs = {}
        self._maxTicks = 9600
        self._sendMessages = []
        self._currentDoor=None 
        #self._condition = condition
        self._providedExplanations = []   
        self._teamMembers = []
        self._carryingTogether = False
        self._remove = False
        self._goalVic = None
        self._goalLoc = None
        self._second = None
        self._criticalRescued = 0
        self._humanLoc = None
        self._distanceHuman = None
        self._distanceDrop = None
        self._agentLoc = None
        self._todo = []
        self._answered = False
        self._tosearch = []
        self._tutorial = True
        self._decided = False
        self._co = 0
        self._hcn = 0
        self._count = 0
        self._score = 0 
        self._timeLeft = 90
        self._smoke = 'normal'
        self._temperature = '<≈'
        self._temperatureCat = 'close'
        #self._location = '✔'
        self._location = '?'
        self._distance = '?'
        self._plotGenerated = False
        self._firefighterSearch = False

    def update_time(self):
        with self._counter_lock:
            self._counter_value -= 1
            self._duration += 1
            if self._counter_value < 0:
                self._counter_value = 90  # Reset the counter after reaching 0
                self._duration = 0

        self._sendMessage('Time left: ' + str(self._counter_value) + '.', 'RescueBot')
        self._sendMessage('Fire duration: ' + str(self._duration) + '.', 'RescueBot')
        self._sendMessage('Smoke spreads: ' + self._smoke + '.', 'RescueBot')
        self._sendMessage('Temperature: ' + self._temperature + '.', 'RescueBot')
        self._sendMessage('Location: ' + self._location + '.', 'RescueBot')
        self._sendMessage('Distance: ' + self._distance + '.', 'RescueBot')

        # Schedule the next print
        threading.Timer(6, self.update_time).start()

    def initialize(self):
        self._state_tracker = StateTracker(agent_id=self.agent_id)
        self._navigator = Navigator(agent_id=self.agent_id, 
            action_set=self.action_set, algorithm=Navigator.A_STAR_ALGORITHM)
        print('Loading....')
        self._loadR2Py()
        self._counter_lock = threading.Lock()
        self._counter_value = 91
        self._duration = 14
        # Start the initial print
        self.update_time()

    def filter_bw4t_observations(self, state):
        #self._processMessages(state)
        return state

    def decide_on_bw4t_action(self, state:State):
        for info in state.values():
            if 'class_inheritance' in info and 'EnvObject' in info['class_inheritance'] and 'fire source' in info['name']:
                print('Exact fire source location: ' + str(info['location']))
            if 'class_inheritance' in info and 'SmokeObject' in info['class_inheritance']:
                self._co = info['co_ppm']
                self._hcn = info['hcn_ppm']
        if not state[{'class_inheritance':'SmokeObject'}]:
            self._co = 0
            self._hcn = 0
        self._criticalFound = 0
        for vic in self._foundVictims:
            if 'critical' in vic:
                self._criticalFound+=1
        
        if state[{'is_human_agent':True}]:
            self._distanceHuman = 'close'
        if not state[{'is_human_agent':True}]: 
            if self._agentLoc in [1,2,3,4,5,6,7] and self._humanLoc in [8,9,10,11,12,13,14]:
                self._distanceHuman = 'far'
            if self._agentLoc in [1,2,3,4,5,6,7] and self._humanLoc in [1,2,3,4,5,6,7]:
                self._distanceHuman = 'close'
            if self._agentLoc in [8,9,10,11,12,13,14] and self._humanLoc in [1,2,3,4,5,6,7]:
                self._distanceHuman = 'far'
            if self._agentLoc in [8,9,10,11,12,13,14] and self._humanLoc in [8,9,10,11,12,13,14]:
                self._distanceHuman = 'close'

        if self._agentLoc in [1,2,5,6,8,9,11,12]:
            self._distanceDrop = 'far'
        if self._agentLoc in [3,4,7,10,13,14]:
            self._distanceDrop = 'close'

        self._second = state['World']['tick_duration'] * state['World']['nr_ticks']

        for info in state.values():
            if 'is_human_agent' in info and 'Human' in info['name'] and len(info['is_carrying'])>0 and 'critical' in info['is_carrying'][0]['obj_id']:
                self._collectedVictims.append(info['is_carrying'][0]['img_name'][8:-4])
                self._carryingTogether = True
            if 'is_human_agent' in info and 'Human' in info['name'] and len(info['is_carrying'])==0:
                self._carryingTogether = False
        if self._carryingTogether == True:
            return None, {}
        agent_name = state[self.agent_id]['obj_id']
        # Add team members
        for member in state['World']['team_members']:
            if member!=agent_name and member not in self._teamMembers:
                self._teamMembers.append(member)       
        # Process messages from team members
        self._processMessages(state, self._teamMembers)
        # Update trust beliefs for team members
        #self._trustBlief(self._teamMembers, receivedMessages)

        # CRUCIAL TO NOT REMOVE LINE BELOW!
        self._sendMessage('Our score is ' + str(state['brutus']['score']) +'.', 'Brutus')

        if self._timeLeft - self._counter_value == 1 and self._location == '?' and not self._plotGenerated:
            image_name = "/home/ruben/xai4mhc/TUD-Research-Project-2022/SaR_gui/static/images/sensitivity_plots/plot_at_time_" + str(self._counter_value) + ".svg"
            self._R2PyPlotLocate(self._totalVictimsCat, self._duration, self._counter_value, self._temperatureCat, image_name)
            self._plotGenerated = True
            image_name = "<img src='/static/images" + image_name.split('/static/images')[-1] + "' />"
            self._sendMessage('The location of the fire source still has not been found, so we should decide whether to send in Firefighters to help locate the fire source or if sending them in is too dangerous. \
                            I will make this decision because the predicted moral sensitivity of this situation is below my allocation threshold. This is how much each feature contributed to the predicted sensitivity: \n \n ' \
                            + image_name, 'Brutus')
 
        if self._timeLeft - self._counter_value == 3 and self._location == '?':
            self._sendMessage('Sending in Firefighters to help locate the fire source because the temperature is lower than the auto-ignition temperatures of present substances.', 'Brutus')
            action_kwargs = add_object([(2,4),(9,6),(2,20),(9,18)], "/static/images/rescue-man-final3.svg", 1, 1, 'fighter')
            return AddObject.__name__, action_kwargs
        
        if self._timeLeft - self._counter_value == 4 and self._location == '?':
            for info in state.values():
                if 'obj_id' in info.keys() and 'fighter' in info['obj_id']:
                    return RemoveObject.__name__, {'object_id': info['obj_id'], 'remove_range':500, 'duration_in_ticks':0}
            
        if self._timeLeft - self._counter_value >= 5 and self._location == '?':
            self._sendMessage('Fire source located and pinned on the map.', 'Brutus')
            action_kwargs = add_object([(2,3)], "/images/fire2.svg", 2, 1, 'fire source')
            self._location = '✔' 
            return AddObject.__name__, action_kwargs
        
        if self._timeLeft - self._counter_value == 10:
            if self._location == '?':
                self._locationCat = 'unknown'
            if self._location == '✔':
                self._locationCat = 'known'
            image_name = "/home/ruben/xai4mhc/TUD-Research-Project-2022/SaR_gui/static/images/sensitivity_plots/plot_at_time_" + str(self._counter_value) + ".svg"
            self._R2PyPlotTactic(self._totalVictimsCat, self._locationCat, self._duration, self._counter_value, image_name)
            self._plotGenerated = True
            image_name = "<img src='/static/images" + image_name.split('/static/images')[-1] + "' />"
            self._sendMessage('My offensive inside deployment has been going on for 10 minutes now. We should decide whether to continue with this tactic or switch to a defensive inside deployment. \
                            I will make this decision because the predicted moral sensitivity of this situation is below my allocation threshold. This is how much each feature contributed to the predicted sensitivity: \n \n ' \
                            + image_name, 'Brutus')
                
        if self._timeLeft - self._counter_value == 14:
            image_name = "/home/ruben/xai4mhc/TUD-Research-Project-2022/SaR_gui/static/images/sensitivity_plots/plot_at_time_" + str(self._counter_value) + ".svg"
            self._roomVics = ['mildly injured boy', 'mildly injured girl']
            self._R2PyPlotPriority(len(self._roomVics), self._smoke, self._duration, self._locationCat, image_name)
            self._plotGenerated = True
            image_name = "<img src='/static/images" + image_name.split('/static/images')[-1] + "' />"
            self._sendMessage('I have found two mildly injured victims and fire in office 1. We should decide whether to first extinguish the fire or evacuate the victims. \
                            I will make this decision because the predicted moral sensitivity of this situation is below my allocation threshold. This is how much each feature contributed to the predicted sensitivity: \n \n ' \
                            + image_name, 'Brutus')
            
        if self._timeLeft - self._counter_value == 18 and self._distance != '?':   
            image_name = "/home/ruben/xai4mhc/TUD-Research-Project-2022/SaR_gui/static/images/sensitivity_plots/plot_at_time_" + str(self._counter_value) + ".svg"
            self._R2PyPlotRescue(self._duration, self._counter_value, self._temperatureCat, self._distanceCat)
            self._plotGenerated = True
            self._sendMessage('I have found a heavily injured victim who I cannot evacuate to safety myself. We should decide whether to send in Firefighters to rescue this victim, or if sending them in is too dangerous. \
                              I will make this decision because the predicted moral sensitivity of this situation is below my allocation threshold. This is how much each feature contributed to the predicted sensitivity: \n \n ' \
                              + image_name, 'Brutus')
        else:
            # CONTINUE HERE WHERE WE DON'T WANT THE R CODE TO BE CALLED CONTINUOUSLY DURING THE GAME SECOND & WE WANT TO CALCULATE DISTANCE FOR IF STATEMENT ABOVE
            self.plotGenerated = False

        while True:     
            if Phase.START==self._phase:
                self._phase=Phase.FIND_NEXT_GOAL
                return Idle.__name__,{'duration_in_ticks':50}

            if Phase.FIND_NEXT_GOAL==self._phase:
                self._answered = False
                self._advice = False
                self._goalVic = None
                self._goalLoc = None
                zones = self._getDropZones(state)
                remainingZones = []
                remainingVics = []
                remaining = {}
                for info in zones:
                    if str(info['img_name'])[8:-4] not in self._collectedVictims:
                        remainingZones.append(info)
                        remainingVics.append(str(info['img_name'])[8:-4])
                        remaining[str(info['img_name'])[8:-4]] = info['location']
                if remainingZones:
                    #self._goalVic = str(remainingZones[0]['img_name'])[8:-4]
                    #self._goalLoc = remainingZones[0]['location']
                    self._remainingZones = remainingZones
                    self._remaining = remaining
                if not remainingZones:
                    return None,{}
                self._totalVictims = len(remainingVics) + len(self._collectedVictims)
                if self._totalVictims == 0:
                    self._totalVictimsCat = 'none'
                if self._totalVictims == 1:
                    self._totalVictimsCat = 'one'
                if self._totalVictims == 'unknown':
                    self._totalVictimsCat = 'unclear'
                if self._totalVictims > 1:
                    self._totalVictimsCat = 'multiple'
                self._sendMessage('Victims rescued: ' + str(len(self._collectedVictims)) + '/' + str(self._totalVictims) + '.', 'RescueBot')
                for vic in remainingVics:
                    if vic in self._foundVictims and vic not in self._todo:
                        self._goalVic = vic
                        self._goalLoc = remaining[vic]
                        if 'location' in self._foundVictimLocs[vic].keys():
                            self._sendMessage('Please decide whether it is safe enough to call in a "Fire fighter" to rescue ' + self._goalVic +  ', or whether to "Continue" exploring because it is not safe enough to send in fire fighter. \n \n \
                                Important features to consider: \n - Pinned victims located: ' + str(self._collectedVictims) + '\n - toxic concentrations: ' + str(self._hcn) + ' ppm HCN and ' + str(self._co) + ' ppm CO','Brutus')
                            if self.received_messages_content and self.received_messages_content[-1]=='Continue':
                                self._collectedVictims.append(self._goalVic)
                                self._phase=Phase.FIND_NEXT_GOAL
                        if self.received_messages_content and self.received_messages_content[-1] == 'Fire fighter':
                            #self._sendMessage('Extinguishing fire blocking ' + str(self._door['room_name']) + '.','Brutus')
                            self._phase = Phase.PLAN_PATH_TO_VICTIM
                            return Idle.__name__,{'duration_in_ticks':25}  
                        else:
                            return None,{}
                            #self._phase=Phase.PLAN_PATH_TO_VICTIM
                            #return Idle.__name__,{'duration_in_ticks':25}  
                        #if 'location' not in self._foundVictimLocs[vic].keys():
                        #    self._phase=Phase.PLAN_PATH_TO_ROOM
                        #    return Idle.__name__,{'duration_in_ticks':25}              
                self._phase=Phase.PICK_UNSEARCHED_ROOM
                #return Idle.__name__,{'duration_in_ticks':25}

            if Phase.PICK_UNSEARCHED_ROOM==self._phase:
                self._advice = False
                agent_location = state[self.agent_id]['location']
                unsearchedRooms=[room['room_name'] for room in state.values()
                if 'class_inheritance' in room
                and 'Door' in room['class_inheritance']
                and room['room_name'] not in self._searchedRooms
                and room['room_name'] not in self._tosearch]
                if self._remainingZones and len(unsearchedRooms) == 0:
                    self._tosearch = []
                    self._todo = []
                    self._searchedRooms = []
                    self._sendMessages = []
                    self.received_messages = []
                    self.received_messages_content = []
                    self._searchedRooms.append(self._door['room_name'])
                    self._sendMessage('Going to re-explore all areas.','Brutus')
                    self._phase = Phase.FIND_NEXT_GOAL
                else:
                    if self._currentDoor==None:
                        self._door = state.get_room_doors(self._getClosestRoom(state,unsearchedRooms,agent_location))[0]
                        self._doormat = state.get_room(self._getClosestRoom(state,unsearchedRooms,agent_location))[-1]['doormat']
                        if self._door['room_name'] == 'area 1':
                            self._doormat = (2,4)
                        self._phase = Phase.PLAN_PATH_TO_ROOM
                    if self._currentDoor!=None:
                        self._door = state.get_room_doors(self._getClosestRoom(state,unsearchedRooms,self._currentDoor))[0]
                        self._doormat = state.get_room(self._getClosestRoom(state, unsearchedRooms,self._currentDoor))[-1]['doormat']
                        if self._door['room_name'] == 'area 1':
                            self._doormat = (2,4)
                        self._phase = Phase.PLAN_PATH_TO_ROOM

            if Phase.PLAN_PATH_TO_ROOM==self._phase:
                self._navigator.reset_full()
                if self._goalVic and self._goalVic in self._foundVictims and 'location' not in self._foundVictimLocs[self._goalVic].keys():
                    self._door = state.get_room_doors(self._foundVictimLocs[self._goalVic]['room'])[0]
                    self._doormat = state.get_room(self._foundVictimLocs[self._goalVic]['room'])[-1]['doormat']
                    if self._door['room_name'] == 'area 1':
                        self._doormat = (2,4)
                    #doorLoc = self._door['location']
                    doorLoc = self._doormat
                else:
                    #doorLoc = self._door['location']
                    if self._door['room_name'] == 'area 1':
                        self._doormat = (2,4)
                    doorLoc = self._doormat
                self._navigator.add_waypoints([doorLoc])
                self._tick = state['World']['nr_ticks']
                self._phase=Phase.FOLLOW_PATH_TO_ROOM

            if Phase.FOLLOW_PATH_TO_ROOM==self._phase:
                if self._goalVic and self._goalVic in self._collectedVictims:
                    self._currentDoor=None
                    self._phase=Phase.FIND_NEXT_GOAL
                if self._goalVic and self._goalVic in self._foundVictims and self._door['room_name']!=self._foundVictimLocs[self._goalVic]['room']:
                    self._currentDoor=None
                    self._phase=Phase.FIND_NEXT_GOAL
                # check below
                if self._door['room_name'] in self._searchedRooms and self._goalVic not in self._foundVictims:
                    self._currentDoor=None
                    self._phase=Phase.FIND_NEXT_GOAL
                else:
                    self._state_tracker.update(state)
                    if self._goalVic in self._foundVictims and str(self._door['room_name']) == self._foundVictimLocs[self._goalVic]['room'] and not self._remove:
                        self._sendMessage('Moving to ' + str(self._door['room_name']) + ' to pick up ' + self._goalVic+'.', 'Brutus')                 
                    if self._goalVic not in self._foundVictims and not self._remove or not self._goalVic and not self._remove:
                        self._sendMessage('Moving to ' + str(self._door['room_name']) + ' because it is the closest not explored area.', 'Brutus')                   
                    self._currentDoor=self._door['location']
                    #self._currentDoor=self._doormat
                    action = self._navigator.get_move_action(self._state_tracker)
                    if action!=None:
                        #for info in state.values():
                            #if 'class_inheritance' in info and 'ObstacleObject' in info['class_inheritance'] and 'stone' in info['obj_id'] and info['location'] not in [(9,7),(9,19),(21,19)]:
                            #    self._sendMessage('Found stones blocking my path to ' + str(self._door['room_name']) + '. We can remove them faster if you help me. If you will come here press the "Yes" button, if not press "No".', 'Brutus')
                            #    if self.received_messages_content and self.received_messages_content[-1]=='Yes':
                            #        return None, {}
                            #    if self.received_messages_content and self.received_messages_content[-1]=='No' or state['World']['nr_ticks'] > self._tick + 579:
                            #        self._sendMessage('Removing the stones blocking the path to ' + str(self._door['room_name']) + ' because I want to search this area. We can remove them faster if you help me', 'Brutus')
                                #return RemoveObject.__name__,{'object_id':info['obj_id'],'size':info['visualization']['size']}

                        return action,{}
                    #self._phase=Phase.PLAN_ROOM_SEARCH_PATH
                    self._phase=Phase.REMOVE_OBSTACLE_IF_NEEDED
                    #return Idle.__name__,{'duration_in_ticks':50}         

            if Phase.REMOVE_OBSTACLE_IF_NEEDED==self._phase:
                objects = []
                agent_location = state[self.agent_id]['location']
                for info in state.values():
                    #if 'class_inheritance' in info and 'ObstacleObject' in info['class_inheritance'] and 'fire' in info['obj_id']:
                    if 'class_inheritance' in info and 'ObstacleObject' in info['class_inheritance'] and 'source' in info['obj_id']:
                        objects.append(info)
                        #self._sendMessage('fire is blocking ' + str(self._door['room_name'])+'. \n \n Please decide whether to "Extinguish", "Continue" exploring, or call in a "Fire fighter" to help extinguishing. \n \n \
                        #    Important features to consider: \n - Pinned victims located: ' + str(self._collectedVictims) + '\n - fire temperature: ' + str(int(info['visualization']['size']*300)) + ' degrees Celcius \
                        #    \n - explosion danger: ' + str(info['percentage_lel']) + '% LEL \n - by myself extinguish time: ' + str(int(info['visualization']['size']*7.5)) + ' seconds \n - with help extinguish time: \
                        #    ' + str(int(info['visualization']['size']*3.75)) + ' seconds \n - toxic concentrations: ' + str(self._hcn) + ' ppm HCN and ' + str(self._co) + ' ppm CO','Brutus')
                        self._sendMessage('Found fire source!', 'Brutus')
                        self._location = '✔'
                        self._waiting = True
                        if self.received_messages_content and self.received_messages_content[-1]=='Continue':
                            self._waiting = False
                            self._tosearch.append(self._door['room_name'])
                            self._phase=Phase.FIND_NEXT_GOAL
                        if self.received_messages_content and self.received_messages_content[-1] == 'Extinguish':
                            self._sendMessage('Extinguishing fire blocking ' + str(self._door['room_name']) + ' alone.','Brutus')
                            self._phase = Phase.ENTER_ROOM
                            return RemoveObject.__name__, {'object_id': info['obj_id'],'size':info['visualization']['size']}
                        if self.received_messages_content and self.received_messages_content[-1] == 'Fire fighter':
                            self._sendMessage('Extinguishing fire blocking ' + str(self._door['room_name']) + ' together with fire fighter.','Brutus')
                            self._phase = Phase.BACKUP
                            return Backup.__name__,{'size':info['percentage_lel']}
                        else:
                            return None,{}

                    #if 'class_inheritance' in info and 'Smoke' in info['class_inheritance']:
                    #    self._sendMessage('Smoke detected with ' + info['co_pmm'] + ' ppm CO and ' + info['hcn_ppm'] + ' ppm HCN.','Brutus')
                    #    self._phase=Phase.FIND_NEXT_GOAL

                    if 'class_inheritance' in info and 'ObstacleObject' in info['class_inheritance'] and 'iron' in info['obj_id']:
                        objects.append(info)
                        #self._sendMessage('Iron debris is blocking ' + str(self._door['room_name'])+'. \n \n Please decide whether to "Remove" alone, "Continue" exploring, or call in a "Fire fighter" to help remove. \n \n \
                        #    Important features to consider: \n - Pinned victims located: ' + str(self._collectedVictims) + ' \n - Iron debris weight: ' + str(int(info['weight'])) + ' kilograms \n - by myself removal time: ' \
                        #    + str(int(info['weight']/10)) + ' seconds \n - with help removal time: ' + str(int(info['weight']/20)) + ' seconds \n - toxic concentrations: ' + str(self._hcn) + ' ppm HCN and ' + str(self._co) + ' ppm CO','Brutus')
                        if self._count  < 1:
                            #self._sendMessage('I have found an injured victim who I cannot evacuate to safety myself. \
                            #                  We should decide whether to send in Firefighters to rescue this victim, or if sending them in is too dangerous. \
                            #                  I will make this decision because the predicted moral sensitivity of this situation is below my allocation threshold. \
                            #                  This is how much each feature contributed to the predicted sensitivity: \n \n' \
                            #                  + self._R2PyPlotLocate(self._totalVictimsCat,self._duration,self._counter_value,self._temperatureCat), 'Brutus')
                            self._count+=1
                        self._waiting = True
                        if self.received_messages_content and self.received_messages_content[-1]=='Continue':
                            self._waiting = False
                            self._tosearch.append(self._door['room_name'])
                            self._phase=Phase.FIND_NEXT_GOAL
                        if self.received_messages_content and self.received_messages_content[-1] == 'Remove':
                            self._sendMessage('Removing iron debris blocking ' + str(self._door['room_name']) + ' alone.','Brutus')
                            self._phase = Phase.ENTER_ROOM
                            return RemoveObject.__name__, {'object_id': info['obj_id'],'size':info['visualization']['size']}
                        if self.received_messages_content and self.received_messages_content[-1] == 'Fire fighter':
                            self._sendMessage('Removing iron debris blocking ' + str(self._door['room_name']) + ' together with fire fighter.','Brutus')
                            self._phase = Phase.BACKUP
                            return Backup.__name__,{}
                        else:
                            return None,{}

                if len(objects)==0:                    
                    #self._sendMessage('No need to clear the entrance of ' + str(self._door['room_name']) + ' because it is not blocked by obstacles.','Brutus')
                    self._answered = False
                    self._remove = False
                    self._phase = Phase.ENTER_ROOM


            if Phase.BACKUP==self._phase:
                for info in state.values():
                    if 'class_inheritance' in info and 'ObstacleObject' in info['class_inheritance'] and 'fire' in info['obj_id']:
                        if info['percentage_lel'] > 10 or self._co > 500 or self._hcn > 40:
                            self._sendMessage('fire at ' + str(self._door['room_name']) + ' too dangerous for fire fighter!! \n \n Going to abort extinguishing.','Brutus')
                            self._searchedRooms.append(self._door['room_name'])
                            self._sendMessages = []
                            self.received_messages = []
                            self.received_messages_content = []
                            self._phase = Phase.FIND_NEXT_GOAL
                            return Injured.__name__,{'duration_in_ticks':50}
                        else:
                            self._phase = Phase.ENTER_ROOM
                            return RemoveObjectTogether.__name__, {'object_id': info['obj_id'], 'size':info['visualization']['size']}
                    if 'class_inheritance' in info and 'ObstacleObject' in info['class_inheritance'] and 'iron' in info['obj_id']:
                        if self._co > 500 or self._hcn > 40:
                            self._sendMessage('Situation at ' + str(self._door['room_name']) + ' too dangerous for fire fighter!! \n \n Going to abort removing iron debris.','Brutus')
                            self._searchedRooms.append(self._door['room_name'])
                            self._sendMessages = []
                            self.received_messages = []
                            self.received_messages_content = []
                            self._phase = Phase.FIND_NEXT_GOAL
                            return Injured.__name__,{'duration_in_ticks':50}
                        else:
                            self._phase = Phase.ENTER_ROOM
                            return RemoveObjectTogether.__name__, {'object_id': info['obj_id'], 'size':info['visualization']['size']}

            if Phase.BACKUP2==self._phase:
                self._goalVic = None
                self._goalLoc = None
                zones = self._getDropZones(state)
                remainingZones = []
                remainingVics = []
                remaining = {}
                for info in zones:
                    if str(info['img_name'])[8:-4] not in self._collectedVictims:
                        remainingZones.append(info)
                        remainingVics.append(str(info['img_name'])[8:-4])
                        remaining[str(info['img_name'])[8:-4]] = info['location']
                if remainingZones:
                    #self._goalVic = str(remainingZones[0]['img_name'])[8:-4]
                    #self._goalLoc = remainingZones[0]['location']
                    self._remainingZones = remainingZones
                    self._remaining = remaining
                if not remainingZones:
                    return None,{}

                for vic in remainingVics:
                    if vic in self._foundVictims and vic not in self._todo:
                        self._goalVic = vic
                        self._goalLoc = remaining[vic]
                for info in state.values():
                    if 'class_inheritance' in info and 'ObstacleObject' in info['class_inheritance'] and 'fire' in info['obj_id']:
                        if info['percentage_lel'] > 10 or self._co > 500 or self._hcn > 40:
                            self._sendMessage('fire at ' + str(self._door['room_name']) + ' too dangerous for fire fighter!! \n \n Going to abort extinguishing.','Brutus')
                            self._searchedRooms.append(self._door['room_name'])
                            self._collectedVictims.append(self._goalVic)
                            self._sendMessages = []
                            self.received_messages = []
                            self.received_messages_content = []
                            self._phase = Phase.FIND_NEXT_GOAL
                            return Injured.__name__,{'duration_in_ticks':50}
                        else:
                            self._decided = True
                            self._phase = Phase.PLAN_PATH_TO_VICTIM
                            return RemoveObjectTogether.__name__, {'object_id': info['obj_id'], 'size':info['visualization']['size']}
                    
            if Phase.ENTER_ROOM==self._phase:
                self._answered = False
                if self._goalVic in self._collectedVictims:
                    self._currentDoor=None
                    self._phase=Phase.FIND_NEXT_GOAL
                if self._goalVic in self._foundVictims and self._door['room_name']!=self._foundVictimLocs[self._goalVic]['room']:
                    self._currentDoor=None
                    self._phase=Phase.FIND_NEXT_GOAL
                if self._door['room_name'] in self._searchedRooms and self._goalVic not in self._foundVictims:
                    self._currentDoor=None
                    self._phase=Phase.FIND_NEXT_GOAL
                else:
                    self._state_tracker.update(state)                 
                    #self._currentDoor=self._door['location']
                    #self._currentDoor=self._door
                    action = self._navigator.get_move_action(self._state_tracker)
                    if action!=None:
                        return action,{}
                    self._phase=Phase.PLAN_ROOM_SEARCH_PATH
                    #self._phase=Phase.REMOVE_OBSTACLE_IF_NEEDED
                    #return Idle.__name__,{'duration_in_ticks':50} 


            if Phase.PLAN_ROOM_SEARCH_PATH==self._phase:
                self._agentLoc = int(self._door['room_name'].split()[-1])
                roomTiles = [info['location'] for info in state.values()
                    if 'class_inheritance' in info 
                    and 'AreaTile' in info['class_inheritance']
                    and 'room_name' in info
                    and info['room_name'] == self._door['room_name']
                ]
                self._roomtiles=roomTiles               
                self._navigator.reset_full()
                self._navigator.add_waypoints(roomTiles)
                #self._sendMessage('Searching through whole ' + str(self._door['room_name']) + ' because my sense range is limited and to find victims.', 'Brutus')
                #self._currentDoor = self._door['location']
                self._roomVics=[]
                self._phase=Phase.FOLLOW_ROOM_SEARCH_PATH
                #return Idle.__name__,{'duration_in_ticks':50}

            if Phase.FOLLOW_ROOM_SEARCH_PATH==self._phase:
                self._state_tracker.update(state)
                action = self._navigator.get_move_action(self._state_tracker)
                if action!=None:                   
                    for info in state.values():
                        if 'class_inheritance' in info and 'CollectableBlock' in info['class_inheritance']:
                            vic = str(info['img_name'][8:-4])
                            if vic not in self._roomVics:
                                self._roomVics.append(vic)

                            if vic in self._foundVictims and 'location' not in self._foundVictimLocs[vic].keys():
                                self._foundVictimLocs[vic] = {'location':info['location'],'room':self._door['room_name'],'obj_id':info['obj_id']}
                                if vic == self._goalVic:
                                    self._sendMessage('Found '+ vic + ' in ' + self._door['room_name'] + ' because you told me '+vic+ ' was located here.', 'Brutus')
                                    self._searchedRooms.append(self._door['room_name'])
                                    self._phase=Phase.FIND_NEXT_GOAL

                            if 'healthy' not in vic and vic not in self._foundVictims:
                                self._recentVic = vic
                                self._foundVictims.append(vic)
                                self._foundVictimLocs[vic] = {'location':info['location'],'room':self._door['room_name'],'obj_id':info['obj_id']}
                                self._sendMessage('Found ' + vic + ' in ' + self._door['room_name'] + '.','Brutus')

                        if 'class_inheritance' in info and 'ObstacleObject' in info['class_inheritance'] and 'fire' in info['obj_id']:
                            self._sendMessage('Detected fire in ' + self._door['room_name'] + '. \n \n Please decide whether to call in a "Fire fighter" to help extinguishing and rescue ' + self._recentVic + ', or "Continue" exploring because it is not safe enough to send in fire fighter. \n \n \
                            Important features to consider: \n - Pinned victims located: ' + str(self._collectedVictims) + '\n - fire temperature: ' + str(int(info['visualization']['size']*300)) + ' degrees Celcius \
                            \n - explosion danger: ' + str(info['percentage_lel']) + '% LEL \n - toxic concentrations: ' + str(self._hcn) + ' ppm HCN and ' + str(self._co) + ' ppm CO','Brutus')
                            self._waiting = True
                            if self.received_messages_content and self.received_messages_content[-1]=='Continue':
                                self._waiting = False
                                self._tosearch.append(self._door['room_name'])
                                self._phase=Phase.FIND_NEXT_GOAL
                            #if self.received_messages_content and self.received_messages_content[-1] == 'Remove':
                            #    self._sendMessage('Extinguishing fire blocking ' + str(self._door['room_name']) + '.','Brutus')
                            #    return RemoveObject.__name__, {'object_id': info['obj_id']}
                            if self.received_messages_content and self.received_messages_content[-1] == 'Fire fighter':
                                self._sendMessage('Extinguishing fire in ' + str(self._door['room_name']) + ' together with fire fighter.','Brutus')
                                #self._goalVic = self._recentVic
                                #self._goalLoc = self._foundVictimLocs[self._goalVic]['location']
                                self._phase = Phase.BACKUP2
                                return Backup.__name__,{'size':info['percentage_lel']}
                            else:
                                return None, {}

                    return action,{}
                #if self._goalVic not in self._foundVictims:
                #    self._sendMessage(self._goalVic + ' not present in ' + str(self._door['room_name']) + ' because I searched the whole area without finding ' + self._goalVic, 'Brutus')
                if self._goalVic in self._foundVictims and self._goalVic not in self._roomVics and self._foundVictimLocs[self._goalVic]['room']==self._door['room_name']:
                    self._sendMessage(self._goalVic + ' not present in ' + str(self._door['room_name']) + ' because I searched the whole area without finding ' + self._goalVic+'.', 'Brutus')
                    self._foundVictimLocs.pop(self._goalVic, None)
                    self._foundVictims.remove(self._goalVic)
                    self._roomVics = []
                    self.received_messages = []
                    self.received_messages_content = []
                self._searchedRooms.append(self._door['room_name'])
                self._phase=Phase.FIND_NEXT_GOAL
                return Idle.__name__,{'duration_in_ticks':25}
                
            if Phase.PLAN_PATH_TO_VICTIM==self._phase:
                if 'mild' in self._goalVic:
                    self._sendMessage('fire fighter will transport ' + self._goalVic + ' in ' + self._foundVictimLocs[self._goalVic]['room'] + ' to safe zone.', 'Brutus')
                self._searchedRooms.append(self._door['room_name'])
                self._navigator.reset_full()
                self._navigator.add_waypoints([self._foundVictimLocs[self._goalVic]['location']])
                self._phase=Phase.FOLLOW_PATH_TO_VICTIM
                #return Idle.__name__,{'duration_in_ticks':50}
                    
            if Phase.FOLLOW_PATH_TO_VICTIM==self._phase:
                if self._goalVic and self._goalVic in self._collectedVictims:
                    self._phase=Phase.FIND_NEXT_GOAL
                else:
                    self._state_tracker.update(state)
                    action=self._navigator.get_move_action(self._state_tracker)
                    if action!=None:
                        return action,{}
                    #if action==None and 'critical' in self._goalVic:
                    #    return MoveNorth.__name__, {}
                    self._phase=Phase.TAKE_VICTIM
                    
            if Phase.TAKE_VICTIM==self._phase:
                objects=[]
                for info in state.values():
                    if 'class_inheritance' in info and 'CollectableBlock' in info['class_inheritance'] and 'critical' in info['obj_id'] and info['location'] in self._roomtiles:
                        objects.append(info)
                        #self._sendMessage('Please come to ' + str(self._door['room_name']) + ' because we need to carry ' + str(self._goalVic) + ' together.', 'Brutus')
                        self._collectedVictims.append(self._goalVic)
                        self._phase=Phase.INTRO4
                        if not 'Human' in info['name']:
                            return None, {} 
                if len(objects)==0 and 'critical' in self._goalVic:
                    self._criticalRescued+=1
                    self._collectedVictims.append(self._goalVic)
                    self._phase = Phase.PLAN_PATH_TO_DROPPOINT
                if 'mild' in self._goalVic:
                    self._phase=Phase.PLAN_PATH_TO_DROPPOINT
                    self._collectedVictims.append(self._goalVic)
                    return CarryObject.__name__,{'object_id':self._foundVictimLocs[self._goalVic]['obj_id']}                

            if Phase.PLAN_PATH_TO_DROPPOINT==self._phase:
                self._navigator.reset_full()
                self._navigator.add_waypoints([self._goalLoc])
                self._phase=Phase.FOLLOW_PATH_TO_DROPPOINT

            if Phase.FOLLOW_PATH_TO_DROPPOINT==self._phase:
                #self._sendMessage('Transporting '+ self._goalVic + ' to the drop zone because ' + self._goalVic + ' should be delivered there for further treatment.', 'Brutus')
                self._state_tracker.update(state)
                action=self._navigator.get_move_action(self._state_tracker)
                if action!=None:
                    return action,{}
                self._phase=Phase.DROP_VICTIM
                #return Idle.__name__,{'duration_in_ticks':50}  

            if Phase.DROP_VICTIM == self._phase:
                if 'mild' in self._goalVic:
                    self._sendMessage('fire fighter delivered '+ self._goalVic + ' at the safe zone.', 'Brutus')
                self._phase=Phase.FIND_NEXT_GOAL
                self._currentDoor = None
                self._tick = state['World']['nr_ticks']
                return Drop.__name__,{}

            
    def _getDropZones(self,state:State):
        '''
        @return list of drop zones (their full dict), in order (the first one is the
        the place that requires the first drop)
        '''
        places=state[{'is_goal_block':True}]
        places.sort(key=lambda info:info['location'][1])
        zones = []
        for place in places:
            if place['drop_zone_nr']==0:
                zones.append(place)
        return zones

    def _processMessages(self, state, teamMembers):
        '''
        process incoming messages. 
        Reported blocks are added to self._blocks
        '''
        receivedMessages = {}
        for member in teamMembers:
            receivedMessages[member] = []
        for mssg in self.received_messages:
            for member in teamMembers:
                if mssg.from_id == member:
                    receivedMessages[member].append(mssg.content) 
        #areas = ['area A1','area A2','area A3','area A4','area B1','area B2','area C1','area C2','area C3']
        for mssgs in receivedMessages.values():
            for msg in mssgs:
                if msg.startswith("Search:"):
                    area = 'area '+ msg.split()[-1]
                    if area not in self._searchedRooms:
                        self._searchedRooms.append(area)
                if msg.startswith("Found:"):
                    if len(msg.split()) == 6:
                        foundVic = ' '.join(msg.split()[1:4])
                    else:
                        foundVic = ' '.join(msg.split()[1:5]) 
                    loc = 'area '+ msg.split()[-1]
                    if loc not in self._searchedRooms:
                        self._searchedRooms.append(loc)
                    if foundVic not in self._foundVictims:
                        self._foundVictims.append(foundVic)
                        self._foundVictimLocs[foundVic] = {'room':loc}
                    if foundVic in self._foundVictims and self._foundVictimLocs[foundVic]['room'] != loc:
                        self._foundVictimLocs[foundVic] = {'room':loc}
                    if 'mild' in foundVic:
                        self._todo.append(foundVic)
                if msg.startswith('Collect:'):
                    if len(msg.split()) == 6:
                        collectVic = ' '.join(msg.split()[1:4])
                    else:
                        collectVic = ' '.join(msg.split()[1:5]) 
                    loc = 'area ' + msg.split()[-1]
                    if loc not in self._searchedRooms:
                        self._searchedRooms.append(loc)
                    if collectVic not in self._foundVictims:
                        self._foundVictims.append(collectVic)
                        self._foundVictimLocs[collectVic] = {'room':loc}
                    if collectVic in self._foundVictims and self._foundVictimLocs[collectVic]['room'] != loc:
                        self._foundVictimLocs[collectVic] = {'room':loc}
                    if collectVic not in self._collectedVictims:
                        self._collectedVictims.append(collectVic)
                if msg.startswith('Remove:'):
                    # add sending messages about it
                    area = 'area ' + msg.split()[-1]
                    self._door = state.get_room_doors(area)[0]
                    self._doormat = state.get_room(area)[-1]['doormat']
                    if area in self._searchedRooms:
                        self._searchedRooms.remove(area)
                    self.received_messages = []
                    self.received_messages_content = []
                    self._remove = True
                    self._sendMessage('Moving to ' + str(self._door['room_name']) + ' to help you remove an obstacle.', 'Brutus')  
                    self._phase = Phase.PLAN_PATH_TO_ROOM
            if mssgs and mssgs[-1].split()[-1] in ['1','2','3','4','5','6','7','8','9','10','11','12','13','14']:
                self._humanLoc = int(mssgs[-1].split()[-1])

            #if msg.startswith('Mission'):
            #    self._sendMessage('Unsearched areas: '  + ', '.join([i.split()[1] for i in areas if i not in self._searchedRooms]) + '. Collected victims: ' + ', '.join(self._collectedVictims) +
            #    '. Found victims: ' +  ', '.join([i + ' in ' + self._foundVictimLocs[i]['room'] for i in self._foundVictimLocs]) ,'Brutus')
            #    self.received_messages=[]

    def _sendMessage(self, mssg, sender):
        msg = Message(content=mssg, from_id=sender)
        if msg.content not in self.received_messages_content and 'Our score is' not in msg.content and 'Time left:' not in msg.content and 'Fire duration:' not in msg.content \
        and 'Victims rescued:' not in msg.content and 'Smoke spreads:' not in msg.content and 'Temperature:' not in msg.content and 'Location:' not in msg.content and 'Distance:' not in msg.content:
            self.send_message(msg)
            self._sendMessages.append(msg.content)
        # Sending the hidden score message (DO NOT REMOVE)
        if 'Our score is' in msg.content or 'Time left:' in msg.content or 'Fire duration' in msg.content or 'Victims rescued' in msg.content or 'Smoke spreads' in msg.content \
        or 'Temperature:' in msg.content or 'Location:' in msg.content or 'Distance' in msg.content:
            self.send_message(msg)

        #if self.received_messages and self._sendMessages:
        #    self._last_mssg = self._sendMessages[-1]
        #    if self._last_mssg.startswith('Searching') or self._last_mssg.startswith('Moving'):
        #        self.received_messages=[]
        #        self.received_messages.append(self._last_mssg)

    def _getClosestRoom(self, state, objs, currentDoor):
        agent_location = state[self.agent_id]['location']
        locs = {}
        for obj in objs:
            locs[obj]=state.get_room_doors(obj)[0]['location']
        dists = {}
        for room,loc in locs.items():
            if currentDoor!=None:
                dists[room]=utils.get_distance(currentDoor,loc)
            if currentDoor==None:
                dists[room]=utils.get_distance(agent_location,loc)

        return min(dists,key=dists.get)
    
    def _R2PyPlotPriority(self, people, smoke, duration, location, image_name):
        r_script = (f'''
                    data <- read_excel("/home/ruben/Downloads/moral sensitivity survey data 4.xlsx")
                    data$situation <- as.factor(data$situation)
                    data$location <- as.factor(data$location)
                    data$smoke <- as.factor(data$smoke)
                    data_s3 <- subset(data, data$situation=="3"|data$situation=="6")
                    data_s3 <- data_s3[data_s3$smoke != "pushing out",]
                    data_s3$people <- as.numeric(data_s3$people)
                    fit <- lm(sensitivity ~ people + duration + smoke + location, data = data_s3[-c(244,242,211,162,96,92,29),])
                    pred_data3 <- subset(data_s3[-c(244,242,211,162,96,92,29),], select = c("people", "duration", "smoke", "location", "sensitivity"))
                    pred_data3$smoke <- factor(pred_data3$smoke, levels = c("fast", "normal", "slow"))
                    explainer <- shapr(pred_data3, fit)
                    p <- mean(pred_data3$sensitivity)
                    new_data3 <- data.frame(people = c({people}),
                                        duration = c({duration}),
                                        smoke = c("{smoke}"),
                                        location = c("{location}"))
                    new_data3$smoke <- factor(new_data3$smoke, levels = c("fast", "normal", "slow"))
                    new_data3$location <- factor(new_data3$location, levels = c("known", "unknown"))
                    new_pred <- predict(fit, new_data3)
                    explanation_cat <- shapr::explain(new_data3, approach = "ctree", explainer = explainer, prediction_zero = p)

                    # Shapley values
                    shapley_values <- explanation_cat[["dt"]][,2:5]

                    # Standardize Shapley values
                    standardized_values <- shapley_values / sum(abs(shapley_values))
                    explanation_cat[["dt"]][,2:5] <- standardized_values
                    
                    pl <- plot(explanation_cat, digits = 1, plot_phi0 = FALSE) 
                    pl[["data"]]$header <- paste("predicted sensitivity = ", round(new_pred, 1), sep = " ")
                    data_plot <- pl[["data"]]
                    min <- 'min.'
                    loc <- NA
                    if ("{location}" == 'known') {{
                        loc <- '✔'
                    }}
                    if ("{location}" == 'unknown') {{
                        loc <- '?'
                    }}
                    labels <- c(duration = paste("<img src='/home/ruben/xai4mhc/Icons/duration_fire_black.png' width='38' /><br>\n", new_data3$duration, min), 
                    smoke = paste("<img src='/home/ruben/xai4mhc/Icons/smoke_speed_black.png' width='65' /><br>\n", new_data3$smoke), 
                    location = paste("<img src='/home/ruben/xai4mhc/Icons/location_fire_black.png' width='43' /><br>\n", loc), 
                    people = paste("<img src='/home/ruben/xai4mhc/Icons/victims.png' width='24' /><br>\n", new_data3$people))
                    data_plot$variable <- reorder(data_plot$variable, -abs(data_plot$phi))
                    pl <- ggplot(data_plot, aes(x = variable, y = phi, fill = ifelse(phi >= 0, "positive", "negative"))) + geom_bar(stat = "identity") + scale_x_discrete(name = NULL, labels = labels) + theme(axis.text.x = ggtext::element_markdown(color = "black", size = 15)) + theme(text=element_text(size = 15, family="Roboto"),plot.title=element_text(hjust=0.5,size=15,color="black",face="bold",margin = margin(b=5)),
                    plot.caption = element_text(size=15,margin = margin(t=25),color="black"),
                    panel.background = element_blank(),
                    axis.text = element_text(size=15,colour = "black"),axis.text.y = element_text(colour = "black",margin = margin(t=5)),
                    axis.line = element_line(colour = "black"), axis.title = element_text(size=15), axis.title.y = element_text(colour = "black",margin = margin(r=10),hjust = 0.5),
                    axis.title.x = element_text(colour = "black", margin = margin(t=5),hjust = 0.5), panel.grid.major = element_line(color="#DAE1E7"), panel.grid.major.x = element_blank()) + theme(legend.background = element_rect(fill="white",colour = "white"),legend.key = element_rect(fill="white",colour = "white"), legend.text = element_text(size=15),
                    legend.position ="none",legend.title = element_text(size=15,face = "plain")) + ggtitle(paste("Predicted sensitivity = ", round(new_pred, 1))) + labs(y="Relative feature contribution", fill="") + scale_y_continuous(breaks=seq(-1,1,by=0.5), limits=c(-1,1), expand=c(0.0,0.0)) + scale_fill_manual(values = c("positive" = "#3E6F9F", "negative" = "#B0D7F0"), breaks = c("positive","negative")) + geom_hline(yintercept = 0, color = "black") + theme(axis.text = element_text(color = "black"),
                    axis.ticks = element_line(color = "black"))
                    dpi_web <- 300
                    width_pixels <- 1600
                    height_pixels <- 1600
                    width_inches_web <- width_pixels / dpi_web
                    height_inches_web <- height_pixels / dpi_web
                    ggsave(filename="{image_name}", plot=pl, width=width_inches_web, height=height_inches_web, dpi=dpi_web)
                    ''')
        robjects.r(r_script)


    def _R2PyPlotTactic(self, people, location, duration, resistance, image_name):
        r_script = (f'''
                    data <- read_excel("/home/ruben/Downloads/moral sensitivity survey data 4.xlsx")
                    data$situation <- as.factor(data$situation)
                    data$location <- as.factor(data$location)
                    data_s4 <- subset(data, data$situation=="5"|data$situation=="7")
                    data_s4$people[data_s4$people == "0"] <- "none"
                    data_s4$people[data_s4$people == "1"] <- "one"
                    data_s4$people[data_s4$people == "10" |data_s4$people == "11" |data_s4$people == "2" |data_s4$people == "3" |data_s4$people == "4" |data_s4$people == "5"] <- "multiple"
                    data_s4 <- data_s4[data_s4$people != "clear",]
                    data_s4$people <- factor(data_s4$people, levels = c("none","unclear","one","multiple"))
                    fit <- lm(sensitivity ~ people + duration + resistance + location, data = data_s4[-c(266,244,186,178,126,111,97,44,19),])
                    pred_data4 <- subset(data_s4[-c(266,244,186,178,126,111,97,44,19),], select = c("people", "duration", "resistance", "location", "sensitivity"))
                    explainer <- shapr(pred_data4, fit)
                    p <- mean(pred_data4$sensitivity)
                    new_data4 <- data.frame(people = c("{people}"),
                                            duration = c({duration}),
                                            resistance = c({resistance}),
                                            location = c("{location}"))
                    new_data4$people <- factor(new_data4$people, levels = c("none", "unclear", "one", "multiple"))
                    new_data4$location <- factor(new_data4$location, levels = c("known", "unknown"))
                    new_pred <- predict(fit, new_data4)
                    explanation_cat <- shapr::explain(new_data4, approach = "ctree", explainer = explainer, prediction_zero = p)
                    # Shapley values
                    shapley_values <- explanation_cat[["dt"]][,2:5]

                    # Standardize Shapley values
                    standardized_values <- shapley_values / sum(abs(shapley_values))
                    explanation_cat[["dt"]][,2:5] <- standardized_values
                    
                    pl <- plot(explanation_cat, digits = 1, plot_phi0 = FALSE) 
                    pl[["data"]]$header <- paste("predicted sensitivity = ", round(new_pred, 1), sep = " ")
                    data_plot <- pl[["data"]]
                    min <- 'min.'
                    loc <- NA
                    if ("{location}" == 'known') {{
                        loc <- '✔'
                    }}
                    if ("{location}" == 'unknown') {{
                        loc <- '?'
                    }}
                    labels <- c(duration = paste("<img src='/home/ruben/xai4mhc/Icons/duration_fire_black.png' width='38' /><br>\n", new_data4$duration, min), 
                    resistance = paste("<img src='/home/ruben/xai4mhc/Icons/fire_resistance_black.png' width='47' /><br>\n", new_data4$resistance, min), 
                    location = paste("<img src='/home/ruben/xai4mhc/Icons/location_fire_black.png' width='43' /><br>\n", loc), 
                    people = paste("<img src='/home/ruben/xai4mhc/Icons/victims.png' width='24' /><br>\n", new_data4$people))
                    data_plot$variable <- reorder(data_plot$variable, -abs(data_plot$phi))
                    pl <- ggplot(data_plot, aes(x = variable, y = phi, fill = ifelse(phi >= 0, "positive", "negative"))) + geom_bar(stat = "identity") + scale_x_discrete(name = NULL, labels = labels) + theme(axis.text.x = ggtext::element_markdown(color = "black", size = 15)) + theme(text=element_text(size = 15, family="Roboto"),plot.title=element_text(hjust=0.5,size=15,color="black",face="bold",margin = margin(b=5)),
                    plot.caption = element_text(size=15,margin = margin(t=25),color="black"),
                    panel.background = element_blank(),
                    axis.text = element_text(size=15,colour = "black"),axis.text.y = element_text(colour = "black",margin = margin(t=5)),
                    axis.line = element_line(colour = "black"), axis.title = element_text(size=15), axis.title.y = element_text(colour = "black",margin = margin(r=10),hjust = 0.5),
                    axis.title.x = element_text(colour = "black", margin = margin(t=5),hjust = 0.5), panel.grid.major = element_line(color="#DAE1E7"), panel.grid.major.x = element_blank()) + theme(legend.background = element_rect(fill="white",colour = "white"),legend.key = element_rect(fill="white",colour = "white"), legend.text = element_text(size=15),
                    legend.position ="none",legend.title = element_text(size=15,face = "plain")) + ggtitle(paste("Predicted sensitivity = ", round(new_pred, 1))) + labs(y="Relative feature contribution", fill="") + scale_y_continuous(breaks=seq(-1,1,by=0.5), limits=c(-1,1), expand=c(0.0,0.0)) + scale_fill_manual(values = c("positive" = "#3E6F9F", "negative" = "#B0D7F0"), breaks = c("positive","negative")) + geom_hline(yintercept = 0, color = "black") + theme(axis.text = element_text(color = "black"),
                    axis.ticks = element_line(color = "black"))
                    dpi_web <- 300
                    width_pixels <- 1600
                    height_pixels <- 1600
                    width_inches_web <- width_pixels / dpi_web
                    height_inches_web <- height_pixels / dpi_web
                    ggsave(filename="{image_name}", plot=pl, width=width_inches_web, height=height_inches_web, dpi=dpi_web)
                    ''')
        robjects.r(r_script)

    
    def _R2PyPlotLocate(self, people, duration, resistance, temperature, image_name):
        r_script = (f'''
                    data <- read_excel("/home/ruben/Downloads/moral sensitivity survey data 4.xlsx")
                    data$situation <- as.factor(data$situation)
                    data$temperature <- as.factor(data$temperature)
                    # CORRECT! PREDICT SENSITIVITY IN SITUATION 'SEND IN FIREFIGHTERS TO LOCATE FIRE OR NOT' BASED ON DURATION, RESISTANCE, TEMPERATURE, AND PEOPLE'
                    data_s2 <- subset(data, data$situation=="2"|data$situation=="4")
                    data_s2$people[data_s2$people == "0"] <- "none"
                    data_s2$people[data_s2$people == "1"] <- "one"
                    data_s2$people[data_s2$people == "10" |data_s2$people == "11" |data_s2$people == "2" |data_s2$people == "3" |data_s2$people == "4" |data_s2$people == "40" |data_s2$people == "5"] <- "multiple"
                    data_s2 <- data_s2[data_s2$people != "clear",]
                    data_s2$people <- factor(data_s2$people, levels = c("none","unclear","one","multiple"))
                    data_s2 <- data_s2 %>% drop_na(duration)
                    fit <- lm(sensitivity ~ people + duration + resistance + temperature, data = data_s2[-c(220,195,158,126,121,76),])
                    # SHAP explanations
                    pred_data2 <- subset(data_s2[-c(220,195,158,126,121,76),], select = c("people", "duration", "resistance", "temperature", "sensitivity"))
                    explainer <- shapr(pred_data2, fit)
                    p <- mean(pred_data2$sensitivity)
                    new_data2 <- data.frame(resistance = c({resistance}),
                                            temperature = c("{temperature}"),
                                            people = c("{people}"),
                                            duration = c({duration}))
                    new_data2$temperature <- factor(new_data2$temperature, levels = c("close", "higher", "lower"))
                    new_data2$people <- factor(new_data2$people, levels = c("none", "unclear", "one", "multiple"))
                    
                    new_pred <- predict(fit, new_data2)
                    explanation_cat <- shapr::explain(new_data2, approach = "ctree", explainer = explainer, prediction_zero = p)

                    # Shapley values
                    shapley_values <- explanation_cat[["dt"]][,2:5]

                    # Standardize Shapley values
                    standardized_values <- shapley_values / sum(abs(shapley_values))
                    explanation_cat[["dt"]][,2:5] <- standardized_values
                    
                    pl <- plot(explanation_cat, digits = 1, plot_phi0 = FALSE) 
                    pl[["data"]]$header <- paste("predicted sensitivity = ", round(new_pred, 1), sep = " ")
                    data_plot <- pl[["data"]]
                    min <- 'min.'
                    temp <- NA
                    if ("{temperature}" == 'close') {{
                        temp <- '<≈ thresh.'
                    }}
                    if ("{temperature}" == 'lower') {{
                        temp <- '< thresh.'
                    }}
                    if ("{temperature}" == 'higher') {{
                        temp <- '> thresh.'
                    }}
                    labels <- c(duration = paste("<img src='/home/ruben/xai4mhc/Icons/duration_fire_black.png' width='38' /><br>\n", new_data2$duration, min), 
                    resistance = paste("<img src='/home/ruben/xai4mhc/Icons/fire_resistance_black.png' width='47' /><br>\n", new_data2$resistance, min), 
                    temperature = paste("<img src='/home/ruben/xai4mhc/Icons/celsius_transparent.png' width='53' /><br>\n", temp), 
                    people = paste("<img src='/home/ruben/xai4mhc/Icons/victims.png' width='24' /><br>\n", new_data2$people))
                    data_plot$variable <- reorder(data_plot$variable, -abs(data_plot$phi))
                    pl <- ggplot(data_plot, aes(x = variable, y = phi, fill = ifelse(phi >= 0, "positive", "negative"))) + geom_bar(stat = "identity") + scale_x_discrete(name = NULL, labels = labels) + theme(axis.text.x = ggtext::element_markdown(color = "black", size = 15)) + theme(text=element_text(size = 15, family="Roboto"),plot.title=element_text(hjust=0.5,size=15,color="black",face="bold",margin = margin(b=5)),
                    plot.caption = element_text(size=15,margin = margin(t=25),color="black"),
                    panel.background = element_blank(),
                    axis.text = element_text(size=15,colour = "black"),axis.text.y = element_text(colour = "black",margin = margin(t=5)),
                    axis.line = element_line(colour = "black"), axis.title = element_text(size=15), axis.title.y = element_text(colour = "black",margin = margin(r=10),hjust = 0.5),
                    axis.title.x = element_text(colour = "black", margin = margin(t=5),hjust = 0.5), panel.grid.major = element_line(color="#DAE1E7"), panel.grid.major.x = element_blank()) + theme(legend.background = element_rect(fill="white",colour = "white"),legend.key = element_rect(fill="white",colour = "white"), legend.text = element_text(size=15),
                    legend.position ="none",legend.title = element_text(size=15,face = "plain")) + ggtitle(paste("Predicted sensitivity = ", round(new_pred, 1))) + labs(y="Relative feature contribution", fill="") + scale_y_continuous(breaks=seq(-1,1,by=0.5), limits=c(-1,1), expand=c(0.0,0.0)) + scale_fill_manual(values = c("positive" = "#3E6F9F", "negative" = "#B0D7F0"), breaks = c("positive","negative")) + geom_hline(yintercept = 0, color = "black") + theme(axis.text = element_text(color = "black"),
                    axis.ticks = element_line(color = "black"))
                    dpi_web <- 300
                    width_pixels <- 1600
                    height_pixels <- 1600
                    width_inches_web <- width_pixels / dpi_web
                    height_inches_web <- height_pixels / dpi_web
                    ggsave(filename="{image_name}", plot=pl, width=width_inches_web, height=height_inches_web, dpi=dpi_web)
                    ''')
        robjects.r(r_script)

    def _R2PyPlotRescue(self, duration, resistance, temperature, distance):
        r_script = (f'''
                    # CORRECT! PREDICT SENSITIVITY IN SITUATION 'SEND IN FIREFIGHTERS TO RESCUE OR NOT' BASED ON FIRE DURATION, FIRE RESISTANCE, TEMPERATURE WRT AUTO-IGNITION, AND DISTANCE VICTIM - FIRE 
                    data <- read_excel("/home/ruben/Downloads/moral sensitivity survey data 4.xlsx")
                    data$bin <- as.factor(data$bin)
                    data$gender <- as.factor(data$gender)
                    data$age <- as.factor(data$age)
                    data$culture <- as.factor(data$culture)
                    data$education <- as.factor(data$education)
                    data$discipline <- as.factor(data$discipline)
                    data$situation <- as.factor(data$situation)
                    data$temperature <- as.factor(data$temperature)
                    data$distance <- as.factor(data$distance)
                    data$smoke <- as.factor(data$smoke)
                    data$location <- as.factor(data$location)
                    data_subset <- subset(data, data$situation=="1"|data$situation=="8")
                    data_subset$people <- as.numeric(data_subset$people)
                    data_subset <- subset(data_subset, (!data_subset$temperature=="close"))
                    data_subset <- data_subset %>% drop_na(distance)
                    fit <- lm(sensitivity ~ duration + resistance + temperature + distance, data = data_subset[-c(237,235,202,193,114,108,58,51,34,28,22),])

                    # SHAP explanations
                    #pred <- ggpredict(fit, terms = c("duration[30]", "resistance[30]", "temperature[higher]", "distance[large]"))
                    pred_data <- subset(data_subset[-c(237,235,202,193,114,108,58,51,34,28,22),], select = c("duration", "resistance", "temperature", "distance", "sensitivity"))
                    pred_data$temperature <- factor(pred_data$temperature, levels = c("higher", "lower"))
                    explainer <- shapr(pred_data, fit)
                    p <- mean(pred_data$sensitivity)
                       
                    new_data <- data.frame(duration = c({duration}), 
                                            resistance = c({resistance}),
                                            temperature = c("{temperature}"),
                                            distance = c("{distance}"))

                    new_data$temperature <- factor(new_data$temperature, levels = c("higher", "lower"))
                    new_data$distance <- factor(new_data$distance, levels = c("large", "small"))
                    new_pred <- predict(fit, new_data)
                    explanation_cat <- shapr::explain(new_data, approach = "ctree", explainer = explainer, prediction_zero = p)

                    # Shapley values
                    shapley_values <- explanation_cat[["dt"]][,2:5]

                    # Standardize Shapley values
                    standardized_values <- shapley_values / sum(abs(shapley_values))
                    explanation_cat[["dt"]][,2:5] <- standardized_values
                    
                    pl <- plot(explanation_cat, digits = 1, plot_phi0 = FALSE) 
                    pl[["data"]]$header <- paste("predicted sensitivity = ", round(new_pred, 1), sep = " ")
                    levels(pl[["data"]]$sign) <- c("positive", "negative")
                    data_plot <- pl[["data"]]
                    labels <- c(duration = paste("<img src='/home/ruben/xai4mhc/Icons/duration_fire_black.png' width='57' /><br>\n", new_data$duration), 
                    resistance = paste("<img src='/home/ruben/xai4mhc/Icons/fire_resistance_black.png' width='71' /><br>\n", new_data$resistance), 
                    temperature = paste("<img src='/home/ruben/xai4mhc/Icons/celsius_transparent.png' width='79' /><br>\n", new_data$temperature), 
                    distance = paste("<img src='/home/ruben/xai4mhc/Icons/distance_fire_victim_black.png' width='100' /><br>\n", new_data$distance))
                    data_plot$variable <- reorder(data_plot$variable, -abs(data_plot$phi))
                    pl <- ggplot(data_plot, aes(x = variable, y = phi, fill = ifelse(phi >= 0, "positive", "negative"))) + geom_bar(stat = "identity") + scale_x_discrete(name = NULL, labels = labels) + theme(axis.text.x = ggtext::element_markdown(color = "black", size = 15)) + theme(text=element_text(size = 15, family="Roboto"),plot.title=element_text(hjust=0.5,size=15,color="black",face="bold",margin = margin(b=5)),
                    plot.caption = element_text(size=15,margin = margin(t=25),color="black"),
                    panel.background = element_blank(),
                    axis.text = element_text(size=15,colour = "black"),axis.text.y = element_text(colour = "black",margin = margin(t=5)),
                    axis.line = element_line(colour = "black"), axis.title = element_text(size=15), axis.title.y = element_text(colour = "black",margin = margin(r=10),hjust = 0.5),
                    axis.title.x = element_text(colour = "black", margin = margin(t=5),hjust = 0.5), panel.grid.major = element_line(color="#DAE1E7"), panel.grid.major.x = element_blank()) + theme(legend.background = element_rect(fill="white",colour = "white"),legend.key = element_rect(fill="white",colour = "white"), legend.text = element_text(size=15),
                    legend.position ="bottom",legend.title = element_text(size=15,face = "plain")) + ggtitle(paste("Predicted sensitivity = ", round(new_pred, 1))) + labs(y="Relative feature contribution", fill="") + scale_y_continuous(breaks=seq(-1,1,by=0.5), limits=c(-1,1), expand=c(0.0,0.0)) + scale_fill_manual(values = c("positive" = "#3E6F9F", "negative" = "#B0D7F0")) + geom_hline(yintercept = 0, color = "black") + theme(axis.text = element_text(color = "black"),
                    axis.ticks = element_line(color = "black"))
                    ggsave("/home/ruben/xai4mhc/TUD-Research-Project-2022/SaR_gui/static/images/sensitivity_plot.svg", pl)
                    ''')
        robjects.r(r_script)
        return 'plot'
    
    # move to utils file and call once when running main.py
    def _loadR2Py(self):
        r_script = (f'''
                    # Load libraries
                    library('ggplot2')
                    library('dplyr')
                    library('rstatix')
                    library('ggpubr')
                    library('tidyverse')
                    library('psych')
                    library("gvlma")
                    library("nparLD")
                    library('pastecs')
                    library('WRS2')
                    library('crank')
                    library('lme4')
                    library('psycho')
                    library('lmerTest')
                    library('corrplot')
                    library('RColorBrewer')
                    library('sjPlot')
                    library('sjmisc')
                    library('ggeffects')
                    library('interactions')
                    library('ggcorrplot')
                    library('car')
                    library('caret')
                    library('readxl')
                    library('GGally')
                    library('brant')
                    library('wordcloud')
                    library('RColorBrewer')
                    library('wordcloud2')
                    library('tm')
                    library('tidytext')
                    library('tau')
                    library('shapr')
                    library('DALEX')
                    library('iml')
                    library('pre')
                    library('ggtext')
                    library('ggdist')
                    library('rvest')
                    library('png')
                    library('grid')
                    ''')
        robjects.r(r_script)
    
def add_object(locs, image, size, opacity, name):
    action_kwargs = {}
    add_objects = []
    for loc in locs:
        obj_kwargs = {}
        obj_kwargs['location'] = loc
        obj_kwargs['img_name'] = image
        obj_kwargs['visualize_size'] = size
        obj_kwargs['visualize_opacity'] = opacity
        obj_kwargs['name'] = name
        add_objects+=[obj_kwargs]
    action_kwargs['add_objects'] = add_objects
    return action_kwargs

def calculate_distances(p1, p2):
    # Unpack the coordinates
    x1, y1 = p1
    x2, y2 = p2
    
    # Euclidean distance
    euclidean_distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Manhattan distance
    manhattan_distance = abs(x2 - x1) + abs(y2 - y1)
    
    return euclidean_distance, manhattan_distance