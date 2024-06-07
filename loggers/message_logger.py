from matrx.logger.logger import GridWorldLogger
from matrx.grid_world import GridWorld
import copy
import json
import numpy as np
import re


class message_logger(GridWorldLogger):
    """ Logs messages send and received by (all) agents """

    def __init__(self, save_path="", file_name_prefix="", file_extension=".csv", delimiter=";"):
        super().__init__(save_path=save_path, file_name=file_name_prefix, file_extension=file_extension,
                         delimiter=delimiter, log_strategy=1)
        self._threshold = 4.2

    def log(self, grid_world: GridWorld, agent_data: dict):

        log_data = {
            'threshold': '',
            'total_number_messages_human': 0,
            'total_number_messages_robot': 0,
            'firefighter_decisions': 0,
            'firefighter_danger': 0,
            'firefighter_danger_rate': 0,
            'total_allocations_human': 0,
            'total_allocations_robot': 0,
            'total_allocations': 0,
            'total_interventions': 0,
            'disagreement_rate': 0,
            'sensitivity': '',
            'decision': '',
            'extra_info_count': 0
        }

        gwmm = grid_world.message_manager
        t = grid_world.current_nr_ticks - 1
        tot_messages_human = 0
        tot_messages_robot = 0
        firefighter_danger = 0
        firefighter_decisions = 0
        tot_allocations_human = 0
        tot_allocations_robot = 0
        tot_interventions = 0
        extra_info_count = 0
        sensitivity = ''
        decision = ''
        processed_messages = []
        interventions_sensitivity = []

        for i in range(0, t):
            if i in gwmm.preprocessed_messages.keys():
                for mssg in gwmm.preprocessed_messages[i]:
                    if 'Counterbalancing' in mssg.content:
                        counterbalance_condition = mssg.content.split()[2]
                        robot = mssg.content.split()[4]
                        self._threshold = mssg.content.split()[6]
                   
                    if (i, mssg.content) not in processed_messages and 'Time left: ' not in mssg.content and 'Smoke spreads: ' not in mssg.content and 'Coordinates vic' not in mssg.content and 'Target' not in mssg.content \
                        and 'Temperature: ' not in mssg.content and 'Location: ' not in mssg.content and 'Distance: ' not in mssg.content and 'Victims rescued: ' not in mssg.content and 'Counterbalancing' not in mssg.content:
                        processed_messages.append((i, mssg.content))

                        if 'No intervention' in mssg.content and float(mssg.content.split()[6]) < 4.2:
                            tot_allocations_robot += 1
                            sensitivity = float(mssg.content.split()[6])

                        if 'No intervention' in mssg.content and float(mssg.content.split()[6]) >= 4.2:
                            tot_allocations_human += 1
                            sensitivity = float(mssg.content.split()[6])

                        if 'Reallocating' in mssg.content and 'to you' in mssg.content and float(mssg.content.split()[9]) < 4.2:
                            tot_allocations_robot += 1
                            tot_interventions += 1
                            sensitivity = float(mssg.content.split()[9])

                        if 'Reallocating' in mssg.content and 'to me' and float(mssg.content.split()[9]) >= 4.2:
                            tot_allocations_human += 1
                            tot_interventions += 1
                            sensitivity = float(mssg.content.split()[9])
                            
                        if 'human' in mssg.from_id:
                            tot_messages_human += 1

                        if 'Titus' in mssg.from_id and 'No intervention' not in mssg.content or 'Brutus' in mssg.from_id and 'No intervention' not in mssg.content:
                            tot_messages_robot += 1

                        if 'ABORTING TASK' in mssg.content:
                            firefighter_danger += 1

                        if 'Sending in' in mssg.content and 'Not sending in' not in mssg.content:
                            firefighter_decisions += 1

                        if 'This is how much each feature contributed to the predicted sensitivity' in mssg.content:
                            extra_info_count += 1

        log_data['threshold'] = 4.2
        log_data['total_number_messages_human'] = tot_messages_human
        log_data['total_number_messages_robot'] = tot_messages_robot
        log_data['total_allocations_human'] = tot_allocations_human
        log_data['total_allocations_robot'] = tot_allocations_robot
        log_data['firefighter_danger'] = firefighter_danger
        log_data['firefighter_decisions'] = firefighter_decisions
        log_data['sensitivity'] = sensitivity
        log_data['decision'] = decision
        log_data['extra_info_count'] = extra_info_count

        if firefighter_decisions > 0:
            log_data['firefighter_danger_rate'] = firefighter_danger / firefighter_decisions

        tot_allocations = tot_allocations_human + tot_allocations_robot
        log_data['total_allocations'] = tot_allocations
        log_data['total_interventions'] = tot_interventions
        if tot_allocations > 0:
            log_data['disagreement_rate'] = tot_interventions / tot_allocations

        return log_data