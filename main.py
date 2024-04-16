import os, requests, random
import sys
import csv
import glob
import pathlib
from custom_gui import visualization_server
from worlds1.world_builder import create_builder
from utils1.util_functions import load_R_to_Py
from pathlib import Path

if __name__ == "__main__":
    print("\nEnter the participant ID:")
    id = input()
    print("\nEnter one of the environments 'trial' or 'experiment':")
    environment = input()
    if environment == 'trial':
        media_folder = pathlib.Path().resolve()
        print("Starting custom visualizer")
        vis_thread = visualization_server.run_matrx_visualizer(verbose = False, media_folder = media_folder)
        builder = create_builder(id = 'na', exp_version = 'trial', name = 'Brutus', condition = 'tutorial', task = 'na', counterbalance_condition = 'na')
        builder.startup(media_folder = media_folder)
        print("Started world...")
        world = builder.get_world()
        builder.api_info['matrx_paused'] = False
        world.run(builder.api_info)
    else:
        print("\nEnter one of the conditions 'baseline', 'adaptive', 'contrastive', 'global', 'on-demand', or 'textual':")
        condition = input()
        if condition == 'baseline':
            print("\nEnter one of the 2 counterbalancing conditions:")
            counterbalance_condition = input()
            if counterbalance_condition == '1':
                robot_order = ['Brutus'] * 2
                task_order = [2, 3]
            if counterbalance_condition == '2':
                robot_order = ['Brutus'] * 2
                task_order = [3, 2]

            start_scenario = None
            media_folder = pathlib.Path().resolve()
            print("Starting custom visualizer")
            vis_thread = visualization_server.run_matrx_visualizer(verbose = False, media_folder = media_folder)

            for i, robot in enumerate(robot_order, start = 0):
                print(f"\nTask {i+1}: The robot is {robot} and task version {task_order[i]}.\n")
                builder = create_builder(id = id, exp_version = 'experiment', name = robot, condition = condition, task = task_order[i], counterbalance_condition = counterbalance_condition)
                builder.startup(media_folder = media_folder)
                print("Started world...")
                world = builder.get_world()
                builder.api_info['matrx_paused'] = True
                world.run(builder.api_info)

                if environment == "experiment":
                    fld = os.getcwd()
                    print(fld)
                    recent_dir = max(glob.glob(os.path.join(fld, '*/counterbalance_' + counterbalance_condition + '/' + id + '/')), key = os.path.getmtime)
                    recent_dir = max(glob.glob(os.path.join(recent_dir, '*/')), key = os.path.getmtime)
                    action_file = glob.glob(os.path.join(recent_dir, 'world_1/action*'))[0]
                    message_file = glob.glob(os.path.join(recent_dir, 'world_1/message*'))[0]
                    action_header = []
                    action_contents = []
                    message_header = []
                    message_contents = []
                    unique_robot_moves = []
                    previous_row = None

                    with open(action_file) as csvfile:
                        reader = csv.reader(csvfile, delimiter = ';', quotechar= "'")
                        for row in reader:
                            if action_header == []:
                                action_header = row
                                continue
                            if row[1:3] not in unique_robot_moves:
                                unique_robot_moves.append(row[1:3])
                            res = {action_header[i]: row[i] for i in range(len(action_header))}
                            action_contents.append(res)
                    
                    with open(message_file) as csvfile:
                        reader = csv.reader(csvfile, delimiter = ';', quotechar = "'")
                        for row in reader:
                            if message_header == []:
                                message_header = row
                                continue
                            
                            if row[6:10] != previous_row and row[11] != "":
                                with open(fld + '/data/complete_data_decisions.csv', mode = 'a+') as csv_file:
                                    csv_writer = csv.writer(csv_file, delimiter = ';', quotechar='"', quoting = csv.QUOTE_MINIMAL)
                                    if row[6] != previous_row[0] and row[8] != previous_row[2] and row[9] == previous_row[3]:
                                        csv_writer.writerow([id, condition, counterbalance_condition, task_order[i], row[0], robot, 'no intervention', row[11]])
                                    if row[6] != previous_row[0] and row[8] != previous_row[2] and row[9] != previous_row[3]:
                                        csv_writer.writerow([id, condition, counterbalance_condition, task_order[i], row[0], robot, 'allocate to robot', row[11]]) 
                                    if row[7] != previous_row[1] and row[8] != previous_row[2] and row[9] != previous_row[3]:
                                        csv_writer.writerow([id, condition, counterbalance_condition, task_order[i], row[0], robot, 'allocate to self', row[11]])   

                            previous_row = row[6:10]
                            res = {message_header[i]: row[i] for i in range(len(message_header))}
                            message_contents.append(res)

                    no_ticks = action_contents[-1]['tick_nr']
                    completeness = action_contents[-1]['completeness']
                    no_messages_human = message_contents[-1]['total_number_messages_human']
                    no_messages_robot = message_contents[-1]['total_number_messages_robot']
                    firefighter_decisions = message_contents[-1]['firefighter_decisions']
                    firefighter_danger = message_contents[-1]['firefighter_danger']
                    firefighter_danger_rate = message_contents[-1]['firefighter_danger_rate']
                    total_allocations = message_contents[-1]['total_allocations']
                    human_allocations = message_contents[-1]['total_allocations_human']
                    robot_allocations = message_contents[-1]['total_allocations_robot']
                    total_interventions = message_contents[-1]['total_interventions']
                    disagreement_rate = message_contents[-1]['disagreement_rate']

                    print("Saving output...")
                    with open(os.path.join(recent_dir, 'world_1/output.csv'), mode= 'w') as csv_file:
                        csv_writer = csv.writer(csv_file, delimiter = ';', quotechar = '"', quoting = csv.QUOTE_MINIMAL)
                        csv_writer.writerow(['completeness', 'ticks', 'moves', 'robot_messages', 'human_messages', 'total_allocations', 'human_allocations', 'robot_allocations', 'total_interventions', 
                                             'disagreement_rate', 'correct_behavior_rate', 'incorrect_behavior_rate', 'correct_intervention_rate', 'incorrect_intervention_rate', 'firefighter_decisions', 
                                             'firefighter_danger', 'firefighter_danger_rate', 'CRR_ND_self', 'FR_ND_self', 'FRR_MD_self', 'CR_MD_self', 'CRR_MD_robot', 'FR_MD_robot', 'CRR_ND_robot', 'FR_ND_robot'])

                        csv_writer.writerow([completeness, no_ticks, len(unique_robot_moves), no_messages_robot, no_messages_human, total_allocations, human_allocations, 
                                             robot_allocations, total_interventions, disagreement_rate, firefighter_decisions, firefighter_danger, firefighter_danger_rate])
                        
                    with open(fld + '/data/complete_data_performance.csv', mode = 'a+') as csv_file:
                        csv_writer = csv.writer(csv_file, delimiter = ';', quotechar='"', quoting = csv.QUOTE_MINIMAL)
                        csv_writer.writerow([id, condition, counterbalance_condition, task_order[i], row[0], robot, completeness, no_ticks, len(unique_robot_moves), no_messages_robot, no_messages_human, 
                                             total_allocations, human_allocations, robot_allocations, total_interventions, disagreement_rate,  firefighter_decisions, firefighter_danger, firefighter_danger_rate])

            print("DONE!")
            print("Shutting down custom visualizer")
            r = requests.get("http://localhost:" + str(visualization_server.port) + "/shutdown_visualizer")
            vis_thread.join()
        else:
            print("\nCondition yet to be implemented by students")
            exit()
    print("DONE!")
    print("Shutting down custom visualizer")
    r = requests.get("http://localhost:" + str(visualization_server.port) + "/shutdown_visualizer")
    vis_thread.join()
    builder.stop()