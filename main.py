import os, requests, random
import sys
import csv
import glob
import pathlib
from custom_gui import visualization_server
from custom_gui_demand import visualization_server_demand
from worlds1.world_builder import create_builder
from utils1.util_functions import load_R_to_Py
from pathlib import Path

if __name__ == "__main__":
    print("\nEnter the participant ID:")
    id = input()
    print("\nEnter one of the conditions 'baseline', 'adaptive', 'contrastive', 'global', 'on-demand', or 'textual':")
    condition = input()
    if condition == 'baseline':
        media_folder = pathlib.Path().resolve()
        print("Starting custom visualizer")
        vis_thread = visualization_server.run_matrx_visualizer(verbose = False, media_folder = media_folder)
        for environment in ['trial', 'experiment']:
            if environment == 'trial':
                builder = create_builder(id = 'na', exp_version = 'trial', name = 'Brutus', condition = 'tutorial', task = 'na')
                builder.startup(media_folder = media_folder)
                print("Started world...")
                world = builder.get_world()
                builder.api_info['matrx_paused'] = False
                world.run(builder.api_info)
            if environment == 'experiment':
                builder = create_builder(id = id, exp_version = 'experiment', name = 'Brutus', condition = condition, task = 1)
                builder.startup(media_folder = media_folder)
                print("Started world...")
                world = builder.get_world()
                builder.api_info['matrx_paused'] = True
                world.run(builder.api_info)

                fld = os.getcwd()
                recent_dir = max(glob.glob(os.path.join(fld, '*/' + id + '/')), key = os.path.getmtime)
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
                                if row[6] != previous_row[0] and row[8] != previous_row[2] and row[9] == previous_row[3] or \
                                    row[7] != previous_row[1] and row[8] != previous_row[2] and row[9] == previous_row[3]:
                                    csv_writer.writerow([id, condition, 'no intervention', row[11]])
                                if row[6] != previous_row[0] and row[8] != previous_row[2] and row[9] != previous_row[3]:
                                    csv_writer.writerow([id, condition, 'allocate to robot', row[11]]) 
                                if row[7] != previous_row[1] and row[8] != previous_row[2] and row[9] != previous_row[3]:
                                    csv_writer.writerow([id, condition, 'allocate to self', row[11]])   

                        previous_row = row[6:10]
                        res = {message_header[i]: row[i] for i in range(len(message_header))}
                        message_contents.append(res)

                completeness = action_contents[-1]['completeness']
                no_messages_human = message_contents[-1]['total_number_messages_human']
                total_allocations = message_contents[-1]['total_allocations']
                human_allocations = message_contents[-1]['total_allocations_human']
                total_interventions = message_contents[-1]['total_interventions']
                disagreement_rate = message_contents[-1]['disagreement_rate']

                print("Saving output...")
                with open(os.path.join(recent_dir, 'world_1/output.csv'), mode= 'w') as csv_file:
                    csv_writer = csv.writer(csv_file, delimiter = ';', quotechar = '"', quoting = csv.QUOTE_MINIMAL)
                    csv_writer.writerow(['completeness', 'human_messages', 'total_allocations', 'human_allocations', 'total_interventions', 'disagreement_rate'])

                    csv_writer.writerow([completeness, no_messages_human, total_allocations, human_allocations, total_interventions, disagreement_rate])
                        
                with open(fld + '/data/complete_data_performance.csv', mode = 'a+') as csv_file:
                    csv_writer = csv.writer(csv_file, delimiter = ';', quotechar='"', quoting = csv.QUOTE_MINIMAL)
                    csv_writer.writerow([id, condition, completeness, no_messages_human, total_allocations, human_allocations, total_interventions, disagreement_rate])

    else:
        if condition == 'on-demand':
            media_folder = pathlib.Path().resolve()
            print("Starting custom visualizer")
            vis_thread = visualization_server_demand.run_matrx_visualizer(verbose=False, media_folder=media_folder)
            for environment in ['trial', 'experiment']:
                if environment == 'trial':
                    builder = create_builder(id='na', exp_version='trial', name='Brutus', condition='tutorial',
                                             task='na')
                    builder.startup(media_folder=media_folder)
                    print("Started world...")
                    world = builder.get_world()
                    builder.api_info['matrx_paused'] = False
                    world.run(builder.api_info)
                if environment == 'experiment':
                    builder = create_builder(id=id, exp_version='experiment', name='Brutus', condition=condition,
                                             task=1)
                    builder.startup(media_folder=media_folder)
                    print("Started world...")
                    world = builder.get_world()
                    builder.api_info['matrx_paused'] = True
                    world.run(builder.api_info)

                    fld = os.getcwd()
                    recent_dir = max(glob.glob(os.path.join(fld, '*/' + id + '/')), key=os.path.getmtime)
                    recent_dir = max(glob.glob(os.path.join(recent_dir, '*/')), key=os.path.getmtime)
                    action_file = glob.glob(os.path.join(recent_dir, 'world_1/action*'))[0]
                    message_file = glob.glob(os.path.join(recent_dir, 'world_1/message*'))[0]
                    action_header = []
                    action_contents = []
                    message_header = []
                    message_contents = []
                    unique_robot_moves = []
                    previous_row = None

                    with open(action_file) as csvfile:
                        reader = csv.reader(csvfile, delimiter=';', quotechar="'")
                        for row in reader:
                            if action_header == []:
                                action_header = row
                                continue
                            if row[1:3] not in unique_robot_moves:
                                unique_robot_moves.append(row[1:3])
                            res = {action_header[i]: row[i] for i in range(len(action_header))}
                            action_contents.append(res)

                    with open(message_file) as csvfile:
                        reader = csv.reader(csvfile, delimiter=';', quotechar="'")
                        for row in reader:
                            if message_header == []:
                                message_header = row
                                continue

                            if row[6:10] != previous_row and row[11] != "":
                                with open(fld + '/data/complete_data_decisions.csv', mode='a+') as csv_file:
                                    csv_writer = csv.writer(csv_file, delimiter=';', quotechar='"',
                                                            quoting=csv.QUOTE_MINIMAL)
                                    if row[6] != previous_row[0] and row[8] != previous_row[2] and row[9] == \
                                            previous_row[3] or \
                                            row[7] != previous_row[1] and row[8] != previous_row[2] and row[9] == \
                                            previous_row[3]:
                                        csv_writer.writerow([id, condition, 'no intervention', row[11]])
                                    if row[6] != previous_row[0] and row[8] != previous_row[2] and row[9] != \
                                            previous_row[3]:
                                        csv_writer.writerow([id, condition, 'allocate to robot', row[11]])
                                    if row[7] != previous_row[1] and row[8] != previous_row[2] and row[9] != \
                                            previous_row[3]:
                                        csv_writer.writerow([id, condition, 'allocate to self', row[11]])

                            previous_row = row[6:10]
                            res = {message_header[i]: row[i] for i in range(len(message_header))}
                            message_contents.append(res)

                    completeness = action_contents[-1]['completeness']
                    no_messages_human = message_contents[-1]['total_number_messages_human']
                    total_allocations = message_contents[-1]['total_allocations']
                    human_allocations = message_contents[-1]['total_allocations_human']
                    total_interventions = message_contents[-1]['total_interventions']
                    disagreement_rate = message_contents[-1]['disagreement_rate']

                    print("Saving output...")
                    with open(os.path.join(recent_dir, 'world_1/output.csv'), mode='w') as csv_file:
                        csv_writer = csv.writer(csv_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        csv_writer.writerow(['completeness', 'human_messages', 'total_allocations', 'human_allocations',
                                             'total_interventions', 'disagreement_rate'])

                        csv_writer.writerow(
                            [completeness, no_messages_human, total_allocations, human_allocations, total_interventions,
                             disagreement_rate])

                    with open(fld + '/data/complete_data_performance.csv', mode='a+') as csv_file:
                        csv_writer = csv.writer(csv_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        csv_writer.writerow(
                            [id, condition, completeness, no_messages_human, total_allocations, human_allocations,
                             total_interventions, disagreement_rate])
        print("\nCondition yet to be implemented by students")
        exit()
    print("DONE!")
    print("Shutting down custom visualizer")
    r = requests.get("http://localhost:" + str(visualization_server.port) + "/shutdown_visualizer")
    vis_thread.join()
    builder.stop()