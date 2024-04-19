# XAI for firefighting
Description will be extended further.

## Installation
Download or clone this repository and the required dependencies listed in the 'requirements.txt' file. We recommend using Python 3.8 or 3.9, and to create a virtual environment for this project. It is probably also wise to each create your own repository branch. You can use the following step by step installation steps after cloning or downloading this repository:
- Create a virtual environment for this project and activate it
- Install the required dependencies through 'pip install -r requirements.txt'. 
- Launch the environment by running main.py.
- You will be asked to enter a participant id, this can be anything you like.
- You will be asked to enter which environment to run. 'Trial' will launch a step by step tutorial of the task in a simplified and smaller world, aimed at getting you familiar with the environment, task, and messaging system. 'Experiment' will launch the complete task, but first you will be asked to enter one of the explainability conditions 'baseline', 'adaptive', 'contrastive', 'global', 'on-demand' or 'textual'. I have already implemented the baseline condition, each of you will implement one of the other conditions. 
- You will be asked to enter one of the two counterbalancing conditions. Since the experiment consists of 2 tasks, we have to balance the order of the tasks. Counterbalancing condition '1' starts with task 1 followed by task 2; counterbalancing condition '2' starts with task 2 followed by task 1.
- Go to http://localhost:3000 and clear your old cache of the page by pressing 'ctrl' + 'F5'.
- Open the 'God' and 'brutus' view. Start the task in either the 'God' or 'brutus' view with the play icon in the top left of the toolbar. The 'God' view can be used as the experimenter to oversee the whole experiment, the 'brutus' view should be used by participants and when you want to play around. Open the messaging interface by pressing the chat box icon in the top right of the toolbar.

## Overview
Below I briefly describe the content and files of the important folders in more detail. For your individual projects, the only implementation modifications should (probably) be made to the 'agents1' folder. 
- 'agents1': Contains the 'firefighter.py' and 'robot.py' files, as well as their versions for the tutorial. For your individual projects, you will modify the 'robot.py' file. When you have this file opened, use 'ctrl' + 'F' to search for 'To be implemented'. Here, you can see where your generated explanations can be implemented. 
- 'custom_gui': Contains the folders and files related to the GUI of the experiment. For your individual projects, you might have to add plots/visual explanations to the 'images' folder under 'static'. 
- 'data': Contains the survey data 'moral_sensitivity_survey_data.xlsx' used to create the baseline explanations and model to predict moral sensitivity. Moreover, it contains two .csv files where all important output data of the experiments will be automatically added to. Therefore, it is important that you do not delete these files and once you start the official experiments, make sure the .csv files are empty/just contain the header.
- 'experiment_logs': Contains all the objectively logged data during the tasks, for each tick. This data is ultimately filtered before adding the relevant information to the 'data' folder.
- 'utils1': Contains the 'util_functions.py' file. This file is used to launch 'rpy2' for running R embedded in Python, using 4 functions to create the visual baseline explanations using the SHAP explainer method. 