from threading import Thread
from typing import List, Dict

import matplotlib.pyplot as plt
from py_trees import Status

from giskardpy import identifier
from giskardpy.goals.goal import Goal
from giskardpy.god_map_user import GodMap
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time, catch_and_raise_to_blackboard
from giskardpy.utils.logging import logwarn
from giskardpy.utils.utils import create_path


class PlotGanttChart(GiskardBehavior):

    @profile
    def __init__(self, name: str = 'plot gantt chart'):
        super().__init__(name)

    def plot_gantt_chart(self, goals: Dict[str, Goal], file_name: str):
        tasks = []
        start_dates = []
        end_dates = []
        for goal_name, goal in goals.items():
            for task in goal.tasks:
                tasks.append(f'{goal_name} - {task.name}')
                if not task.to_start:
                    start_dates.append(0)
                else:
                    start_dates.extend([x.state_flip_times[0] for x in task.to_start])
                if not task.to_end:
                    end_dates.append(GodMap.trajectory_time_in_seconds)
                else:
                    end_dates.extend([x.state_flip_times[0] for x in task.to_end])

        plt.figure(figsize=(10, 5))

        for i, (task, start_date, end_date) in enumerate(zip(tasks, start_dates, end_dates)):
            plt.barh(task, end_date - start_date, height=0.8, left=start_date, color=(133/255, 232/255, 133/255))

        for monitor in GodMap.monitors:
            state = False
            for flip_event in monitor.state_flip_times:
                text = f'{monitor.name} {state} -> {not state}'
                state = not state
                plt.axvline(x=flip_event, color='k', linestyle='--')
                plt.text(flip_event, (len(tasks)-1)/2, text, color='k', rotation='vertical', va='center')

        plt.xlabel('Time')
        plt.ylabel('Tasks')
        plt.tight_layout()
        create_path(file_name)
        plt.savefig(file_name)

    @catch_and_raise_to_blackboard
    @record_time
    @profile
    def update(self):
        goals = GodMap.god_map.get_data(identifier.motion_goals)
        file_name = GodMap.god_map.get_data(identifier.tmp_folder) + f'/gantt_charts/goal_{GodMap.goal_id}.png'
        self.plot_gantt_chart(goals, file_name)
        return Status.SUCCESS
