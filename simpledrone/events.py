import heapq
from typing import Callable


class Event:
    def __init__(self, t: float, action: Callable):
        self.t = t
        self.action = action

    def execute(self):
        self.action()

    def __lt__(self, other):
        return self.t < other.t

class Task:
    def get_next_task_date(self) -> float:
        raise NotImplementedError()
    
    def execute(self):
        raise NotImplementedError()
    
    
class EventQueue:
    def __init__(self):
        self.events = []
    
    def empty(self):
        return len(self.events) == 0

    def push(self, event: Event):
        heapq.heappush(self.events, event)


    def pop(self) -> Event:
        return heapq.heappop(self.events)
    
    def add_task(self, task: Task):
        heapq.heappush(self.events, Event(task.get_next_task_date(), lambda: self._execute_task(task)))

    def _execute_task(self, task: Task):
        t = task.get_next_task_date()
        print(f"executing {task} @ {t}")
        task.execute(t)
        heapq.heappush(self.events, Event(task.get_next_task_date(), lambda: self._execute_task(task)))

