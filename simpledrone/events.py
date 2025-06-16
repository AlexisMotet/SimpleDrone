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
    def get_next_t(self) -> float:
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
        heapq.heappush(self.events, Event(task.get_next_t(), lambda: self._execute_task(task)))

    def _execute_task(self, task: Task):
        task.execute(task.get_next_t())
        heapq.heappush(self.events, Event(task.get_next_t(), lambda: self._execute_task(task)))

    



# class ReadIMUData(Event):
#     def __init__(self, t: float, drone: Drone):
#         super().__init__(t)
#         self.drone = drone

#     def __call__(self, event_queue):
#         cmds = self.drone.radio_receiver.receive(self.t, self.raw_cmds)
#         event_queue.push(ReadIMUData(self.t))



# def read_radio_commands(t, drone, event_queue):
#     raw_cmds = drone.radio_command.read_commands(t)
#     t += drone.radio_receiver.get_transmission_delay()
#     event_queue.push(Event(t, lambda: ))
