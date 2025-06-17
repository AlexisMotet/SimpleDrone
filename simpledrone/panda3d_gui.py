import numpy as np

from typing import Tuple
from direct.showbase.ShowBase import ShowBase

class Panda3D_GUI(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.disableMouse()

        # self.scene = self.loader.loadModel("models/environment")
        # self.scene.reparentTo(self.render)

        self.camera.setPos(100, -10, 7)
        self.camera.lookAt(0, 0, 0)

        self.entities = {}

    def register_keys(self, keys_pressed: dict[str, int]):
        for key in keys_pressed.keys():
            self.accept(key, lambda k=key: keys_pressed.__setitem__(k, 1), [])
            self.accept(f"{key}-up", lambda k=key: keys_pressed.__setitem__(k, 0), [])

    def step(self):
        self.task_mgr.step()

    def add_entity(self, name: str, model_path: str, position: Tuple[float]):
        entity = self.loader.loadModel(model_path)
        self.entities[name] = entity
        entity.reparentTo(self.render)
        entity.setPos(*position)

    def move_entity(self, name: str, position: Tuple[float]):
        self.entities[name].setPos(*position)

    def look_at_entity(self, position: Tuple[float], forward: Tuple[float]):
        camera_pos = (position[i] - 10*forward[i] for i in range(3))
        self.camera.setPos(*camera_pos)
        self.camera.lookAt(*position)


    
