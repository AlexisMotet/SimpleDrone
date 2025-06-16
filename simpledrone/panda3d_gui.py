import numpy as np

from typing import List
from direct.showbase.ShowBase import ShowBase

class Panda3D_GUI(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.disableMouse()

        self.scene = self.loader.loadModel("models/environment")
        self.scene.reparentTo(self.render)

        self.camera.setPos(100, -10, 7)
        self.camera.lookAt(0, 0, 0)

        self.entities = {}

    def register_action_on_key_pressed(self, key, action, autorepeat=True):
        keyboard_map = base.win.get_keyboard_map()
        local_key = str(keyboard_map.get_mapped_button(key))
        self.accept(local_key if not autorepeat else local_key + "-repeat", action)


    def get_keys_pressed(self, keys: List[str]) -> dict[str, bool]:
        res = {}
        keyboard_map = base.win.get_keyboard_map()
        for key in keys:
            button = keyboard_map.get_mapped_button(key)
            res[key] = base.mouseWatcherNode.is_button_down(button)
        return res

    def step(self):
        self.task_mgr.step()

    def add_entity(self, name: str, model_path: str, position: List[float]):
        entity = self.loader.loadModel(model_path)
        self.entities[name] = entity
        entity.reparentTo(self.scene)
        entity.setPos(*position)

    def move_entity(self, name: str, position: List[float]):
        self.entities[name].setPos(*position)

    
