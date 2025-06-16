from simpledrone.panda3d_gui import Panda3D_GUI
from simpledrone.radio_command.keyboard_rc import KeyboardRC


keyboard_rc = KeyboardRC()
gui = Panda3D_GUI()

gui.register_keys(keyboard_rc.get_keys_pressed())

while True:
    print(keyboard_rc.read_rc_inputs())
    gui.step()

