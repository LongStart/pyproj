import keyboard

def on_press_alt(e):
    print("yemiedie")

def on_press_ctrl(e):
    print("666")

def on_release_ctrl(e):
    # keyboard.on_press_key('tab', on_press_tab)
    pass



if __name__ == "__main__":
    # keyboard.hook(print_pressed_keys)
    keyboard.on_press_key('alt', on_press_alt)
    keyboard.on_press_key('ctrl', on_press_ctrl)
    # keyboard.wait()
    input("~:")