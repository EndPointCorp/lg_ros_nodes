#!/usr/bin/python3

import os
import sys
import time
import subprocess
import importlib

try:
    from Xlib import X, display
except Exception as e:
    subprocess.run(['/usr/local/bin/pip', 'install', 'xlib'])  # , check=True)
    exit(subprocess.run([*sys.argv], check=True).returncode)

os.environ['DISPLAY'] = ':0'


class Earth:
    icons_column = 50
    checks_column = 36
    tabs_column = 11
    icon_colors = {}  # colors on the icons on each layer within the checkmark's y range to find where to click on
    icon_colors["imagery"] = ["0F0F0F", "F3F8FC"]  #50, lg/solo, lg/solo-selected, ...
    icon_colors["borders"] = ["89BFF4", "8FC7F7"]
    icon_colors["places"] = ["FFFF99", "E2F4B7"]
    icon_colors["photos"] = ["A8C74C", "A5CD81"]
    icon_colors["roads"] = ["89BFF4", "8FC7F7"]
    icon_colors["buildings"] = ["E8F4F9", "D2ECFB"]
    icon_colors["weather"] = ["8A5F00", "90844C"]
    icon_colors["gallery"] = ["8EBCE8", "93C5EF"]
    icon_colors["more"] = ["E1DB8E", "FEDC5E"]
    icon_colors["terrain"] = ["BBBBBB", "67ABD5"]


    def __init__(self, win_id, offset):
        self.win_id = win_id
        self.offset = offset
        self.is_master = False
        self.states_image = None
        self.icons_image = None
        self.display = display.Display()
        self.states = {}
        self.layer_ys = {}

    def __str__(self):
        return f"XDO_Window ID: {self.win_id}, Offset: {self.offset}, Is Master: {self.is_master}"

    def toggle_menu(self):
        win_id_str = str(self.win_id)
        try:
            subprocess.check_call(['xdotool', 'windowfocus', win_id_str, 'key', 'ctrl+alt+b'])  # .decode().strip()
        except Exception as e:
            print(f"Warning: Failed to toggle menu for window ID: {self.win_id}:\n", e)

    def click(self, x, y):
        x_str = str(x+self.offset)  #TODO check strs are needed
        y_str = str(y)
        try:
            subprocess.check_call(['xdotool', 'mousemove', '--clearmodifiers', x_str, y_str, 'sleep', '.001', 'click', '1'])
        except Exception as e:
            print(f"Failed to click on window ID: {self.win_id}:\n", e)

    def find_layer_states(self):  # find y coordinate and state of layers
        if not earth.wait_for_menu():
            return None
        y_min = 1
        for layer in self.icon_colors.keys():
            y_coord = self.find_color_in_column(self.icons_column, self.icon_colors[layer], y_min)
            print(layer, y_coord)
            if y_coord is None:
                continue
            y_min = y_coord
            self.layer_ys[layer] = y_coord
            self.states[layer] = self.get_state(y_coord)


    def find_color_in_column(self, x, target_colors, y_min=1):
        #target_color = target_color.upper()
        if self.icons_image is None:
            root = self.display.screen().root
            adjusted_x = x + self.offset
            self.icons_image = root.get_image(adjusted_x, 0, 1, 1920, X.ZPixmap, 0xffffffff)
            # Calculate the stride based on the image depth
            depth = self.icons_image.depth
            pixel_size = (depth + 7) // 8
            self.stride = (1 * pixel_size + 3) & ~3  ## TODO check it is always 4(bytes per pixel), simplify
            print("stride", self.stride)
        try:  # Get each pixel value, check if it is in the list
            for y in range(1, 1920):
                if y < y_min:
                    continue
                pixel_value_bytes = self.icons_image.data[y * self.stride : (y + 1) * self.stride]
                pixel_value = int.from_bytes(pixel_value_bytes, byteorder='little')
                hex_color = hex(pixel_value)[2:].zfill(6).upper()
                # if y < 1500 and hex_color != 'FFFFFF':  #dev, testing for colors
                #     print("y: ", y, "hex: ", hex_color)
                if hex_color in target_colors:
                    return y
            return None
        except Exception as e:
            print(e)
            return None

    def get_pixel_color(self, x, y):  # unused now, could be useful
        adjusted_x = x + self.offset
        root = self.display.screen().root
        try:  # Get the pixel color at the specified coordinates
            image = root.get_image(adjusted_x, y, 1, 1, X.ZPixmap, 0xffffffff)
            data = bytes(image.data)
            pixel_value = int.from_bytes(data, byteorder='little')
            hex_color = hex(pixel_value)[2:].zfill(6).upper()
            print(x, y, "get_pixel got: ", hex_color)
            return hex_color
        except subprocess.CalledProcessError:
            print(f"Failed to get pixel color for window ID: {self.win_id}")
            return None

    def wait_for_menu(self, looped=False):
        keep_count = 0
        while earth.get_pixel_color(Earth.checks_column, 1750) != 'FFFFFF':
            time.sleep(.1)
            keep_count += 1
            if keep_count > 9:  #wait up to 0.7 seconds for menu, plenty at ephq could be shortened TEST
                if looped == False:  # try to open menu and wait a second time
                    earth.toggle_menu()
                    return earth.wait_for_menu(True)
                else:
                    return False
        else:  # if the while's condition becomes False
            return True

    # def get_state_old(self, layer_y):  # TODO could work for any but it is set for buildings rn
    #     color_state = self.get_pixel_color(self.checks_column, layer_y)
    #     if color_state in ['FFFFFF']:
    #         return 1
    #     elif color_state in ['000000', '2F2F2F', '9A9A9A', 'BCB9B6', 'F2F2F2', 'F4F4F4']:
    #         return 0
    #     else:
    #         print(layer_y, " !!!!!!!!!color_state: ", color_state)
    #         return None

    def get_state(self, layer_y):  # TODO could work for any but it is set for buildings rn
        if self.states_image is None:
            try:
                root = self.display.screen().root
                adjusted_x = self.offset + self.checks_column
                self.states_image = root.get_image(adjusted_x, 0, 1, 1920, X.ZPixmap, 0xffffffff)
            except Exception as e:
                print(e)
                return None
        pixel_value_bytes = self.states_image.data[layer_y * self.stride: (layer_y + 1) * self.stride]
        pixel_value = int.from_bytes(pixel_value_bytes, byteorder='little')
        color_state = hex(pixel_value)[2:].zfill(6).upper()
        print("layer_row: ", layer_y, " hex_state: ", color_state)
        if color_state == 'FFFFFF':
            return 1
        elif color_state in ['000000', '2F2F2F', '9A9A9A', 'BCB9B6', 'F2F2F2', 'F4F4F4']:
            return 0
        else:
            return None

def find_and_create_earths():
    try:
        window_ids = subprocess.check_output(['xdotool', 'search', '--onlyvisible', '--name', 'Google Earth']).decode().strip().split()
    except Exception as e:
        print("Error: 'xdotool' failed searching for earths:\n", e)
        return []

    EARTHS = []  # Earth objects list
    for win_id in window_ids:
        try:
            geometry_info = subprocess.check_output(['xdotool', 'getwindowgeometry', '--shell', win_id]).decode().strip()
            offset = int(geometry_info.split('X=')[1].split()[0])
            EARTHS.append(Earth(win_id, offset))
        except Exception as e:
            print(f"Failed to retrieve window geometry for window ID: {win_id}:\n", e)
    sorted_earths = sorted(EARTHS, key=lambda earth: earth.offset)  # Sort by offsets
    center_index = len(sorted_earths) // 2
    if len(sorted_earths) in [3, 5, 7]:  #mark center and change order of toggle
        sorted_earths[center_index].is_master = True
        newsort_earths = [sorted_earths[center_index]] + [x for pair in zip(sorted_earths[center_index-1::-1], sorted_earths[center_index+1:]) for x in pair]
        print("earth xdotool ids from center out on canvas:", *[earth.win_id for earth in newsort_earths])
        return newsort_earths
    else:
        sorted_earths[0].is_master = True
        print("earth xdotool ids left to right on canvas:", *[earth.win_id for earth in sorted_earths])
        return sorted_earths


if __name__ == "__main__":
    onoff = 2  #default to toggle
    layer = "buildings" #default to buildings layer
# handle command line arguments
    if len(sys.argv) == 2:
        try:
            onoff = int(sys.argv[1])
        except:
            first = sys.argv[1].upper()
            if first == 'ON':
                onoff = 0
            elif first == 'OFF':
                onoff = 1
            elif sys.argv[1] in Earth.icon_colors.keys():
                layer = sys.argv[1]
            else:
                print("bad arguments: ", sys.argv[1:],
                      f"\nuse: {sys.argv[0]} <layer> <0/1/2>")
                exit(1)
    elif len(sys.argv) == 3:
        layer = sys.argv[1]
        onoff = int(sys.argv[2])
    elif len(sys.argv) == 1:
        print("toggling <buildings> <2> ...")
    else:
        print("bad arguments: ", sys.argv[1:], f"\nuse: {sys.argv[0]} <layer> <0/1/2>")
        exit(1)

    EARTHS = find_and_create_earths()  # create earth objects into a list
    [earth.toggle_menu() for earth in EARTHS]  #open menus ahead of time
    for earth in EARTHS:  # find layer positions and states, toggle if it must
        t_y = None
        while t_y is None:
            earth.find_layer_states()  # or continue?
            try:
                t_y = earth.layer_ys[layer]  # should verify it got a value TODO #if t_y is not None:
            except:
                earth.icons_image = None
                time.sleep(.1)
        print(earth.offset, earth.layer_ys, earth.states)
        if onoff != earth.states[layer]:
            print("toggle: ", onoff, "it was: ", earth.states[layer])
            try:
                earth.click(Earth.checks_column, earth.layer_ys[layer])
            except Exception as e:
                print(f"Failed to click for window ID: {earth.win_id}:\n{e}")
        if onoff == 2:  #we only toggle first instance, rest match the first
            onoff = abs(earth.states[layer] - 1)
        # elif onoff == 3:  # dev/undocumented leave menus open :)
        #     continue
        earth.toggle_menu()
    # TODO check it is closed?
