import sys
import os

# Add the parent directory of utility_scripts to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import numpy as np
from utility_scripts.create_joint_angle_gaits import * 
from utility_scripts.interactive_bezier import * 
import tkinter as tk 
from tkinter import ttk
from tkinter import messagebox 
from tkinter.font import Font

class GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Gait Generator")
        self.create_widgets()

    def create_widgets(self):
        bold_font = Font(family="Times", size=12, weight="bold")
        explain_font = Font(family="Times", size=10, weight="normal", slant="italic")
        
        # Create a label and combobox for gait_type
        ttk.Label(self.root, text="Gait type:", font=bold_font).grid(row=0, column=0, padx=20, pady=20)
        self.gait_type_var = tk.StringVar()
        self.gait_type_combobox = ttk.Combobox(self.root, textvariable=self.gait_type_var)
        self.gait_type_combobox['width'] = 40
        self.gait_type_combobox['state'] = 'readonly'
        self.gait_type_combobox['values'] = ("small_gait", "large_gait", "high_step_1", "high_step_2", "high_step_3", "ascending", "descending", "sit_to_stand", "stand_to_sit", "hinge", "sideways")
        self.gait_type_combobox.grid(row=0, column=1, padx=20, pady=20)

        # Create a label and entry for gait length in seconds
        ttk.Label(self.root, text="Gait length in seconds:", font=bold_font).grid(row=1, column=0, padx=20, pady=20)
        self.gait_length_var = tk.DoubleVar()
        self.gait_length_entry = ttk.Entry(self.root, textvariable=self.gait_length_var)
        self.gait_length_entry.grid(row=1, column=1, padx=20, pady=20)

        # Create a piece of explanatory text with the gait length widget 
        self.gait_length_explain = ttk.Label(self.root, text="\u2022small_gait or large_gait: time taken for one leg to place a full step. \n\u2022high steps: time taken to step onto the box. \n\u2022sideways: time taken to place one step outward, without step close. \n\u2022others: self_explanatory.", font=explain_font)
        self.gait_length_explain.grid(row=2, column=0, columnspan=2, padx=20, pady=20)

        # Create a label and entry for high level frequency 
        ttk.Label(self.root, text="HL frequency:", font=bold_font).grid(row=3, column=0, padx=20, pady=20)
        self.hl_frequency_var = tk.IntVar()
        self.hl_frequency_entry = ttk.Entry(self.root, textvariable=self.hl_frequency_var)
        self.hl_frequency_entry.grid(row=3, column=1, padx=20, pady=20)

        # Create a button to generate the CSV
        self.generate_button = ttk.Button(self.root, text="Generate CSV", command=self.generate_gait)
        self.generate_button.grid(row=4, column=0, columnspan=2, padx=20, pady=20)

    def show_generating_message(self):
        self.popup = tk.Toplevel(self.root)
        self.popup.title("Generating")
        tk.Label(self.popup, text="Generating gaits!").pack(padx=300, pady=300)
        self.root.after(1000, self.close_popup)

    
    def close_popup(self):
        if self.popup:
            self.popup.destroy()


    def generate_gait(self):
        gait_type = self.gait_type_var.get()
        gait_length = self.gait_length_var.get()
        frequency = self.hl_frequency_var.get()

        array_size = gait_length * frequency 

        if gait_type and gait_length and frequency: 
            if messagebox.askyesno(
                message="You are changing gait files with \nGait type: {} \nGait length in seconds: {} \nHL frequency: {}\n\nAre you sure?".format(gait_type, gait_length, frequency),
                title="Changing Gaits"
            ): 
                gait_functions = {
                'sit_to_stand': sit_to_stand,
                'stand_to_sit': stand_to_sit,
                'sideways': sideways,
                'hinge': hinge_gait
                }   

                if gait_type in gait_functions:
                    self.show_generating_message()
                    self.root.after(1001, self.root.destroy)
                    gait_functions[gait_type](int(array_size))
                else:
                    self.root.destroy()
                    interactive_bezier(gait_type, int(array_size))
        else:
            messagebox.showwarning("Input Error", "Please enter all the required fields.")
        
if __name__ == "__main__":
    root = tk.Tk()
    app = GUI(root)
    root.mainloop()