import tkinter as tk
from tkinter import ttk, scrolledtext
root = tk.Tk()
main_frame = ttk.Frame(root, padding=10)
main_frame.pack(fill=tk.BOTH, expand=True)

title_label = ttk.Label(main_frame, text="Test", font=("Arial", 16, "bold"))
title_label.pack(pady=(0, 15))

ctrl_frame = ttk.LabelFrame(main_frame, text="Test Frame", padding=10)
ctrl_frame.pack(fill=tk.X, pady=5)

for i in range(4):
    cmd_var = tk.StringVar(value="hello")
    cmd_entry = ttk.Entry(ctrl_frame, textvariable=cmd_var, width=55)
    cmd_entry.grid(row=i, column=2, padx=10, pady=5)

root.update()
print("Success")
root.destroy()
