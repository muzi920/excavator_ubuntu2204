import tkinter as tk
from tkinter import ttk
root = tk.Tk()
var = tk.StringVar(value="hello")
# e = ttk.Entry(root, textvariable=var)
e = tk.Entry(root, textvariable=var)
e.pack()
root.update()
print("Tkinter tk.Entry initialized")
root.destroy()
