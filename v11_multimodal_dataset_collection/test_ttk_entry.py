import tkinter as tk
from tkinter import ttk
root = tk.Tk()
var = tk.StringVar(value="hello")
e = ttk.Entry(root, textvariable=var)
e.pack()
root.update()
print("Tkinter ttk.Entry initialized")
root.destroy()
