import tkinter as tk

root = tk.Tk()
root.title("Tkinter smoke test")
root.geometry("700x500")
root.configure(bg="white")

canvas = tk.Canvas(root, width=700, height=500, bg="white")
canvas.pack(fill="both", expand=True)

canvas.create_rectangle(50, 50, 650, 450, fill="#dbeafe", outline="blue", width=4)
canvas.create_oval(280, 180, 420, 320, fill="#2563eb")
canvas.create_text(
    350, 100,
    text="TKINTER IS DRAWING CORRECTLY",
    fill="black",
    font=("Arial", 20, "bold"),
)

root.mainloop()