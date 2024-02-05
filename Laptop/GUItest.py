import cv2
import customtkinter as ctk
from PIL import Image, ImageTk
import time
from customtkinter import *

# Set default appearance
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Set size of window
app = ctk.CTk()
app.title("OpenCV Video Feed")
app.geometry("1280x720")

# custom canvas widget
canvas = ctk.CTkCanvas(app, width = 800, height = 500)
canvas.pack()
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, width) 
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
rep_count = 0
error_count = 0
start_time = time.time()

# Functions
def change_appearance_mode_event(new_appearance_mode: str):
    ctk.set_appearance_mode(new_appearance_mode)

def sidebar_button_event():
    print("sidebar_button click")

def update_frame():
     ret, frame = cap.read()
     if ret:
         # convert to image
         cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
         img = Image.fromarray(cv_img)
         img_tk = ctk.CTkImage(light_image=img, dark_image=img)
         # display image
         lbl_video.imgtk = img_tk
         lbl_video.configure(image=img_tk)

         # frame update parameter
         lbl_video.after(20, update_frame)

def increase_rep(event):
    global rep_count
    rep_count += 1
    lbl_rep.configure(text=f"Rep Count: {rep_count}")

def increase_error(event):
    global error_count
    error_count += 1
    lbl_error.configure(text=f"Error Count: {error_count}")
    
# Code for Side Bar and Appearance Mode Switch
sidebar_frame = ctk.CTkFrame(master=app, width=140, corner_radius=0)
logo_label = ctk.CTkLabel(sidebar_frame, text="CustomTkinter", font=ctk.CTkFont(size=20, weight="bold"))
logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))
sidebar_button_1 = ctk.CTkButton(sidebar_frame, command=sidebar_button_event)
sidebar_button_1.grid(row=1, column=0, padx=20, pady=10)
sidebar_button_2 = ctk.CTkButton(sidebar_frame, command=sidebar_button_event)
sidebar_button_2.grid(row=2, column=0, padx=20, pady=10)
sidebar_button_3 = ctk.CTkButton(sidebar_frame, command=sidebar_button_event)
sidebar_button_3.grid(row=3, column=0, padx=20, pady=10)
appearance_mode_label = ctk.CTkLabel(sidebar_frame, text="Appearance Mode:", anchor="w")
appearance_mode_label.grid(row=5, column=0, padx=20, pady=(10, 0))
appearance_mode_optionemenu = ctk.CTkOptionMenu(sidebar_frame, values=["Light", "Dark", "System"],
                                                               command=change_appearance_mode_event)
appearance_mode_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))


# initialize OpenCV video capture
cap = cv2.VideoCapture(0) # my webcam is 1, may have to change to 0 depending

# Initialize Frames
rep_frame = ctk.CTkFrame(master=app, fg_color = "#5BA7DA", corner_radius = 10)
rep_frame.pack(anchor = "w", padx = 200, pady = 5)
err_frame = ctk.CTkFrame(app, fg_color = "#5BA7DA")
err_frame.pack(anchor = "w", padx = 200, pady = 5)
time_frame = ctk.CTkFrame(app, fg_color = "#5BA7DA")
time_frame.pack(anchor = "w", padx = 200, pady = 5)

#for test purposes these are binded to keys, update to event you have
app.bind("<r>", increase_rep)
app.bind("<e>", increase_error)

# video display label
lbl_video = ctk.CTkLabel(app)
lbl_video.pack(padx=10, pady=10)

# Label version
lbl_rep = ctk.CTkLabel(rep_frame, text="Rep Count: 0", text_color = "white", font=("Calibri", 50), padx = 10, pady = 25)
lbl_rep.pack(anchor = "w")
lbl_error = ctk.CTkLabel(err_frame, text="Error Count: 0",text_color = "white", font=("Calibri", 50),  padx = 10, pady = 25)
lbl_error.pack(anchor = "w")
lbl_time = ctk.CTkLabel(time_frame, text="Time elapsed: 0 Seconds",text_color = "white", font=("Calibri", 50),  padx = 10, pady = 25)
lbl_time.pack(anchor = "w")

update_frame()

app.mainloop()

cap.release()