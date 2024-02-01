import cv2
import customtkinter as ctk
from PIL import Image, ImageTk
import time
from customtkinter import *
# window makin
app = ctk.CTk()
app.title("OpenCV Video Feed")
app.geometry("800x600")

ctk.set_appearance_mode("System")  # Modes: system (default), light, dark
ctk.set_default_color_theme("green")  # Themes: blue (default), dark-blue, green

#opencv
cap = cv2.VideoCapture(0) # my webcam is 1, may have to change to 0 depending
width, height = 500, 500 # Width of camera, #Height of Camera 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width) 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
rep_count = 0
error_count = 0
start_time = time.time()



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


#for test purposes these are binded to keys, update to event you have
app.bind("<r>", increase_rep)
app.bind("<e>", increase_error)

#video display label
lbl_video = ctk.CTkLabel(app)
lbl_video.pack(padx=10, pady=10)

#labels for rep count, error count, and time elapsed
lbl_rep = ctk.CTkButton(master=app, text="Rep Count: 0", font=("Arial", 50), border_width=2, border_color="red")
lbl_rep.place(relx=0.5, rely=0.5, anchor=ctk.CENTER)
lbl_rep.pack()
lbl_error = ctk.CTkButton(app, text="Error Count: 0", font=("Times New Roman", 35),  border_width=2, border_color = "green")
lbl_error.pack()
lbl_time = ctk.CTkButton(app, text="Time Elapsed: 0 seconds",  border_width=2)
lbl_time.pack()

update_frame()


app.mainloop()


cap.release()



