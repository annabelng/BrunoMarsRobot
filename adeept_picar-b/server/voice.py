import speech_recognition as sr

#import move
#move.setup()
# obtain audio from the microphone
r = sr.Recognizer()
mic = sr.Microphone()
sr.Microphone.list_microphone_names()
with sr.Microphone() as source: #using laptop mic not ROBO CAR!!
    print("Say something!")
    audio = r.listen(source)
    v_command = r.recognize_google(audio)
# recognize speech using Google Speech Recognition
try:
    print("Google Speech Recognition thinks you said " + r.recognize_google(audio))
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))


if 'forward' in v_command:
  print("moving forward")
  #scGear.moveAngle(2, 0)
  #move.motor_left(1, 0, speed_set)
  #move.motor_right(1, 0, speed_set)
  #time.sleep(2)
  #move.motorStop()

elif 'backward' in v_command:
  print("moving backwards")
  #scGear.moveAngle(2, 0)
  #move.motor_left(1, 1, speed_set)
  #move.motor_right(1, 1, speed_set)
  #time.sleep(2)
  #move.motorStop()

elif 'left' in v_command:
  print("moving left")
 # scGear.moveAngle(2, 45)
  #move.motor_left(1, 0, speed_set)
  #move.motor_right(1, 0, speed_set)
  #time.sleep(2)
  #move.motorStop()
  #scGear.moveAngle(2, 0)
elif "right" in v_command:
  print("moving right")
  #scGear.moveAngle(2,-45)
  #move.motor_left(1, 0, speed_set)
  #move.motor_right(1, 0, speed_set)
  #time.sleep(2)
  #move.motorStop()
  #scGear.moveAngle(2, 0)

elif 'stop' in v_command:
   #move.motorStop()  else:
   pass
 
