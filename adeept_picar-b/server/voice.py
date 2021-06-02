import speech_recognition as sr

r1 = sr.Recognizer()
r2 = sr.Recognizer()
r3 = sr.Recognizer()

#set mic as the source
with sr.Microphone() as source:
    print('turn left: turn right') # 2 options liste for user 
    print('speak now')
    audio = r3.listen(source) #get sounds from mic

#if user says left
#transaltes audio to text

#if 'left' in r2.recognize_google(audio): 
   # r2 = sr.Recognizer()
    #url
  #  with sr.Microphone() as source: 
