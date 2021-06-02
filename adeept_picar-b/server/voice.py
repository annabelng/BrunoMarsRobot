import speech_recognition as sr

r1 = sr.Recognizer()
r2 = sr.Recognizer()
r3 = sr.Recognizer()

#set mic as the source
def run():
  r = sr.Recognizer()
  with sr.Microphone() as source:                # use the default microphone as the audio source
    audio = r.listen(source)                   # listen for the first phrase and extract it into audio data

try:
    print("You said " + r.recognize(audio))    # recognize speech using Google Speech Recognition
except LookupError:                            # speech is unintelligible
    print("Could not understand audio")

try:
        v_command = r.recognize_sphinx(audio,
        keyword_entries=[('forward',1.0),('backward',1.0),
        ('left',1.0),('right',1.0),('stop',1.0)])
        #You can add your own command here
        print(v_command)
       # RL.both_off()
       # RL.cyan()
    except sr.UnknownValueError:
        print("say again")
       # RL.both_off()
       # RL.red()
    except sr.RequestError as e:
       # RL.both_off()
       # RL.red()
        pass