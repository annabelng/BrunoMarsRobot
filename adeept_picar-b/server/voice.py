import speech_recognition as sr
import time
from adafruit_servokit import ServoKit
#import RPIservo


#scGear = RPIservo.ServoCtrl()
#scGear.moveInit()
import auto_obstacle
import move
move.setup()
kit = ServoKit(channels=16)
# obtain audio from the microphone
#r = sr.Recognizer()
#mic = sr.Microphone(device_index=2)

def recognize_speech_from_mic(recognizer, microphone):
    """Transcribe speech from recorded from `microphone`.

    Returns a dictionary with three keys:
    "success": a boolean indicating whether or not the API request was
               successful
    "error":   `None` if no error occured, otherwise a string containing
               an error message if the API could not be reached or
               speech was unrecognizable
    "transcription": `None` if speech could not be transcribed,
               otherwise a string containing the transcribed text
    """
    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #     update the response object accordingly
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response


'''
with mic as source: #using laptop mic not ROBO CAR!!
    print("Say something!")
    r.adjust_for_ambient_noise()
    audio = r.listen(source)
    v_command = r.recognize_google(audio)
# recognize speech using Google Speech Recognition
try:
    print("Google Speech Recognition thinks you said " + r.recognize_google(audio))
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))
'''

if __name__ == "__main__":
    # set the list of words, maxnumber of guesses, and prompt limit
    WORDS = ["forward", "backward", "left", "right"]
    NUM_COMMANDS = 5 #number of tries
    RETRY_LIMIT = 3 #didn't transcribe, try again
    # create recognizer and mic instances
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    # show instructions and wait 1 seconds before starting the game
   # format the instructions string
    instructions = (
        "Here are the commands:\n"
        "{words}\n"
        "You have {n} commands to give\n"
    ).format(words=', '.join(WORDS), n=NUM_COMMANDS)

    # show instructions and wait 3 seconds before starting the game
    print(instructions)
    time.sleep(3)

    for i in range(NUM_COMMANDS):
        # get the guess from the user
        # if a transcription is returned, break out of the loop and
        #     continue
        # if no transcription returned and API request failed, break
        #     loop and continue
        # if API request succeeded but no transcription was returned,
        #     re-prompt the user to say their guess again. Do this up
        #     to PROMPT_LIMIT times
        for j in range(RETRY_LIMIT):
            print('Guess {}. Speak!'.format(i+1))
            command = recognize_speech_from_mic(recognizer, microphone)
            #if command["transcription"]:
            #    break
            if not command["success"]:
                break
            print("I didn't catch that. What did you say?\n")

        # if there was an error, stop the game
        if command["error"]:
            print("ERROR: {}".format(command["error"]))
            break

        # show the user the transcription
        print("You said: {}".format(command["transcription"]))

        # determine if guess is correct and if any attempts remain
        go_forwards = command["transcription"].lower() == "forward"
        go_backwards = command["transcription"].lower() == "backwards"
        go_left = command["transcription"].lower() == "left"
        go_right = command["transcription"].lower() == "right"

        user_has_more_attempts = i < NUM_COMMANDS - 1

        # determine if the user has won the game
        # if not, repeat the loop if user has more attempts
        # if no attempts left, the user loses the game
        if go_forwards:
            print("Going forwards")
            #scGear.moveAngle(2, 0)
            kit.continuous_servo[2].throttle = 0.2
            move.motor_left(1, 0, 50)
            move.motor_right(1, 0, 50)
            time.sleep(5)
            move.motorStop()

        if go_backwards:
            print("Going backwards")
            #scGear.moveAngle(2, 0)
            kit.continuous_servo[2].throttle = 0.2
            move.motor_left(1, 1, 50)
            move.motor_right(1, 1, 50)
            time.sleep(2)
            move.motorStop()
            #break
        if go_left:
            print("Going left")
            kit.continuous_servo[2].throttle = 0.4
            #scGear.moveAngle(2, 45)
            move.motor_left(1, 0, 50)
            move.motor_right(1, 0, 50)
            time.sleep(2)
            move.motorStop()
            #scGear.moveAngle(2, 0)

        if go_right:
            print("Going right")
            kit.continuous_servo[2].throttle = 0.4
            #scGear.moveAngle(2,-45)
            move.motor_left(1, 0, 50)
            move.motor_right(1, 0, 50)
            time.sleep(2)
            move.motorStop()
            #scGear.moveAngle(2, 0)

        else:
            print("Sorry, not a command")
            break


""""
if 'forward' in v_command:
  print("moving forward")
  #scGear.moveAngle(2, 0)
  #move.motor_left(1, 0, speed_set)
  #move.motor_right(1, 0, speed_set)
  #time.sleep(2)
  #move.motorStop()

elif 'backward' in v_command:
  print("moving backwards")


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

"""
