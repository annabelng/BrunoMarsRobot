import speech_recognition as sr;print(sr.__version__)
import pyaudio as p;print(p.__version__)

print(sr.Microphone.list_microphone_names())