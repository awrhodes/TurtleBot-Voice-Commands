from __future__ import print_function
import transcript as t
import sys
import os
import random

transcriber = t.Transcriber()

transcription = transcriber.Transcribe(sys.argv[1])

move_commands = ['move', 'go', 'head']
full_command = {'command':None, 'destination':None}
trash = ['to', 'please']
aff_resp = ['Okay.', 'Sure thing.', 'Will do.', 'Roger roger.']
neg_resp = ["I didn't catch that.", "I didn't understand."]

name = "Dixon"

#parse transcription for keywords
def Parse(transcript):
    destination = ""
    for word in transcript:
        print()
        print("Parsing: " + "'" + word + "'")
        if word in move_commands: #if current word is a known command
            if full_command['command'] is None: #if no command is given
                print("Adding: " + "'" + word + "'" + " to command.")
                full_command['command'] = word #add it to the command dict 
        else:
            if word in trash or word in name:
                print("Skipping: " + "'" + word + "'")
            else:
                print("Adding: " + "'" + word + "'" + " to destination.")
                destination += word
                destination += ' '
    full_command['destination'] = destination

    print()
    print("full_command dict: ", end="")
    print(full_command)

def nameCheck(name, transcription, audio):
    if name not in transcription:
        print("Name not detected, deleting file.")
        os.remove(audio)
        return False
    else:
        return True

def stateChecker(command, aff, neg):
    if command['command'] == None:
        print(neg[random.randrange(len(neg))])
    else:
        print(aff[random.randrange(len(aff))])

if nameCheck(name, transcription, transcriber.audio):
    Parse(transcription)
    stateChecker(full_command, aff_resp, neg_resp)
