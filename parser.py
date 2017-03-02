from __future__ import print_function
import transcript as t
import sys

transcriber = t.Transcriber()

transcription = transcriber.Transcribe(sys.argv[1])

move_commands = ['move', 'go', 'head']
full_command = {'command':None, 'destination':None}
trash = ['to', 'please']

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
        #elif word in trash:
            #print("Removing: " + "'" + word + "'")
            #transcript.remove(word)
        else:
            if word in trash:
                print("Skipping: " + "'" + word + "'")
            else:
                print("Adding: " + "'" + word + "'" + " to destination.")
                destination += word
                destination += ' '
    full_command['destination'] = destination

    print()
    print("full_command dict: ", end="")
    print(full_command)

Parse(transcription)
