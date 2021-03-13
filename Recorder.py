import pyaudio
import math
import struct
import wave
import os
import time

from AWSClient import SpeechClient

import csv


### ADAPTED FROM:
### https://stackoverflow.com/questions/18406570/python-record-audio-on-detected-sound/18423188

Threshold = 18

SHORT_NORMALIZE = (1.0/32768.0)
chunk = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
swidth = 2

TIMEOUT_LENGTH = 0.5

f_name_directory = r'.\\records'

class Recorder:
    @staticmethod
    def rms(frame):
        count = len(frame) / swidth
        format = "%dh" % (count)
        shorts = struct.unpack(format, frame)

        sum_squares = 0.0
        for sample in shorts:
            n = sample * SHORT_NORMALIZE
            sum_squares += n * n
        rms = math.pow(sum_squares / count, 0.5)

        return rms * 1000

    def __init__(self):
        self.is_playing = False
        self.aws_client = SpeechClient()
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=FORMAT,
                                  channels=CHANNELS,
                                  rate=RATE,
                                  input=True,
                                  output=True,
                                  frames_per_buffer=chunk)

    def record(self):
        print('Noise detected, recording beginning')
        rec = []
        current = time.time()
        end = time.time() + TIMEOUT_LENGTH

        while current <= end:
            data = self.stream.read(chunk)
            if self.rms(data) >= Threshold:
                end = time.time() + TIMEOUT_LENGTH
            current = time.time()
            rec.append(data)
        # self.write(b''.join(rec)) ## uncomment to write to files
        return b''.join(rec)

    # def write(self, recording):
    #     n_files = len(os.listdir(f_name_directory))

    #     filename = os.path.join(f_name_directory, '{}.wav'.format(n_files))

    #     wf = wave.open(filename, 'wb')
    #     wf.setnchannels(CHANNELS)
    #     wf.setsampwidth(self.p.get_sample_size(FORMAT))
    #     wf.setframerate(RATE)
    #     wf.writeframes(recording)
    #     wf.close()
    #     print('Written to file: {}'.format(filename))
    #     print('Returning to listening')

    def listen(self):
        print('Listening beginning')
        while True:
            input = self.stream.read(chunk)
            rms_val = self.rms(input)
            if rms_val > Threshold and not self.is_playing:
                audio = self.record()
                time.sleep(TIMEOUT_LENGTH)
                return audio
    
    def playAudio(self, data):
        self.is_playing = True
        play_stream = self.p.open(
            format=self.p.get_sample_size(FORMAT),
            channels=1,
            rate=8000,
            input=False,
            output=True
        )
        play_stream.start_stream()
        play_stream.write(data)

        # time.sleep(1.0)

        play_stream.close()
        
        self.is_playing = False
        # self.audio.terminate()

def main():
    a = Recorder()
    status = 0
    while True:
        audio = a.listen()
        print(f'outgoing status is {status}')
        response = a.aws_client.respond(audio, status)
        session_attributes = response['sessionAttributes']

        status = int(session_attributes['status']) if 'status' in session_attributes else 0
        print(f'incoming status : {status}')
        if status == 2:
            scan_item_id = int(session_attributes['scanItemId'])
            if scan_item_id == 2:
                lex_audio = response['audioStream'].read()
                a.playAudio(lex_audio)

                print('activate QR scanner')
                time.sleep(4)
                print('scanning complete')
                status = 1
                session_attributes['status'] = status
                response = a.aws_client.respond('999', status)

            else:
                a.playAudio(a.aws_client.toAudio('scanning completed')['AudioStream'].read())
                if scan_item_id == 0:
                    status = 3
                elif scan_item_id == 1:
                    status = 4

                session_attributes['status'] = status
                response = a.aws_client.respond('.*.', status)

            
            print(f'status :  {status}')
            
            lex_audio = response['audioStream'].read() ## plays response
        else:
            lex_audio = response['audioStream'].read() ## plays response
        a.playAudio(lex_audio)
 
'''
cobo proactive for:
- wear mask
- follow me to booth
- testing fdone, follow to exit
- we are in exit good bye -fak of-
'''

def test():
    a = Recorder()
    response = a.aws_client.toAudio("hi my name is amy and you are watching the disney channel")
    sound = response["AudioStream"].read()
    a.playAudio(sound)

if __name__ == "__main__":
    # test()

    main()