import boto3

class SpeechClient:
    def __init__(self):
        self.lex_client = boto3.client('lex-runtime')

        self.polly_client = boto3.client('polly')

    def sendAudio(self, audio, session_status):
        response = self.lex_client.post_content(
            botName="CoBo",
            botAlias="beta",
            userId="aaabkjsdgkdsjn", #### update later with cobo id if we have multiple robots
            contentType="audio/l16; rate=16000; channels=1",
            accept="audio/pcm",
            inputStream=audio
        )
        return response

    def sendCode(self, string, session_status):
        response = self.lex_client.post_content(
            botName="CoBo",
            botAlias="beta",
            userId="aaabkjsdgkdsjn", #### update later with cobo id if we have multiple robots
            contentType="text/plain; charset=utf-8",
            accept="audio/pcm",
            inputText=string,
            sessionAttributes= {'status' : session_status}
        )
        return response

