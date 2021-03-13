import boto3

class SpeechClient:
    def __init__(self):
        self.lex_client = boto3.client('lex-runtime')

        self.polly_client = boto3.client('polly')

    def respond(self, data, session_status):
        response = self.lex_client.post_content(
            botName="CoBo",
            botAlias="charlie",
            userId="asdasada", #### update later with cobo id if we have multiple robots
            contentType= "text/plain; charset=utf-8" if type(data) == type('string') else "audio/l16; rate=16000; channels=1",
            accept="audio/pcm",
            inputStream=data,
            sessionAttributes= {'status' : session_status}
        )
        print(response["intentName"] if "intentName" in response else response)
        print(response["botVersion"])
        return response

    def toAudio(self, text):
        response = self.polly_client.synthesize_speech(
            Text=text,
            VoiceId="Amy",
            Engine="neural",
            OutputFormat="pcm",
        )
        return response

