import json
import openai
from typing import TypeVar, Generic, Dict
from .Bot import Bot

BotType = TypeVar('BotType')

class ChatGPT(Bot):
    '''
    Interface to ChatGPT
    '''
    def __init__(self, config_file: str, api_key: str):
        self.config = self._load_config(config_file)
        self.api_key = self._load_config(api_key)
        self.api = openai.OpenAI(api_key=self.api_key['api_key'], organization=self.api_key['organization'])
        self.set_context([])

    def _load_config(self, filename: str) -> Dict:
        with open(filename, 'r') as f:
            config = json.load(f)
        return config
    
    

    def chat(self, prompt: str) -> str:
        # Interact with ChatGPT
        response = self.api.chat.completions.create(
            model=self.config['model'],
            messages=self.context + [{"role": "user", "content": prompt}],
            temperature=self.config['temperature'],
            max_tokens=self.config['max_tokens']
        )

        # Expand context (in case we want conversation history)
        history = self.context + [
            {"role": "user", "content": prompt}, 
            {"role": "user", "content":response.choices[0].message.content}
            ]
        # self.contex = history

        return response.choices[0].message.content
    
def main():
    bot = ChatGPT("../config/config_bot.json", "../secrets/api_key.json")
    prompt = input("Insert prompt here: ")
    answer = bot.chat(prompt)
    print("\nChatGPT: " + answer)

if __name__ == "__main__":
    main()