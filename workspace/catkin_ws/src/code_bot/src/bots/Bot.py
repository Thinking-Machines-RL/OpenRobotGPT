import json
import openai
from typing import TypeVar, Generic, Dict

BotType = TypeVar('BotType')

class Bot:
    '''
    Generic interface for an LLM
    '''
    def __init__(self):
        '''
        Initialization
        '''
        return NotImplementedError("This is a generic interface for a chatbot. Do not instantiate but write a class that inherits from Bot instead.")

    def set_context(self, context):
        self.context = context

    def chat(self, prompt: str):
        '''
        Send LLM a prompt and return response
        '''
        pass
