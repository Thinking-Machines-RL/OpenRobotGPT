import json
import openai
from typing import TypeVar, Generic, Dict

BotType = TypeVar('BotType')


class EvaluationBot:
    '''
    Bot that checks if the generated code succeeds or not
    '''
    def __init__(self, bot: BotType):
        self.bot = bot
        self.task = ""
        self.docs = ""

    def set_task(self, task: str):
        self.task = task

    def set_docs(self, docs: str):
        self.docs = docs

    def _contextualize(self, task: str, generated_code: str, docs: str) -> str:
        prompt = f"Task: {task} | Docs: {docs}. Provide Python code that analyses the ability of another Python code, \
            provided as input, to solve the specified task succesfully. The code should define a function 'check_function' \
            that takes the code to be checked as input and return a boolean (True=success, False=failure)."
        return prompt
    
    def generate_check_code(self, generated_code: str) -> str:
        prompt = self._contextualize(self.task, generated_code, self.docs)
        return self.bot.chat(prompt)
    
    def check_code(self, code: str) -> bool:
        '''
        !!! Probably should execute the code on another node that includes also the simulation.
        '''
        local_vars = {}
        exec(self.generate_check_code(code), globals(), local_vars)
        func = local_vars.get("check_function")

        if func is None:
            raise ValueError("Function 'check_function' not found in the provided code")
        
        try:
            result = func(code)
            return result
        except Exception as e:
            print("Error occurred while executing code:", e)
            return None