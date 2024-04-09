import json
import openai
from typing import TypeVar, Generic, Dict

BotType = TypeVar('BotType')


class CorrectionBot:
    '''
    A bot that generates a report that analyses why a provided code failed to solve the given task
    '''
    def __init__(self, bot: BotType):
        self.bot = bot
        self.task = ""
        self.docs= ""

    def set_task(self, task: str):
        self.task = task

    def set_docs(self, docs: str):
        self.docs = docs

    def _contextualize(self, task: str, generated_code: str, docs: str) -> str:
        prompt = f"Task: {task} | Generated code: {generated_code} | Docs: {docs}. Use the provided task description and the code generated \
            by another chatbot for solving the same task and provide a report that explains why the code fails to achive the goal."
        return prompt
    
    def generate_report(self, generated_code: str) -> str:
        prompt = self._contextualize(self.task, generated_code, self.docs)
        return self.bot.chat(prompt)