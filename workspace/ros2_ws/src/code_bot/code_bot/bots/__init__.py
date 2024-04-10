# __init__.py

# Import classes from the Python files within the package
from .DecisionBot import DecisionBot
from .CorrectionBot import CorrectionBot
from .EvaluationBot import EvaluationBot
from .ChatGPT import ChatGPT

# Define __all__ to specify what symbols will be exported when using 'from bots import *'
__all__ = ['DecisionBot', 'CorrectionBot', 'EvaluationBot', 'ChatGPT']