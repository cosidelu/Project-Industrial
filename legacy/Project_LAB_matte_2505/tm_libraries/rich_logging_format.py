import logging
from rich.logging import RichHandler

def rich_logger():
    FORMAT = '%(message)s'
    logging.basicConfig(
        level=logging.INFO, format=FORMAT, datefmt="[%X]", handlers=[RichHandler()]
    )
    log = logging.getLogger("__name__")
    return log