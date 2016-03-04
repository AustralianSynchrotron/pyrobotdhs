import click
from aspyrobotmx import RobotClientMX
from . import RobotDHS
from .config import logging_config
import logging
import logging.config
from time import sleep


@click.command()
@click.option('--dcss', required=True)
def run(dcss):
    logging.config.dictConfig(logging_config)
    dhs = RobotDHS(dcss=dcss, robot=RobotClientMX())
    dhs.setup()
    while True:
        sleep(.01)
