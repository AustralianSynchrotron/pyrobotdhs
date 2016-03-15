import click
from aspyrobotmx import RobotClientMX
from . import RobotDHS
import json
import logging.config
from time import sleep


@click.command()
@click.option('--dcss', required=True)
@click.option('--config', type=click.Path(exists=True))
def run(dcss, config):
    if config:
        with open(config) as file:
            config = json.load(file)
        if 'logging' in config:
            logging.config.dictConfig(config['logging'])
    dhs = RobotDHS(dcss=dcss, robot=RobotClientMX())
    dhs.setup()
    while True:
        sleep(.01)
