import pygame
import time

import asyncio
import argparse
import logging
import os
from pathlib import Path

from kuksa_client.grpc import Datapoint
from kuksa_client.grpc import DataEntry
from kuksa_client.grpc import EntryUpdate
from kuksa_client.grpc import Field
from kuksa_client.grpc import VSSClientError
from kuksa_client.grpc.aio import VSSClient


def init_argparse() -> argparse.ArgumentParser:
    """This inits the argument parser for the CSV-provider."""
    parser = argparse.ArgumentParser(
        usage="-a [BROKER ADDRESS] -p [BROKER PORT] -f [FILE]",
        description="This provider writes the content of a csv file to a kuksa.val databroker",
    )
    environment = os.environ
    parser.add_argument(
        "-a",
        "--address",
        default=environment.get("KUKSA_DATA_BROKER_ADDR", "127.0.0.1"),
        help="This indicates the address of the kuksa.val databroker to connect to."
        " The default value is 127.0.0.1",
    )
    parser.add_argument(
        "-p",
        "--port",
        default=environment.get("KUKSA_DATA_BROKER_PORT", "55555"),
        help="This indicates the port of the kuksa.val databroker to connect to."
        " The default value is 55555",
        type=int,
    )
    parser.add_argument(
        "-l",
        "--log",
        default=environment.get("PROVIDER_LOG_LEVEL", "INFO"),
        help="This sets the logging level. The default value is WARNING.",
        choices={"INFO", "DEBUG", "WARNING", "ERROR", "CRITICAL"},
    )
    parser.add_argument(
        "--cacertificate",
        help="Specify the path to your CA.pem. If used provider will connect using TLS",
        nargs="?",
        default=None,
    )
    parser.add_argument(
        "--tls-server-name",
        help="TLS server name, may be needed if addressing a server by IP-name",
        nargs="?",
        default=None,
    )
    return parser


async def main():
    """the main function as entry point for the CSV-provider"""
    parser = init_argparse()
    args = parser.parse_args()
    numeric_value = getattr(logging, args.log.upper(), None)
    if args.cacertificate:
        root_path = Path(args.cacertificate)
    else:
        root_path = None
    if isinstance(numeric_value, int):
        logging.basicConfig(encoding="utf-8", level=numeric_value)

    joystick = initialize_joystick()

    if joystick==None:
        print("Joystick not initialized.")
    else:
        try:
            async with VSSClient(
                args.address,
                args.port,
                root_certificates=root_path,
                tls_server_name=args.tls_server_name,
            ) as client:
                await vehicle_control(client, joystick)
        except VSSClientError:
            logging.error(
                "Could not connect to the kuksa.val databroker at %s:%s."
                " Make sure to set the correct connection details using --address and --port"
                " and that the kuksa.val databroker is running.",
                args.address,
                args.port,
            )


async def pub_value(client, rows):
    for row in rows:
        if row["field"] == "current":
            # print(row['field'])
            entry = DataEntry(
                row["signal"],
                value=Datapoint(value=row["value"]),
            )
            updates = (EntryUpdate(entry, (Field.VALUE,)),)
            logging.info(
                "Update current value of %s to %s", row["signal"], row["value"]
            )
        elif row["field"] == "target":
            # print(row['field'])
            entry = DataEntry(
                row["signal"], actuator_target=Datapoint(value=row["value"])
            )
            updates = (EntryUpdate(entry, (Field.ACTUATOR_TARGET,)),)
            logging.info("Update target value of %s to %s", row["signal"], row["value"])
        else:
            updates = []
        try:
            await client.set(updates=updates)
        except VSSClientError as ex:
            logging.error("Error while updating %s\n%s", row["signal"], ex)
        try:
            await asyncio.sleep(delay=float(row["delay"]))
        except ValueError:
            logging.error(
                "Error while waiting for %s seconds after updating %s to %s."
                " Make sure to only use numbers for the delay value.",
                row["delay"],
                row["signal"],
                row["value"],
            )


async def vehicle_control(client, joystick):
    # import RPi.GPIO as GPIO
    while True:
        pygame.event.pump()

        # Get the values of the X and Y axes
        x_axis = joystick.get_axis(0)
        if abs(x_axis) < 0.04:
            x_axis = 0.0
        y_axis = joystick.get_axis(1)
        if abs(y_axis) < 0.1:
           y_axis = 0.0

        default_value = [
             {
                "field": "target",
                "signal": "Vehicle.Teleoperation.IsEnabled",
                "value": "TRUE",
                "delay": "0",
            },
            {
                "field": "target",
                "signal": "Vehicle.Powertrain.Transmission.ClutchEngagement",
                "value": "0",
                "delay": "0",
            },
            {
                "field": "target",
                "signal": "Vehicle.Powertrain.Transmission.SelectedGear",
                "value": "3",
                "delay": "0",
            },
            ]

        value = default_value
        if -y_axis < -0.99:
            value.extend(
                [{
                "field": "target",
                "signal": "Vehicle.Teleoperation.Brake",
                "value": "1",
                "delay": "0",
            },
            {
                "field": "target",
                "signal": "Vehicle.Teleoperation.SteeringAngle",
                "value": 0.0,
                "delay": "0",
            },
            {
                "field": "target",
                "signal": "Vehicle.Teleoperation.Torque",
                "value": 0.0,
                "delay": "0",
            }])
        else:
            value.extend(
            [{
                "field": "target",
                "signal": "Vehicle.Teleoperation.Brake",
                "value": "0",
                "delay": "0",
            },
            {
                "field": "target",
                "signal": "Vehicle.Teleoperation.SteeringAngle",
                "value": x_axis,
                "delay": "0",
            },
            {
                "field": "target",
                "signal": "Vehicle.Teleoperation.Torque",
                "value": -y_axis,
                "delay": "0",
            }])

        await pub_value(client, value)
        time.sleep(0.1)

def initialize_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joysticks found.")
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Initialized joystick: {joystick.get_name()}")
    return joystick

asyncio.run(main())
