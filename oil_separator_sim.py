"""
3-Phase Oil Separator Demo Backend

Filename: oil_separator_sim.py
Author: Liam McNaughton

Description: Main file to run the backend for the oil separator demo system
"""

ACTUATOR_POS_SCALE = 100

# Standard Library Imports
import argparse
import logging

# Internal Dependency Imports
import separator_classes

# Configure logger
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(asctime)s - %(message)s')

# Initialise parser
parser = argparse.ArgumentParser()
parser.add_argument("-b", "--Broker", help="MQTT broker IP address to connect to. Leave empty to not connect to MQTT")
args = parser.parse_args()

# Sets up MQTT variables depending on args
if args.Broker:
    logging.info(f"Broker provided. MQTT enabled, connecting to broker {args.Broker}.")
    mqtt_connect_choice = True
    broker_ip = args.Broker
else:
    logging.warning("Broker not provided. MQTT not enabled.")
    mqtt_connect_choice = False
    broker_ip = ""

# Example Broker IP's
# broker_ip_1 = "192.168.1.3" # IP of Flow rig broker
# broker_ip_2 = "broker.emqx.io" # Public testing broker

demo_separator = separator_classes.Separator(mqtt_connect=mqtt_connect_choice, mqtt_broker=broker_ip)
demo_separator.input_rate = 2
demo_separator.gas_valve.PID_data["setpoint"] = 20000

def main():
    logging.info("Simulation starting now...")
    while True:
        try:
            demo_separator.runSimulationFrame_AUTO()
            demo_separator.updateMQTTData()

            # Publish position demands to actuators on the flow rig
            if demo_separator.mqtt_active:
                demo_separator.mqtt_client.publish("Actuator/IQT/Demand_Position", demo_separator.oil_valve.position*ACTUATOR_POS_SCALE)
                demo_separator.mqtt_client.publish("Actuator/CVA/Demand_Position", demo_separator.gas_valve.position*ACTUATOR_POS_SCALE)

        except KeyboardInterrupt:
            logging.info("Program terminated by user, exiting...")
            break
        except Exception as e:
            logging.error(f"Unexpected runtime error occurred: {e}")

if __name__ == "__main__":
    main()