"""
3-Phase Oil Separator Classes

Filename: separator_classes.py
Author: Liam McNaughton

Description: Classes used to simulate a 3-Phase Horizontal Separator process
"""

# Standard Library Imports
from time import sleep
import json
from datetime import datetime
import logging

# Third-Party Imports
import paho.mqtt.client as mqtt

# Constants
R = 8.314 # Gas constant
T = 300 # Temperature standard, Kelvin
MOLAR_VOLUME_STP = 0.0224
VALVE_POS_MAX = 100
VALVE_POS_MIN = 0
WEIR_POSITION = 0.75
OVERFLOW_FACTOR = 0.1
ONE_ATM = 1000 # pascal
POSITION_SCALE = 100

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(asctime)s - %(message)s')

def roundTwoDP(num: int | float):
    """
    Rounds the given number to 2 decimal places.

    Args:
        num (int/float): Number to be rounded. Can accept int but most useful for float.

    Returns:
        int/float: num rounded to 2 decimal places.
    """
    return round(num, 2)

def on_connect(client, userdata, flags, reason_code, properties):
    """
    Callback function called when the MQTT client connects to the broker.
    Only prints a terminal message with the current date and time and the reason code of connection.

    Parameters:
        client (mqtt.Client): The MQTT client instance.
        userdata (dict): User data passed to the client.
        flags (dict): Response flags from the broker.
        reason_code (mqtt.ReasonCode): The connection result code.
        properties (dict): MQTT v5 properties.
    """
    print(f"[>] {datetime.now()} Connected with reason code {reason_code}")

def on_publish(client, userdata, mid, reason_code, properties):
    """
    Callback function called when the MQTT client publishes a message to the broker.
    Only prints a terminal message confirmation with the current date and time

    Parameters:
        client (mqtt.Client): The MQTT client instance.
        userdata (dict): User data passed to the client.
        mid (int): Message ID of published message.
        reason_code (mqtt.ReasonCode): The connection result code.
        properties (dict): MQTT v5 properties.
    """
    print(f"[>] {datetime.now()} Published message successfully")

class ActuatedValve():
    """
    Class representing a valve that is being controlled via an actuator.
    Intended as a basic model and parent class to other more refined types of control.

    Attributes:
        max_flow (int/float): The flow through the 'pipe' when the valve is fully open.
        command (int/float): The command position that the actuator should move to.
        position (int/float): The current position of the actuated valve.
        speed (int/float): The change in the actuator's position every time frame.
    """
    def __init__(self, max_flow: int | float, speed: int | float):
        """
        Initialises an ActuatedValve object.

        Parameters: 
            max_flow (int/float): The flow through the 'pipe' when the valve is fully open.
            speed (int/float): The change in the actuator's position every time frame.
        """
        if not isinstance(max_flow, (int, float)) or max_flow <= 0:
            raise ValueError("max_flow must be int/float and be greater than 0")
        if not isinstance(speed, (int, float)) or speed <= 0:
            raise ValueError("speed must be of type float and be greater than 0")

        self.max_flow = max_flow
        self.command = 0
        self.position = 0
        self.speed = speed

    def updatePosition(self):
        """
        Moves the valve slightly (by the speed value) every frame if the command is different to the position.

        """
        if self.command > self.position:
            self.position = roundTwoDP(self.position+self.speed)
        elif self.command < self.position:
            self.position = roundTwoDP(self.position-self.speed)
        else:
            pass # Position is at command

        # Ensure valve position is within bounds
        self.position = max(VALVE_POS_MIN, min(VALVE_POS_MAX, self.position))

    def returnFlow(self):
        """
        Calculates the flow through the 'pipe' based on the max_flow and the valve position.

        Returns:
            int/float: The correctly proportioned flow rate depending on the valve position.
        """
        try:
            return roundTwoDP(self.max_flow*(self.position/POSITION_SCALE))
        except ZeroDivisionError:
            return 0
        
class IsolatingValve(ActuatedValve):
    """
        Class that models an isolating type actuated valve, inherits from the ActuatedValve parent class.
        Takes in an upper and lower limit and opens and closes the valve fully at these levels.

        Attributes:
            upper_limit (int): sensor reading limit at which to open the actuator.
            lower_limit (int): sensor reading limit at which to close the actuator.
            max_flow (int/float): The flow through the 'pipe' when the valve is fully open.
            command (int/float): The command position that the actuator should move to.
            position (int/float): The current position of the actuated valve.
            speed (int/float): The change in the actuator's position every time frame.
    """
    def __init__(self, upper_limit: int | float, lower_limit: int | float, max_flow: int | float, speed: int | float):
        """
            Initialises an IsolatingValve object.

            Parameters:
                upper_limit (int): sensor reading at which the actuator will open.
                lower_limit (int): sensor reading at which the actuator will close.
                max_flow (int/float): The flow through the valve when the actuator is fully open.
                speed (int/float): The change in actuator position every time frame.
        """
        if not isinstance(upper_limit, (int, float)) or upper_limit <= 0:
            raise ValueError("upper_limit must be of type int/float and be greater than 0")
        if not isinstance(lower_limit, (int, float)) or lower_limit == 0:
            raise ValueError("lower_limit must be of type int/float and be greater than 0")

        self.upper_limit = upper_limit
        self.lower_limit = lower_limit

        super().__init__(max_flow=max_flow, speed=speed)

    def updateCommand(self, sensor_reading: int | float):
        """
            Checks a sensor reading against the limits of the valve object and updates the command appropriately.

            Parameters:
                sensor_reading (int/float): The input sensor reading to check against the limits.
        """
        if sensor_reading > self.upper_limit:
            self.command = VALVE_POS_MAX
        elif sensor_reading < self.lower_limit:
            self.command = VALVE_POS_MIN
        else:
            pass # Sensor reading is within bounds

class ModulatingValve(ActuatedValve):
    """
        Class that models a modulating/control valve, inherits from the ActuatedValve parent class.
        Performs a PID control loop on the valve to maintain an external value at a particular setpoint.
        PID parameters are hard-coded but can be altered to act on different systems. Tuned to this particular use case. 

        Attributes:
            PID_data (dict): Dictionary that contains all the PID control parameters. KP, Ki, Kd, dt, integral, prev_error, command_output, and setpoint.
            max_flow (int/float): The flow through the valve when the actuator is fully open.
            speed (int/float): The change in actuator position every time frame.
    """
    def __init__(self, max_flow: int | float, setpoint: int, dt: float, speed: int | float,
                 kp: float=0.04, ki: float=0.00095, kd: float=0.0):
        """
            Initialises a ModulatingValve object.

            Parameters:
                max_flow (int/float): The flow through the valve when the actuator is fully open.
                setpoint (int): The setpoint that the control algorithm aims toward.
                dt (float): The timestep for each control loop.
                speed (int/float): The change in actuator position every time frame.
        """
        if not isinstance(dt, float) or dt <= 0:
            raise ValueError("dt must be of type float and be greater than 0")
        if not isinstance(setpoint, int):
            raise ValueError("setpoint must be of type int")

        self.PID_data = {
            "Kp" : kp,
            "Ki" : ki,
            "Kd" : kd,
            "dt" : dt,
            "integral" : 0,
            "prev_error" : 0,
            "command_output" : 0,
            "setpoint" : setpoint
        }

        super().__init__(max_flow=max_flow, speed=speed)

    def updateController(self, sensor_reading: int | float, noisy=False):
        """
            Updates the control command given to the actuator based on a PID control of a sensor reading.

            Parameters:     
                sensor_reading (int/float): The input sensor reading to check against the setpoint.
                noisy (bool): Set to True to output messages to the terminal about PID updates. False by default.
        """
        error = self.PID_data["setpoint"] - sensor_reading
        self.PID_data["integral"] += error*self.PID_data["dt"]
        if self.PID_data["dt"] > 0: # prevent divide by zero error possibility
            derivative = (error-self.PID_data["prev_error"]/self.PID_data["dt"])
        else:
            derivative = 0
        
        self.PID_data["command_output"] = 100 - ((self.PID_data["Kp"]*error) + (self.PID_data["Ki"]*self.PID_data["integral"]) + (self.PID_data["Kd"]*derivative))
        self.PID_data["prev_error"] = error

        if noisy:
            print("Setpoint: ", self.PID_data["setpoint"])
            print("Pressure Sensor: ", sensor_reading)
            print("Error: ", error)
            print("PID Output: ", self.PID_data["command_output"])

        # limit command output between 0 and 100
        if self.PID_data["command_output"] < VALVE_POS_MIN:
            self.PID_data["command_output"] = VALVE_POS_MIN
        elif self.PID_data["command_output"] > VALVE_POS_MAX:
            self.PID_data["command_output"] = VALVE_POS_MAX
        else:
            pass # command output is within bounds
        
        self.command = self.PID_data["command_output"]

        if noisy:
            print("Gas Valve Pos Command: ", self.command)
            print("Gas Valve Pos: ", self.position)

class Separator():
    """
        Class that models a 3-Phase Separator tank. Crude oil come in at an inlet and separates into water, gas, and product oil.
        The tank has 3 valves, 2 for dumping water and product oil, and 1 for controlling the pressure in the tank by exhausting gas.

        Attributes:
            dt (float): Timestep between each simulation frame and PID timing.

            total_volume (int): 2D representation of the tank's total volume.
            weir_height (int/float): Height of the internal weir at which oil spills over into the product side.
            input_rate (int/float): Amount of crude input to the tank every time frame.

            oil_fraction (float): Proportion of input that consists of oil.
            gas_fraction (float): Proportion of input that consists of gas.
            wat_fraction (float): Proportion of input that consists of water.

            oil_outflow_max (int): Maximum flow rate of product oil out per time frame.
            oil_outflow (float): Current flow rate of product oil out per time frame.
            gas_outflow_max (int): Maximum flow rate of gas out per time frame.
            gas_outflow (float): Current flow rate of gas out per time frame.
            wat_outflow_max (int): Maximum flow rate of water out per time frame.
            wat_outflow (float): Current flow rate of water out per time frame.

            pressure (float): Simulated internal pressure of the tank based on molar gas and volume remaining.
            gas_mol (float): Moles of gas in the system based on a volume of input.

            wat_level (float): Current level of water in the tank.
            wat_min (float): Minimum level alarm for water.
            wat_max (float): Maximum level alarm for water.

            oil_level (float): Current level of oil in the tank before it's spilled over the weir (not product). 

            oil_min (float): Minimum level alarm of product oil.
            oil_max (float): Maximum level alarm of product oil.
            product_level (float): Current level of product oil in the tank.

            total_level (float): The top level in the input side of the tank (water level + oil level).
            
            wat_valve (IsolatingValve): Dump valve that controls the level of water in the tank. Speed = 10 by default.
            oil_valve (IsolatingValve): Dump valve that controls the level of produc oil in the tank. Speed = 5 by default.
            gas_valve (ModulatingValve): Control valve that controls the pressure in the tank by exhausting gas. Setpoint = 10000, Speed = 1 by default.

            mqtt_json (str): Holds the data about the separator in JSON string format for sending to the MQTT broker.
            mqtt_active (bool): True if the separator is set to connect to MQTT
            mqtt_client (mqtt.Client): MQTT handler from the paho module, handles connecting, subscribing and publishing to the broker.
            publish_topic (str): Topic to which the data is published about the separator simulator.
    """
    def __init__(self, mqtt_connect=False, mqtt_broker="test.mosquitto.org"): # Should be altered to not hardcode the values but instead use default values in the object init
        """
            Initialises a Separator object.

            Parameters:
                mqtt_connect (bool): Determines whether or not the Separator object will connect to MQTT. False by default.

                mqtt_broker (str): The IP address for the MQTT broker the object will connect to. 'test.mosquitto.org' by default.
        """
        if not isinstance(mqtt_connect, bool):
            raise ValueError("mqtt_connect must be of type bool")
        if not isinstance(mqtt_broker, str):
            raise ValueError("mqtt_broker must be of type string")
 
        self.dt = 0.1

        self.total_volume = 1000 # m cubed
        self.weir_height = 0.5*self.total_volume 
        
        self.input_rate = 0

        self.oil_fraction = 0.3 # 0.05
        self.gas_fraction = 0.2 # 0.06
        self.wat_fraction = 0.5 # 0.89

        self.oil_outflow_max = 4
        self.oil_outflow = 0
        self.gas_outflow_max = 200
        self.gas_outflow = 0
        self.wat_outflow_max = 4
        self.wat_outflow = 0

        self.pressure = 0
        self.gas_mol = 0

        self.wat_level = 0.05*self.total_volume
        self.wat_min = 0.20*self.total_volume
        self.wat_max = 0.45*self.total_volume

        self.oil_level = 0.0
        self.oil_min = 0.20*self.total_volume
        self.oil_max = 0.45*self.total_volume

        self.total_level = self._totalLevelUpdate()

        self.product_level = 0.05*self.total_volume

        wat_valve_speed = 10
        oil_valve_speed = 2.5
        gas_valve_speed = 1

        self.wat_valve = IsolatingValve(self.wat_max, self.wat_min, self.wat_outflow_max, speed=wat_valve_speed)
        self.oil_valve = IsolatingValve(self.oil_max, self.oil_min, self.oil_outflow_max, speed=oil_valve_speed)
        self.gas_valve = ModulatingValve(self.gas_outflow_max, setpoint=15000, dt=self.dt, speed=gas_valve_speed)

        self.mqtt_json = ""
        if mqtt_connect:
            self.mqtt_active = True
        else: 
            self.mqtt_active = False
        
        if self.mqtt_active:
            try:
                self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
                self.mqtt_client.on_connect = on_connect
                self.mqtt_client.on_publish = on_publish
                
                self.mqtt_client.connect(mqtt_broker, port=1883, keepalive=60)
                self.mqtt_client.loop_start()
                
                # Wait for connection, integrated timeout
                connection_wait_time = 0
                while not self.mqtt_client.is_connected() and connection_wait_time < 5: # 5 second connection timeout
                    logging.info("Waiting for broker connection...")
                    sleep(1)
                    connection_wait_time += 1

                if self.mqtt_client.is_connected():
                    logging.info("Connected to broker")
                    self.publish_topic = "lmrtk/Oil_Separator/JSON_Data"
                else:
                    logging.error("Failed to connect to broker after timeout")
                    self.mqtt_active = False # Disable MQTT functionality if connection fails
            except ConnectionRefusedError:
                logging.error(f"Failed to connect to broker: Refused by broker at {mqtt_broker}")
                self.mqtt_active = False # Disable MQTT functionality if connection fails
            except TimeoutError:
                logging.error(f"Failed to connect to broker: Connection timed out to broker at {mqtt_broker}")
                self.mqtt_active = False # Disable MQTT functionality if connection fails
            except Exception as e:
                logging.error(f"Unexpected error during MQTT connection: {e}")
                self.mqtt_active = False # Disable MQTT functionality if connection fails

    def _convertDataToMQTTJSON(self, noisy=False):
        """
            Converts specific variable data into a JSON string in preparation for sending to MQTT. 
            Only has to send a single message this way to allow the digital twin to unpack and display.

            Parameters:
                noisy (bool): Determines whether the method displays the JSON string in the terminal. False by default.

            Returns:
                str: JSON string of various data values.
        """
        data_to_send = {
            "inlet_flow" : self.input_rate,
            "oil_fraction" : self.oil_fraction,
            "water_fraction" : self.wat_fraction,
            "gas_fraction" : self.gas_fraction,
            "product_level" : self.product_level,
            "interface_level" : self.wat_level,
            "top_level" : self.total_level,
            "gas_amount" : self.gas_mol,
            "pressure" : self.pressure,
            "oil_valve_pos" : self.oil_valve.position,
            "oil_valve_flow" : self.oil_outflow,
            "water_valve_pos" : self.wat_valve.position,
            "water_valve_flow" : self.wat_outflow,
            "gas_valve_position" : self.gas_valve.position,
            "gas_valve_flow" : self.gas_outflow
        }
        self.mqtt_json = json.dumps(data_to_send)
        if noisy:
            print(self.mqtt_json)
        return self.mqtt_json
    
    def updateMQTTData(self):
        """
            Calls the convertDataToMQTTJSON method to update the JSON string then publishes the message to MQTT if active.
        """
        self._convertDataToMQTTJSON()
        if self.mqtt_active:
            message = self.mqtt_client.publish(self.publish_topic, self.mqtt_json)

    def _totalLevelUpdate(self):
        """
            Updates the total level attribute by combining the levels of water and oil.

            Returns:
                float: The new total level.
        """
        self.total_level = roundTwoDP(self.wat_level + self.oil_level)
        return self.total_level

    def _pressureUpdate(self):
        """
            Updates the pressure value using the molar gas as well as the volume remaining in the tank. pV=nRT.

            Returns:
                float: The new pressure.
        """
        V = self.total_volume - self.total_level*WEIR_POSITION - self.product_level*(1-WEIR_POSITION) # Assumes weir is 75% across length
        n = self.gas_mol
        P = (n*R*T)/V
        self.pressure = roundTwoDP(ONE_ATM+P) # 1 atm plus additional
        return self.pressure

    def _handleInput_AUTO(self):
        """
            Adjusts the levels of each input component depending on the proportion and input rate of the object. 
            Updates the total level of the tank as well.
        """
        wat_amt = self.input_rate*self.wat_fraction
        oil_amt = self.input_rate*self.oil_fraction
        gas_amt = self.input_rate*self.gas_fraction

        self.wat_level = roundTwoDP(self.wat_level+wat_amt)
        self.oil_level = roundTwoDP(self.oil_level+oil_amt)
        self.gas_mol = self.gas_mol+(gas_amt/MOLAR_VOLUME_STP)

        self._totalLevelUpdate()

    def handleInput_MANUAL(self, input_amt: float | int):
        """
            Adjusts the levels of each input component depending on the proportion and input rate given as a parameter. 
            Updates the total level of the tank as well.

            Parameters:
                input_amt (int/float): The amount of input mixture to inject to the tank.
        """
        wat_amt = input_amt*self.wat_fraction
        oil_amt = input_amt*self.oil_fraction
        gas_amt = input_amt*self.wat_fraction

        self.wat_level = roundTwoDP(self.wat_level+wat_amt)
        self.oil_level = roundTwoDP(self.oil_level+oil_amt)
        self.gas_mol = self.gas_mol+(gas_amt/MOLAR_VOLUME_STP)

        self._totalLevelUpdate()

    def _handleWeirOverflow(self):
        """
            Checks and handles the oil overflowing the weir and into the product section.
            Updates each level affected - oil, product, and total.
        """
        if self.total_level > self.weir_height:
            difference = self.total_level - self.weir_height
            if (self.oil_level - OVERFLOW_FACTOR*difference) > 0:
                self.oil_level = roundTwoDP(self.oil_level-(OVERFLOW_FACTOR*difference))
                self.product_level = roundTwoDP(self.product_level+(OVERFLOW_FACTOR*difference))
            else:
                self.product_level = roundTwoDP(self.product_level+self.oil_level)
                self.oil_level = 0
            self._totalLevelUpdate()
        else:
            pass # Top level is below weir 

    def _updateProductValve(self):
        """
            Handles the control and movement of the oil dump valve and the output flow rate.
        """
        self.oil_valve.updateCommand(self.product_level)
        self.oil_valve.updatePosition()
        self.oil_outflow = self.oil_valve.returnFlow()
        self.product_level = roundTwoDP(self.product_level-self.oil_outflow)
        if self.product_level < 0:
            self.product_level = 0
        
    def _updateWaterValve(self):
        """
            Handles the control and movement of the water dump valve and the output flow rate.
        """
        self.wat_valve.updateCommand(self.wat_level)
        self.wat_valve.updatePosition()
        self.wat_outflow = self.wat_valve.returnFlow()
        self.wat_level = roundTwoDP(self.wat_level-self.wat_outflow)
        if self.wat_level < 0:
            self.wat_level = 0
        self._totalLevelUpdate()

    def _updateGasValve(self, noisy=False):
        """
            Handles the control and movement of the gas control valve and the output flow rate.

            Parameters:
                noisy (bool): Determines whether status messages are sent to the terminal. False by default.
        """
        self.gas_valve.updateController(self.pressure, noisy=noisy)
        self.gas_valve.updatePosition()
        self.gas_outflow = self.gas_valve.returnFlow()
        self.gas_mol = roundTwoDP(self.gas_mol-self.gas_outflow)
        if self.gas_mol < 0:
            self.gas_mol = 0

    def runSimulationFrame_AUTO(self, noisy=False):
        """
            Runs an entire system simulation for one time frame. Uses the automatic input handling.

            Parameters: 
                noisy (bool): Determines whether status messages are sent to the terminal. False by default.
        """
        self._handleInput_AUTO()
        self._pressureUpdate()
        self._handleWeirOverflow()
        self._updateWaterValve()
        self._updateGasValve(noisy=noisy)
        self._updateProductValve()

        if self.mqtt_active:
            self.updateMQTTData()

        if noisy:
            logging.info(f"Inlet Flow: {self.input_rate}")
            logging.info(f"Gas: {self.gas_mol}")
            logging.info(f"Pressure: {self.pressure}")
            logging.info(f"Water: {self.wat_level}")
            logging.info(f"Oil: {self.oil_level}")
            logging.info(f"Total: {self.total_level}")
            logging.info(f"Product: {self.product_level}")

        sleep(self.dt)

    def resetParameters(self, new_input_rate=2.0, noisy=False):
        if noisy:
            logging.warning("Resetting all simulation values...")
        self.pressure = 0
        self.gas_mol = 0
        self.wat_level = 0.05*self.total_volume
        self.product_level = 0.05*self.total_volume
        self.oil_level = 0.0
        self.total_level = self._totalLevelUpdate()
        self.input_rate = new_input_rate
        self.gas_valve.position = 0
        self.wat_valve.position = 0
        self.oil_valve.position = 0


if __name__ == "__main__":
    example_choice = 0
    while example_choice == 0:
        try:
            example_choice = int(input("[?] Number of code example to run (see code): "))
        except:
            logging.warning("value entered needs to be integer (see code)")
        
    match example_choice:
        case 1: # Runs a simulation for 2 minutes and stores various data points. Plots graphs at the end 
            logging.info("Starting example 1...")
            import matplotlib.pyplot as plt

            test_separator = Separator(mqtt_connect=False, mqtt_broker="192.168.1.3")
            test_separator.input_rate = 2
            # test_separator.gas_valve.PID_data["setpoint"] = 20000

            oils = []
            waters = []
            gases = []
            top_levels = []
            pressures = []
            products = []
            counts = []
            valve_positions = []

            counter = 0
            logging.info("Beginning sim, completion in 120 seconds...")
            while counter < 1200:
                test_separator.runSimulationFrame_AUTO()

                counts.append(counter)
                oils.append(test_separator.oil_level)
                waters.append(test_separator.wat_level)
                gases.append(test_separator.gas_mol)
                top_levels.append(test_separator.total_level)
                pressures.append(test_separator.pressure)
                products.append(test_separator.product_level)
                valve_positions.append(test_separator.gas_valve.position)

                if counter == 600:
                    test_separator.gas_valve.PID_data["setpoint"] = 20000
                    # test_separator.input_rate = 1

                if counter % 10 == 0:
                    test_separator.updateMQTTData()

                counter += 1
            
            logging.info("sim finished")
            logging.info("plotting data...")

            plt.subplot(1,2,1)
            plt.plot(counts, pressures, label="Pressure", linestyle="-.", color="ForestGreen")
            plt.legend()

            plt.subplot(1,2,2)
            plt.axline((0,500), slope=0, color="Red")
            plt.plot(counts, waters, label="Water Level", linestyle="-", color="Blue")
            plt.plot(counts, top_levels, label="Oil Level", linestyle="-", color="Black")
            plt.plot(counts, products, label="Product Level", linestyle="-", color="Indigo")
            plt.grid()
            plt.legend()

            plt.show()

            logging.info("exiting...")

        case 2: # Continuous simulation, running until the program is stopped. Plots a live graph of some data while running
            import matplotlib.pyplot as plt
            import msvcrt
            import random
            # print(f"[>] {datetime.now()} Starting example 2, press 'q' to quit.")
            logging.info("Starting example 2, press 'q' to quit.")

            test_separator = Separator(mqtt_connect=False, mqtt_broker="192.168.1.3")
            test_separator.input_rate = 2
            test_separator.gas_valve.PID_data["setpoint"] = 15000

            counter = 0

            waters = []
            tops = []
            products = []
            pressures = []

            plt.ion()
            fig, ax = plt.subplots()
            line1, = ax.plot([], [], label="Water Level")
            line2, = ax.plot([], [], label="Total Level")
            line3, = ax.plot([], [], label="Product Level")
            line4, = ax.plot([], [], label="Pressure (Pa)")
            weir_height = ax.axhline(y=500, color='r', linestyle='--', label="Weir Height")
            ax.set_xlim(0, 1000)
            ax.set_ylim(0, 2000)
            ax.set_xlabel("Time Steps")
            ax.set_ylabel("Levels")
            ax.legend()

            while True:
                try:
                    if msvcrt.kbhit():
                        if msvcrt.getch().decode() == 'q':
                            plt.close()
                            if test_separator.mqtt_active:
                                test_separator.mqtt_client.loop_stop()
                                test_separator.mqtt_client.disconnect()
                            logging.info("User initiated exit...")
                            break
                        elif msvcrt.getch().decode() == 'r':
                            logging.info("Resetting simulation parameters and plot...")
                            test_separator.resetParameters()
                            counter = 0

                            waters.clear()
                            tops.clear()
                            products.clear()
                            pressures.clear()

                            line1.set_data([], [])
                            line2.set_data([], [])
                            line3.set_data([], [])
                            line4.set_data([], [])
                            ax.set_xlim(0, 1000)
                            fig.canvas.draw()
                            fig.canvas.flush_events()
                            continue

                    
                    if counter % 10 == 0:
                        test_separator.input_rate = random.uniform(1, 3)

                    test_separator.runSimulationFrame_AUTO()

                    waters.append(test_separator.wat_level)
                    tops.append(test_separator.total_level)
                    products.append(test_separator.product_level)
                    pressures.append(test_separator.pressure/10)

                    if len(waters) > 1000:
                        waters.pop(0)
                    if len(tops) > 1000:
                        tops.pop(0)
                    if len(products) > 1000:
                        products.pop(0)
                    if len(pressures) > 1000:
                        pressures.pop(0)

                    line1.set_data(range(len(waters)), waters)
                    line2.set_data(range(len(tops)), tops)
                    line3.set_data(range(len(products)), products)
                    line4.set_data(range(len(pressures)), pressures)
                    ax.set_xlim(0, max(len(waters), len(tops)))
                    plt.pause(0.01)

                    if counter % 1 == 0:
                        test_separator.updateMQTTData()

                        if test_separator.mqtt_active:
                            # Publish commands to actuators on flow rig
                            test_separator.mqtt_client.publish("Actuator/IQT/Demand_Position", test_separator.oil_valve.position*100)
                            test_separator.mqtt_client.publish("Actuator/CVA/Demand_Position", test_separator.gas_valve.position*100)
                    else:
                        pass

                    counter += 1
                except KeyboardInterrupt:
                    plt.close()
                    if test_separator.mqtt_active:
                        test_separator.mqtt_client.loop_stop()
                        test_separator.mqtt_client.disconnect()
                    logging.warning(f"User interrupted program, exiting...")
                    break

        case 0:
            logging.info("User chose to exit, exiting...")

        case _:
            logging.warning("Invalid input, exiting...")