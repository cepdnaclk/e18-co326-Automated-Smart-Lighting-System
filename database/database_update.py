import mysql.connector
import paho.mqtt.client as mqtt
import datetime
import json

# MQTT callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker")
    client.subscribe("UoP/CO/326/E18/18/BH1750")
    client.subscribe("UoP/CO/326/E18/18/PIR")

def on_message(client, userdata, msg):
    print("Received message: " + str(msg.payload.decode()))
    # Parse and process the MQTT message here
    # Extract the data from the message and store it in variables

    # Insert data into MySQL database based on the topic
    if msg.topic == "UoP/CO/326/E18/18/BH1750":
        # if data received from broker
        data = json.loads(str(msg.payload.decode("utf-8")))
        
        # calculate rtc time
        rtc_time = datetime.datetime.now().replace(microsecond=0) - datetime.datetime.strptime(data["DateTime"], '%Y-%m-%d %H:%M:%S:%f')

        # Process and insert data for topic 1
        data_to_insert = (data["Intensity"], data["DateTime"], datetime.datetime.now().replace(microsecond=0), rtc_time)
        
        db_cursor.execute(sql_query1, data_to_insert)
        
    elif msg.topic == "UoP/CO/326/E18/18/PIR":
        # if data received from broker
        data = json.loads(str(msg.payload.decode("utf-8")))
        
        # calculate rtc time
        rtc_time = datetime.datetime.now().replace(microsecond=0) - datetime.datetime.strptime(data["DateTime"], '%Y-%m-%d %H:%M:%S:%f')

        # Process and insert data for topic 2
        data_to_insert = (data["Occupancy"], data["DateTime"], datetime.datetime.now().replace(microsecond=0), rtc_time)
        
        db_cursor.execute(sql_query2, data_to_insert)

    db_connection.commit()
    print("Data inserted into MySQL database")

# Set up MQTT client
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# Connect to MQTT broker
mqtt_client.connect("test.mosquitto.org", 1883, 60)

# Set up MySQL connection
db_connection = mysql.connector.connect(
    host='localhost',
    user='root',
    password='',
    database='smart_lighting_system'
)

# Define database queries
db_cursor = db_connection.cursor()
sql_query1 = "INSERT INTO light_intensity VALUES(uuid(), %s, %s, %s, %s)"
sql_query2 = "INSERT INTO occupancy VALUES(uuid(), %s, %s, %s, %s)"

mqtt_client.loop_start()  # Start the MQTT client loop

# Keep the script running
while True:
    pass
