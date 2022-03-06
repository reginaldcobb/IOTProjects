#define wifi_ssid "PBellAire1"
#define wifi_password "miltonflorida1"

//
// IO Zoom MQTT Broker
#define mqtt_server "198.55.125.23"
#define mqtt_user "mqtt"
#define mqtt_password "n21902"


//
//Raspberry Pi MQTT Broker
// #define mqtt_server "192.168.0.223"
// #define mqtt_user "mosquitto"
// #define mqtt_password "n219022"

#define temperatureF_topic "sensor/temperatureF"  // Temp F MQTT Topic
#define temperatureC_topic "sensor/temperatureC"  // Temp F MQTT Topic
#define humidity_topic "sensor/humidity"  //Humidity MQTT Topic
#define motion_topic "sensor/motion"  //?? MQTT Topic
#define loopcount_topic "sensor/loopcount"  //?? MQTT Topic
#define ir_topic "sensor/ir"  //?? MQTT Topic
#define full_topic "sensor/full"  //?? MQTT Topic
#define visible_topic "sensor/visible"  //?? MQTT Topic
#define lux_topic "sensor/lux"  //?? MQTT Topic
#define sound_topic "sensor/sound"  //?? MQTT Topic
#define ambient_topic "sensor/ambient"  //?? MQTT Topic
#define delta_topic "sensor/delta"  //?? MQTT Topic
#define maxsound_topic "sensor/maxsound"  //?? MQTT Topic
#define minsound_topic "sensor/minsound"  //?? MQTT Topic
//#define humidity_topic "sensor/??"  //?? MQTT Topic
#define unix_date_topic "sensor/unixdate"  // Unix Date MQTT Topic
#define doc_topic "sensor/doc"  // Unix Date MQTT Topic

// char message_buff[100];