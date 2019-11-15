/*****************************************************************************/
/*!
* Vehicle Location Tracking
* Using the Adafruit Feather 32u4 FONA with the Ultimate GPS FeatherWing.
*
* @product Vehicle Location Tracking
* @version 0.1.0
* @copyright 2019 - MimoCAD, Inc.
* @author Mark 'Dygear' Tomlin, CEO MimoCAD, Inc.
******************************************************************************/

/**
 * FONA Hardware Setup
 */
#include <Adafruit_FONA.h>

// Define the correct GPIO Pins for RX / TX and Reset
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

// Serial Interface
#include <SoftwareSerial.h>
// Raw Software Serial Interface
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *FONASerial = &fonaSS;

// Define the name of the Software Serial Port
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

/**
 * GPS Hardware Setup
 */
#include <Adafruit_GPS.h>

// Define the name of the Hardware Serial Port
#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

/**
 * @global struct VecLocFrame
 */
struct VecLocFrame {
    char        IMEI[15];           // FONA Hardware IMEI
    struct      GPS {
        uint8_t Fix;                // TRUE If GPS Has Fix
        uint8_t Quality;            // Fix Quality
        char    Satellites;         // Number of Satellites
        int32_t Latitude;           // Latitude / 10,000,000 For Degrees
        int32_t Longitude;          // Longitude / 10,000,000 For Degrees
        float   Knots;              // Speed Over Ground In Knots
        float   Angle;              // Course in Degrees from True North; 0 North; 90 East; 180 South; 270 West;
        float   Altitude;           // Altitude in Meters Above MSL
    } GPS;
    uint16_t    Battery;            // Battery Level In mV.
    uint16_t    Signal;             // RSSI
} VecLocFrame;
struct VecLocFrame Frame;

/**
 * Setup Entry Function
 * 
 * @return void
 */
void setup() {
    /**
     * Serial Interface Initialization
     */
    // Wait for the Hardware Serial to Initiate

    // Set Serial Baud Rate
    Serial.begin(115200);
    // Set GPS Baud Rate
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);

    FONASerial->begin(4800);
    if (!fona.begin(*FONASerial)) {
        Serial.println(F("Couldn't find FONA"));
        while (1);
    }

    fona.getIMEI(Frame.IMEI);
    fona.setGPRSNetworkSettings(F("wholesale"), F(""), F(""));

    while(!fona.getNetworkStatus()); // Wait for Network to be available
}

/**
 * Main Program Loop
 * 
 * @return void
 */
void loop() {
    GPS.read();

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) {
            return;
        }
    }

    // Wrap Around Timer Correction
    static uint32_t timer = millis();
    if (timer > millis()) {
        timer = millis();
    }

    // Every 10 seconds print and send out the current status.
    static unsigned nextInterval = 10000;
    if (millis() - timer > nextInterval) {
        timer = millis();

        /**
         * Populate Dynamic Data In VecLoc Frame
         */
        Frame.GPS.Fix = GPS.fix;
        Frame.GPS.Quality = GPS.fixquality;
        Frame.GPS.Satellites = GPS.satellites;
        Frame.GPS.Latitude = GPS.latitude_fixed;
        Frame.GPS.Longitude = GPS.longitude_fixed;
        Frame.GPS.Knots = GPS.speed;
        Frame.GPS.Angle = GPS.angle;
        Frame.GPS.Altitude = GPS.altitude;
        fona.getBattVoltage(&Frame.Battery);
        Frame.Signal = fona.getRSSI();

        static bool GPRSConnection = false;
        if (GPRSConnection) {
            static char TCP_SERVER[] = "mimocad.io";
            static uint16_t TCP_PORT = 31337;

            static bool TCPConnection = false;
            if (!TCPConnection) {
                // Reconnect if Disconnected
                if (TCPConnection = fona.TCPconnect(TCP_SERVER, TCP_PORT)) {
                    Serial.println(F("TCP Connection Established"));
                } else {
                    Serial.println(F("TCP Connection Failed"));
                }
            } else {
                // Convert the VecLoc Frame into Char Array.
                static char FrameBuf[sizeof VecLocFrame] = {0};
                memcpy(FrameBuf, &Frame, sizeof VecLocFrame);

                if (!fona.TCPsend((char *) FrameBuf, sizeof VecLocFrame)) {
                    TCPConnection = false; // Close the Connection
                }
            }
        } else {
            if (GPRSConnection = fona.enableGPRS(true)) {
                Serial.println(F("GPRS Connection Established"));
            } else {
                Serial.println(F("GPRS Connection Failed"));
            }
        }
    }
}
