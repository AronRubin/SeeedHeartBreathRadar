#ifndef _SEEED_HEART_BREATH_RADAR_H__
#define _SEEED_HEART_BREATH_RADAR_H__

#include <SoftwareSerial.h>
#include <utility>

namespace stdmissing {
template<typename T1, typename T2>
struct pair {
  T1 first;
  T2 second;
  constexpr pair() = default;
  constexpr pair( const T1 &x, const T2 &y ) : first( x ), second( y ) {}
  template< class U1, class U2 >
  constexpr pair( const pair<U1, U2> &p ) : first( p.first ), second( p.second ) {}
  template< class U1, class U2 >
  constexpr pair( pair<U1, U2> &&p ) : first( std::forward<U1>(p.first) ), second( std::forward<U2>( p.second ) ) {}
  pair( const pair &other ) = default;
  pair( pair &&other ) = default;
};

} // namespace stdmissing

class SeeedHeartBreathRadar {
private:
public:
/*
----------------+----+--------+---------------------------------
Field Name      |    | bytes  | Description
----------------+----+--------+---------------------------------
1 Frame header  | FH | 2      | Always bytes 0x53 0x59 ("SY")
----------------+----+--------+---------------------------------
2 Control Topic | CD | 1      | See control topics
----------------+----+--------+---------------------------------
3 Operation     | OD | 1      | Operation type (listed for each)
----------------+----+--------+---------------------------------
4 Data Length   | L1 | 2      | Length of DA in bytes
----------------+----+--------+---------------------------------
5 Data          | DA | 0-2048 | payload data
----------------+----+--------+---------------------------------
6 Checksum      | CH | 1      | Checksum
----------------+----+--------+---------------------------------
7 End of frame  | FT | 2      | Always bytes 0x54 0x43 ("TC")
----------------+----+--------+---------------------------------
*/
  struct Frame {
    uint8_t head[2];
    uint8_t topic;
    uint8_t op;
    uint16_t length; // big endian in network
    uint8_t payloadbuf1[32]; // the first bytes of the payload
    uint8_t checksum;
    uint8_t foot[2];
  };

  // Command Table
  enum ControlTopic : uint8_t {
    CD_HEARTBEAT_ID     = 0x01, // heartbeat pack identification;
    CD_PRODUCT_INFO     = 0x02, // product information;
    CD_OTA_UPDRADE      = 0x03, // OTA (over the air) upgrade;
    CD_RADAR_TEST       = 0x04, // radar test;
    CD_OPERATING_STATUS = 0x05, // operating status;
    CD_LOCATION_INFO    = 0x06, // radar location information;
    CD_LOC_DET_ANOMAL   = 0x07,
    CD_HUMAN_PRESENCE   = 0x80, // human presence;
    CD_RESPIRATION_INFO = 0x81,  // respiratory heartbeat.
    CD_FALL_DETECTION   = 0x83, // fall detection
    CD_SLEEP_INFO       = 0x84,  // sleep state
    CD_HEARTBEAT_INFO   = 0x85  // heart state.
  };

  static const char *topicToString( ControlTopic topic ) {
    switch (topic) {
    case CD_HEARTBEAT_ID: return "heartbeat pack identification";
    case CD_PRODUCT_INFO: return "product information";
    case CD_OTA_UPDRADE: return "OTA (over the air) upgrade";
    case CD_RADAR_TEST: return "radar test";
    case CD_OPERATING_STATUS: return "operating status";
    case CD_LOCATION_INFO: return "radar location information";
    case CD_LOC_DET_ANOMAL: return "location detection abnormality";
    case CD_HUMAN_PRESENCE: return "human presence";
    case CD_RESPIRATION_INFO: return "respiration information";
    case CD_FALL_DETECTION: return "fall detection";
    case CD_SLEEP_INFO: return "sleep state";
    case CD_HEARTBEAT_INFO: return "heartbeat information";
    default: return "unknown";
    }
  }

  enum TransmissionBytes : uint8_t {
    HEAD_0 = 0x53,  // first byte of frame header
    HEAD_1 = 0x59,  // second byte of frame header
    FOOT_0 = 0x54,  // first byte of frame footer
    FOOT_1 = 0x43   // second byte of frame footer
  };

  enum ProductInfoOpertion {
    OD_GET_PRODUCT_MODEL  = 0x01,
    OD_SET_PRODUCT_MODEL  = 0x02,
    OD_GET_PRODUCT_ID     = 0x03,
    OD_SET_PRODUCT_ID     = 0x04,
    OD_GET_HARDWARE_MODEL = 0x05,
    OD_SET_HARDWARE_MODEL = 0x06,
    OD_GET_FIRMWARE_INFO  = 0x07,
    OD_SET_FIRMWARE_INFO  = 0x08,
    OD_GET_PROTOCOL_INFO  = 0x09,
    OD_SET_PROTOCOL_INFO  = 0x0a
  };

  static const char *productInfoOperationToString( ProductInfoOpertion op ) {
    switch ( op ) {
    case OD_GET_PRODUCT_MODEL: return "get product model";
    case OD_SET_PRODUCT_MODEL: return "set product model";
    case OD_GET_PRODUCT_ID: return "get product id";
    case OD_SET_PRODUCT_ID: return "set product id";
    case OD_GET_HARDWARE_MODEL: return "get hardware model";
    case OD_SET_HARDWARE_MODEL: return "set hardware model";
    case OD_GET_FIRMWARE_INFO: return "get firmware info";
    case OD_SET_FIRMWARE_INFO: return "set firmware info";
    case OD_GET_PROTOCOL_INFO: return "get protocol info";
    case OD_SET_PROTOCOL_INFO: return "set protocol info";
    default: return "unknown";
    }
  }

  enum OperatingStatusOperation {
    OD_SET_OPERATING_STATUS  = 0x01,
    OD_GET_OPERATING_STATUS  = 0x02,
    OD_GET_WORKING_HOURS = 0x03,
  };
  
  static const char *operatingStatusOperationToString( OperatingStatusOperation op ) {
    switch ( op ) {
    case OD_SET_OPERATING_STATUS: return "set operating status";
    case OD_GET_OPERATING_STATUS: return "get operating status";
    case OD_GET_WORKING_HOURS: return "get working hours";
    default: return "unknown";
    }
  }
  
  enum LocationInfoOperation {
    OD_GET_FOV  = 0x09
  };


  enum PersonInfoOperation : uint8_t {
    OD_GET_PRESENCE_INF   = 0x01,  // Presence Information
    OD_GET_MOVEMENT_STATE = 0x02,  // Campaign Information
    OD_GET_MOVEMENT_LEVEL = 0x03,  // Body movement information
    OD_GET_DISTANCE       = 0x04,  // Distance from the person being detected
    OD_GET_ANGLE          = 0x05   // Body orientation
  };

  static const char *personInfoOperationToString( PersonInfoOperation op ) {
    switch (op) {
    case OD_GET_PRESENCE_INF: return "get presence Information";
    case OD_GET_MOVEMENT_STATE: return "get movement state";
    case OD_GET_MOVEMENT_LEVEL: return "get movement level";
    case OD_GET_DISTANCE: return "get distance to person";
    case OD_GET_ANGLE: return "get angle to person";
    default: return "unknown";
    }
  }

  enum MovementVal : uint8_t {
    MV_NONE       = 0x00,        //None
    MV_STATIONARY = 0x01,  //A person is stationary
    MV_MOVEMENT   = 0x02    //A person in motion
  };

  static const char *movementValToString( MovementVal val ) {
    switch ( val) {
    case MV_NONE: return "none";
    case MV_STATIONARY: return "stationary";
    case MV_MOVEMENT: return "movement";
    default: return "unknown";
    }
  }

  enum BreathOperations : uint8_t {
    OD_GET_BREATH_STATE = 0x01, // heart rate and classification
    OD_GET_BREATH_RATE  = 0x02, // respiratory rate and classification
    OD_GET_BREATH_INTENSITY = 0x03,  //Breathing intensity
    OD_GET_BREATH_CONFIDENCE = 0x04,  //Confidence
    OD_GET_BREATH_WAVE = 0x05  //Respiratory waveform (No analysis for now)
  };

  static const char *breathOperationsToString( BreathOperations op ) {
    switch (op) {
    case OD_GET_BREATH_STATE: return "get breathing state";
    case OD_GET_BREATH_RATE: return "get breathing rate";
    case OD_GET_BREATH_INTENSITY: return "get breathing intensity";
    case OD_GET_BREATH_CONFIDENCE: return "get breath rate confidence";
    case OD_GET_BREATH_WAVE: return "get respiratory waveform"; // (No analysis for now)
    default: return "unknown";
    }
  };

  enum HeartOperations : uint8_t {
    OD_GET_HEART_STATE = 0x01, // heart rate and classification
    OD_GET_HEART_RATE  = 0x02, // respiratory rate and classification
  };

  static const char *heartOperationsToString( HeartOperations op ) {
    switch (op) {
    case OD_GET_HEART_STATE: return "get heart state";
    case OD_GET_HEART_RATE: return "get heart rate";
    default: return "unknown";
    }
  };

  enum VitalsStateVal : uint8_t {
    VS_UNKNOWN   = 0,
    VS_NORMAL    = 0x01,     //Normal breathing
    VS_HIGH      = 0x02,      //Acute respiratory abnormalities
    VS_LOW       = 0x03,       //Slow heartbeat
    VS_DETECTING = 0x04  //Radar detection in progress
  };

  static const char *vitalsStateValToString( VitalsStateVal state ) {
    switch (state) {
    case VS_NORMAL: return "normal";
    case VS_HIGH: return "high";
    case VS_LOW: return "low";
    case VS_DETECTING: return "detecting";
    default: return "unknown";
    }
  }
  
  enum FallDetectionOperation : uint8_t {
    OD_FALL_DETECTION_SWITCH       = 0x00, // 0 = set, 1 = on, 2 = off
    OD_FALL_STATE                  = 0x01, // 0 = not, 1 = falling
    OD_FALL_CONFIDENCE             = 0x02, // val
    OD_FALL_LOCATION               = 0x03, // uint16[2] (x, y)
    OD_FALL_POINT_CLOUD            = 0x04, // uint16[3] (x, y ,z)
    OD_FALL_STATIONARY             = 0x05, // bool
    OD_FALL_SET_PC_FREQUENCY       = 0x09, // 0 = set, 0 - 100 Hz
    OD_FALL_DWELL_TIME             = 0x0A, // uint32 (seconds)
    OD_FALL_DWELL_SWITCH           = 0x0B, // bool
    OD_FALL_STATE_SWITCH_QUERY     = 0x80, // 0 = query, 1 = on, 2 = off
    OD_FALL_STATE_QUERY            = 0x81, // 0 = query, 0 = not, 1 = falling
    OD_FALL_CONFIDENCE_QUERY       = 0x82, // 0 = query, val
    OD_FALL_LOCATION_QUERY         = 0x83, // 0 = query, uint16[2] (x, y)
    OD_FALL_POINT_CLOUD_QUERY      = 0x84, // 0 = query, uint16[3] (x, y, z)

  };

  static const char *fallDetectionOperationToString( FallDetectionOperation op ) {
    switch (op) {
    case OD_FALL_DETECTION_SWITCH: return "fall detection switch";
    case OD_FALL_STATE: return "fall state";
    case OD_FALL_CONFIDENCE: return "fall confidence";
    case OD_FALL_LOCATION: return "fall location";
    case OD_FALL_POINT_CLOUD: return "point cloud";
    case OD_FALL_STATIONARY: return "fall stationary";
    case OD_FALL_SET_PC_FREQUENCY: return "point cloud frequency";
    case OD_FALL_DWELL_TIME: return "fall dwell time";
    case OD_FALL_DWELL_SWITCH: return "fall dell switch";
    case OD_FALL_STATE_SWITCH_QUERY: return "fall state switch query";
    case OD_FALL_STATE_QUERY: return "fall state query";
    case OD_FALL_CONFIDENCE_QUERY: return "fall confidence query";
    case OD_FALL_LOCATION_QUERY: return "fall location query";
    case OD_FALL_POINT_CLOUD_QUERY: return "point cloud query";
    default: return "unknown";
    }
  }
  
  ~SeeedHeartBreathRadar();

  bool recvRadarBytes();

  bool begin( HardwareSerial *serial ) {
    if (!serial) {
      serial = &Serial1;
    }
    m_serial = (Stream *)serial;
    serial->begin( 115200 );
    return (bool)*serial;
  }

  bool begin( SoftwareSerial *serial ) {
    if (!serial) {
      return false;
    }
    m_serial = (Stream *)serial;
    serial->begin( 115200 );
    return (bool)*serial;
  }

  unsigned getHeartRate() const {
    return m_hr;
  }

  unsigned getRespiratoryRate() const {
    return m_rr;
  }

  VitalsStateVal getHeartRateState() const {
    return m_hr_state;
  }

  VitalsStateVal getRespiratoryState() const {
    return m_rr_state;
  }

  unsigned getDistance() const {
    return m_distance;
  }

  stdmissing::pair<unsigned, unsigned> getAngles() const {
    return { m_angles.first, m_angles.second };
  }

  unsigned getMovementLevel() const {
    return m_movementLevel;
  }

  MovementVal getMovementState() const {
    return m_movementState;
  }

  unsigned getDebugLevel() const {
    return m_debugLevel;
  }

  void setDebugLevel( unsigned level ) {
    m_debugLevel = level;
  }
  
  bool sendFrame( const Frame &frame );
  bool requestProductInfo( ProductInfoOpertion op );
  bool requestOperatingStatus();

protected:
  void resetFrame();
  bool handleHeartbeatIdFrame();
  bool handleProductInfoFrame();
  bool handleOtaUpgradeFrame();
  bool handleRadarTestFrame();
  bool handleOperatingStatusFrame();
  bool handleLocationInfoFrame();
  bool handlePersonInfoFrame();
  bool handleRespirationInfoFrame();
  bool handleHeartInfoFrame();
  bool handlePersonLocationAnomalyFrame();
  bool handleSleepInfoFrame();
  bool handleFrame();

  static constexpr uint8_t calcCrc( uint8_t current, uint8_t next );
  static constexpr uint8_t calcCrc( uint8_t current, const uint8_t *next, size_t nextLen );

  unsigned m_debugLevel = 1;
  Stream *m_serial = nullptr;
  size_t m_framePos = 0;
  uint8_t m_runningSum = 0;
  uint8_t m_calcChecksum = 0;
  Frame m_frame;
  uint8_t *m_extPayloadBuf = nullptr;
  unsigned m_hr = 0;
  VitalsStateVal m_hr_state = VS_UNKNOWN;
  unsigned m_rr = 0;
  VitalsStateVal m_rr_state = VS_UNKNOWN;
  unsigned m_distance = 0;
  bool m_outOfRange = false;
  stdmissing::pair<unsigned, unsigned> m_angles;

  unsigned m_movementLevel = 0;
  MovementVal m_movementState = MV_NONE;

};

#endif
