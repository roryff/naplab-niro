# One sensor entry from UBX-ESF-STATUS
uint8  type          # raw sensor type ID
string type_name     # human-readable name (e.g. "Gyro Z", "Wheel Speed")
bool   used          # sensor is used in fusion
bool   ready         # sensor is ready
string calib         # calibration state: not-calibrated | calibrating | calibrated
string time_tag      # time-tag type: no-data | byte-tag | event-tag | provided
uint8  freq          # measurement frequency [Hz]
string faults        # space-separated fault flags, or "none"
