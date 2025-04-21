// MotorDriver.cpp
#include "MotorDriver.h"

//==============================================================================
// Constructor and Initialization
//==============================================================================

MotorDriver::MotorDriver(uint8_t nSleepPin, uint8_t drvoffPin, uint8_t pwmPin, uint8_t fgoutPin,
                         uint8_t nFaultPin, uint8_t ilimPin, uint8_t csPin, uint8_t brakePin) {
  // Store pin assignments
  _nSleepPin = nSleepPin;
  _drvoffPin = drvoffPin;
  _pwmPin = pwmPin;
  _fgoutPin = fgoutPin;
  _nFaultPin = nFaultPin;
  _ilimPin = ilimPin;
  _csPin = csPin;
  _brakePin = brakePin;

  // SPI Setup
  _spiSpeed = 1000000;

  // Initialize state variables
  _motorRunning = false;
  _currentSpeed = 0;
  _currentAcc = 0;
  _currentDirection = false;  // CW
  _maxSpeed = 511;
  _minSpeed = -511;

  // Initialize speed measurement variables
  _lastFgPulseTime = 0;
  _fgPulsePeriod = 0;
  _pulseCount = 0;
  _currentRPM = 0;
  _rawRPM = 0;
  _lastValidPulsePeriod = 0;
  _pulseTimeout = 300000;  // 300ms
  _maxPulseDeviation = 0.1;
  _pulsesPerRevolution = 4;
  _motorPolePairs = 4;
  _lastDirChange = 0;
  _lastDir = false;

  // Initialize PID variables
  _kp = 0.05;     // Reduced from 0.1
  _ki = 0.0005;   // Much lower for stability
  _kd = 0.01;     // Added derivative for damping
  _kp_low = 0.02; // Lower gains for low speed
  _ki_low = 0.0001;
  _kd_low = 0.005;
  _targetRPM = 0;
  _targetTorque = 0;
  _integral = 0;
  _lastError = 0;
  _pidEnabled = false;
  _lastPIDUpdate = 0;
  _torqueMode = true;
  _inZeroRegion = false;
  _lastOutput = 0;

  // Filter variables initialization
  _filtersInitialized = false;
  _historyIndex = 0;
  _notchQ = 10;
  _lowPassQ = 0.75;
  _lowPassCutoff = .05;
  _lowPassAlpha = 0.05;
  _freqMultiplier = 1.0;
  _secondaryFreqMultiplier = 2.0;
  
  for (int i = 0; i < 5; i++) {
    _lastFilteredValues[i] = 0;
    _recentValues[i] = 0;
  }

  // Initialize Savitzky-Golay filter buffers
  for (int i = 0; i < SG_WINDOW_SIZE; i++) {
    _sgBuffer[i] = 0;
  }
  _sgBufferIndex = 0;
  _sgBufferFilled = false;
}

void MotorDriver::begin() {
  // Initialize pins
  pinMode(_nSleepPin, OUTPUT);
  pinMode(_drvoffPin, OUTPUT);
  pinMode(_nFaultPin, INPUT_PULLUP);
  pinMode(_fgoutPin, INPUT_PULLUP);
  pinMode(_ilimPin, OUTPUT);
  pinMode(_csPin, OUTPUT);
  pinMode(_brakePin, OUTPUT);

  // Set initial pin states
  digitalWrite(_nSleepPin, HIGH);  // Enable the driver
  digitalWrite(_drvoffPin, LOW);   // Enable the driver output
  digitalWrite(_csPin, HIGH);      // Set CW direction
  digitalWrite(_brakePin, LOW);    // Disable Brake

  // Set current limit to mid-range
  analogWrite(_ilimPin, 127);

  // Initialize SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);  // Using SPI Mode 1 based on diagnostics
  SPI.setBitOrder(MSBFIRST);

  // Reset the driver
  resetDriver();
  initializeDriver();
  setupFilters();
  setupPWM();
  setupBrakePWM();
}

void MotorDriver::printAllRegisters() {
  Serial.println("\n----- All Register Values -----");

  for (int reg = 0; reg <= 0x0C; reg++) {
    SpiResponse resp = readRegisterFull(reg);

    Serial.print("REG 0x");
    if (reg < 16) Serial.print("0");
    Serial.print(reg, HEX);
    Serial.print(": 0x");
    if (resp.data < 16) Serial.print("0");
    Serial.print(resp.data, HEX);
    Serial.print(" (Status: 0x");
    if (resp.status < 16) Serial.print("0");
    Serial.print(resp.status, HEX);
    Serial.println(")");
  }

  Serial.println("------------------------------");
}

void MotorDriver::setupPWM(uint32_t frequency) {
  // This method should implement the detailed PWM setup from your original code
  // for the Arduino MKR Zero / SAMD21

  // Step 1: Configure GCLK4 for 48MHz
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Step 2: Connect GCLK4 to TCC0
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Step 3: Enable pin multiplexing for the PWM pin
  PORT->Group[g_APinDescription[_pwmPin].ulPort].PINCFG[g_APinDescription[_pwmPin].ulPin].bit.PMUXEN = 1;

  // Determine if pin is odd or even
  uint8_t pin_num = g_APinDescription[_pwmPin].ulPin;
  if (pin_num % 2 == 0) {
    // Even pin - PMUXE
    PORT->Group[g_APinDescription[_pwmPin].ulPort].PMUX[pin_num >> 1].reg &= ~PORT_PMUX_PMUXE_Msk;
    PORT->Group[g_APinDescription[_pwmPin].ulPort].PMUX[pin_num >> 1].reg |= PORT_PMUX_PMUXE_F;
  } else {
    // Odd pin - PMUXO
    PORT->Group[g_APinDescription[_pwmPin].ulPort].PMUX[pin_num >> 1].reg &= ~PORT_PMUX_PMUXO_Msk;
    PORT->Group[g_APinDescription[_pwmPin].ulPort].PMUX[pin_num >> 1].reg |= PORT_PMUX_PMUXO_F;
  }

  // Step 4: Disable TCC0 before configuration
  TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ;

  // Step 5: Configure waveform generation
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;  // Normal PWM mode
  while (TCC0->SYNCBUSY.bit.WAVE)
    ;

  // Step 6: Configure output matrix to connect CC0 to WO4
  TCC0->WEXCTRL.reg = TCC_WEXCTRL_OTMX(2);  // Output matrix setting

  // Step 7: Set the period value (frequency)
  // Calculate PER value based on frequency
  uint32_t per_value = 512;  // Default value

  TCC0->PER.reg = per_value;
  while (TCC0->SYNCBUSY.bit.PER)
    ;

  // Step 8: Set initial duty cycle
  TCC0->CC[0].reg = 0;  // Start with 0% duty cycle
  while (TCC0->SYNCBUSY.bit.CC0)
    ;

  // Step 9: Enable TCC0 with prescaler
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ;
}

void MotorDriver::setupFilters(float notchQ, float lowPassAlpha,
                               float freqMultiplier, float secondaryFreqMultiplier) {
  // Store filter parameters
  _notchQ = notchQ;
  _lowPassAlpha = lowPassAlpha;
  _freqMultiplier = freqMultiplier;
  _secondaryFreqMultiplier = secondaryFreqMultiplier;

  // Create the filters
  _primaryNotchFilter = new Biquad(bq_type_notch, 0.1, _notchQ, 0);
  _secondaryNotchFilter = new Biquad(bq_type_notch, 0.2, _notchQ, 0);
  _lowPassFilter = new Biquad(bq_type_lowpass, _lowPassCutoff, _lowPassQ, 0);

  // Initialize history buffers
  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    _rpmHistory[i] = 0;
  }

  for (int i = 0; i < 5; i++) {
    _lastFilteredValues[i] = 0;
  }

  _filtersInitialized = true;
}

//==============================================================================
// Core Motor Control Methods
//==============================================================================

void MotorDriver::start() {
  // Check for faults before starting
  if (!digitalRead(_nFaultPin)) {
    return;  // Can't start if there's a fault
  }

  // ADDED: Reset all RPM measurement variables to prevent initial spikes
  _lastFgPulseTime = 0;
  _currentRPM = 0;
  _rawRPM = 0;
  _lastValidPulsePeriod = 0;
  _pulseCount = 0;
  _inZeroRegion = true;  // Start in zero region
  _lastOutput = 0;
  
  // If currently at zero speed, set to a minimum starting speed
  if (_currentSpeed == 0) {
    _currentSpeed = 50;  // Minimum starting speed
  }

  if (_torqueMode) { _targetRPM = _currentRPM; }

  // Apply PWM signal gradually to prevent current spikes
  // ADDED: Ramp up PWM to prevent initial jolts
  int finalSpeed = _currentSpeed;
  for (int i = 0; i < finalSpeed; i += 20) {
    TCC0->CCB[0].reg = i;
    while (TCC0->SYNCBUSY.bit.CC0);
    delay(1);
  }
  TCC0->CCB[0].reg = finalSpeed;
  while (TCC0->SYNCBUSY.bit.CC0);

  // Make sure DRVOFF is LOW (motor enabled)
  digitalWrite(_drvoffPin, LOW);

  _motorRunning = true;
}

void MotorDriver::stop() {
  // Set PWM to zero
  TCC0->CCB[0].reg = 0;
  while (TCC0->SYNCBUSY.bit.CC0)
    ;
  _motorRunning = false;
}

float MotorDriver::getRawRPM() {
  return _rawRPM;
}

// Add these method implementations to MotorDriver.cpp

bool MotorDriver::isTorqueMode() const {
  return _torqueMode;
}

void MotorDriver::setTorqueMode(bool enable) {
  _torqueMode = enable;
}

float MotorDriver::getTargetTorque() const {
  return _targetTorque;
}

float MotorDriver::getTargetRPM() const {
  return _targetRPM;
}

float MotorDriver::getCurrentTorque() const {
  // Calculate torque from acceleration using T = I * α
  // Convert RPM/s to rad/s² (RPM/s * 2π/60)
  float accelerationRadPerSec2 = _currentAcc * (2.0 * PI / 60.0);
  return _motorInertia * accelerationRadPerSec2;
}

void MotorDriver::setMotorInertia(float inertia) {
  _motorInertia = inertia;
}

float MotorDriver::getMotorInertia() const {
  return _motorInertia;
}


void MotorDriver::setupBrakePWM() {
  // Step 1: Enable pin multiplexing for the brake pin
  PORT->Group[g_APinDescription[_brakePin].ulPort].PINCFG[g_APinDescription[_brakePin].ulPin].bit.PMUXEN = 1;

  // Determine if pin is odd or even
  uint8_t pin_num = g_APinDescription[_brakePin].ulPin;
  if (pin_num % 2 == 0) {
    // Even pin - PMUXE
    PORT->Group[g_APinDescription[_brakePin].ulPort].PMUX[pin_num >> 1].reg &= ~PORT_PMUX_PMUXE_Msk;
    PORT->Group[g_APinDescription[_brakePin].ulPort].PMUX[pin_num >> 1].reg |= PORT_PMUX_PMUXE_E;
  } else {
    // Odd pin - PMUXO
    PORT->Group[g_APinDescription[_brakePin].ulPort].PMUX[pin_num >> 1].reg &= ~PORT_PMUX_PMUXO_Msk;
    PORT->Group[g_APinDescription[_brakePin].ulPort].PMUX[pin_num >> 1].reg |= PORT_PMUX_PMUXO_E;
  }

  // Connect GCLK4 to TCC1
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Disable TCC1 before configuration
  TCC1->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (TCC1->SYNCBUSY.bit.ENABLE)
    ;

  // Configure waveform generation
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;  // Normal PWM mode
  while (TCC1->SYNCBUSY.bit.WAVE)
    ;

  // Use fixed period value of 512
  TCC1->PER.reg = 512;
  while (TCC1->SYNCBUSY.bit.PER)
    ;

  // Set initial duty cycle to 0
  TCC1->CC[0].reg = 0;
  while (TCC1->SYNCBUSY.bit.CC0)
    ;

  // Enable TCC1 with no prescaler for highest frequency
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC1->SYNCBUSY.bit.ENABLE)
    ;
}

void MotorDriver::adjustSpeed(int increment) {
  int newSpeed = _currentSpeed + increment;
  setSpeed(newSpeed);
}

void MotorDriver::setDirection(bool directionCCW) {
  _currentDirection = directionCCW;
  writeRegister(CONTROL_REG_7, _currentDirection ? 0x01 : 0x00);
}

void MotorDriver::toggleDirection() {
  if (!_motorRunning) {
    _currentDirection = !_currentDirection;
    writeRegister(CONTROL_REG_7, _currentDirection ? 0x01 : 0x00);
  }
}

void MotorDriver::brake() {
  // First stop the PWM
  TCC0->CCB[0].reg = 0;
  while (TCC0->SYNCBUSY.bit.CC0)
    ;
  _motorRunning = false;

  TCC1->CCB[0].reg = 511;
  while (TCC1->SYNCBUSY.bit.CC0)
    ;
}

void MotorDriver::releaseBrake() {
  TCC1->CCB[0].reg = 0;
  while (TCC1->SYNCBUSY.bit.CC0)
    ;
}

//==============================================================================
// Status Methods
//==============================================================================

bool MotorDriver::isRunning() const {
  return _motorRunning;
}

float MotorDriver::getCurrentRPM() const {
  return _currentRPM;
}

int MotorDriver::getCurrentSpeed() const {
  return _currentSpeed;
}

bool MotorDriver::getDirection() const {
  return _currentDirection;
}

bool MotorDriver::checkFaults() {
  // Check fault pin
  bool faultPinState = digitalRead(_nFaultPin);

  // Return true if fault detected
  return !faultPinState;
}

// Clear faults
void MotorDriver::clearFaults() {
  Serial.println("Clearing faults...");

  // Set the CLR_FLT bit in CONTROL_REG_2A
  // Preserve PWM_MODE bits and other settings
  uint8_t currentValue = readRegisterFull(CONTROL_REG_2A).data;
  writeRegister(CONTROL_REG_2A, currentValue | 0x01);

  // Wait for fault clear to process
  delay(5);

  // Check if faults were cleared
  SpiResponse icStatus = readRegisterFull(IC_STATUS_REG);
  if (!(icStatus.data & 0x01) && digitalRead(_nFaultPin)) {
    Serial.println("All faults cleared");
  } else {
    Serial.println("Some faults still present - check status");
  }
}

//==============================================================================
// Configuration Methods
//==============================================================================

void MotorDriver::setCurrentLimit(uint8_t limit) {
  analogWrite(_ilimPin, limit);
}

float MotorDriver::getCurrentLimit() {
  return analogRead(_ilimPin);
}

//==============================================================================
// PID Control Methods
//==============================================================================

void MotorDriver::setPIDParameters(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  
  // Also set region-specific PID parameters
  _kp_low = kp * 0.4f;  // Lower proportional gain at low speeds
  _ki_low = ki * 0.2f;  // Lower integral at low speeds
  _kd_low = kd * 0.5f;  // Stronger derivative at low speeds for damping
}

float MotorDriver::setTargetRPM(float targetRPM) {
  _targetRPM = targetRPM;
  return _currentSpeed;
}

float MotorDriver::setTargetTorque(float targetTorque) {
  _targetTorque = targetTorque;
  return _currentSpeed;
}

void MotorDriver::enablePID(bool enable) {
  if (enable && !_pidEnabled) {
    // Reset PID variables when enabling
    _integral = 0;
    _lastError = 0;
    _lastPIDUpdate = micros();
    _lastOutput = _currentSpeed; // Initialize last output for rate limiting
  }
  _pidEnabled = enable;
}

// Savitzky-Golay filter implementation
float MotorDriver::savitzkyGolayFilter(float newValue) {
  // Add new value to circular buffer
  _sgBuffer[_sgBufferIndex] = newValue;
  _sgBufferIndex = (_sgBufferIndex + 1) % SG_WINDOW_SIZE;

  // Check if buffer is filled
  if (!_sgBufferFilled && _sgBufferIndex == 0) {
    _sgBufferFilled = true;
  }

  // If buffer isn't filled yet, return input value
  if (!_sgBufferFilled) {
    return newValue;
  }

  // Apply 7-point quadratic Savitzky-Golay filter
  // These coefficients are pre-calculated for a quadratic fit
  static const float coeffs[] = { -2, 3, 6, 7, 6, 3, -2 };
  static const float norm = 21.0;  // Normalization factor

  float result = 0;
  for (int i = 0; i < SG_WINDOW_SIZE; i++) {
    int idx = (_sgBufferIndex + i) % SG_WINDOW_SIZE;
    result += coeffs[i] * _sgBuffer[idx];
  }

  return result / norm;
}

// Calculate derivative using Savitzky-Golay
float MotorDriver::savitzkyGolayDerivative() {
  // If buffer isn't filled yet, use simple difference
  if (!_sgBufferFilled) {
    return _currentAcc;
  }

  // Apply 7-point Savitzky-Golay first derivative coefficients
  // These coefficients are pre-calculated for first derivative
  static const float coeffs[] = { -3, -2, -1, 0, 1, 2, 3 };
  static const float norm = 28.0;  // Normalization factor

  float derivative = 0;
  for (int i = 0; i < SG_WINDOW_SIZE; i++) {
    int idx = (_sgBufferIndex + i) % SG_WINDOW_SIZE;
    derivative += coeffs[i] * _sgBuffer[idx];
  }

  // Scale by effective sampling rate (samples per second)
  unsigned long currentTime = micros();
  static unsigned long lastTime = currentTime;
  float deltaT = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Assuming consistent sampling, convert to RPM/s
  float samplesPerSecond = 1.0 / deltaT;
  return (derivative / norm) * samplesPerSecond;
}

// Public method to get filtered acceleration
float MotorDriver::getFilteredAcceleration() {
  return savitzkyGolayDerivative();
}

void MotorDriver::setSpeed(int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, _minSpeed, _maxSpeed);
  
  // Don't change anything if motor isn't running
  if (!_motorRunning) {
    _currentSpeed = speed;
    return;
  }
  
  // Check for direction change
  bool wantForward = speed > 0;
  bool wantReverse = speed < 0;
  bool currentForward = _currentSpeed >= 0;
  bool currentReverse = _currentSpeed < 0;
  bool directionChange = (wantForward && currentReverse) || (wantReverse && currentForward);
  
  // If very small power requested, just stop
  if (abs(speed) < 20) {
    TCC0->CCB[0].reg = 0;
    while (TCC0->SYNCBUSY.bit.CC0);
    _currentSpeed = 0;
    return;
  }
  
  // Direction change - use simpler approach with consistent timing
  if (directionChange) {
    // Stop motor first
    TCC0->CCB[0].reg = 0;
    while (TCC0->SYNCBUSY.bit.CC0);
    
    // Brief brake to ensure full stop
    TCC1->CCB[0].reg = 200;
    while (TCC1->SYNCBUSY.bit.CC0);
    delay(50); // Fixed delay - predictable timing
    
    // Release brake
    TCC1->CCB[0].reg = 0;
    while (TCC1->SYNCBUSY.bit.CC0);
    
    // Change direction register
    bool newDirection = wantReverse;
    writeRegister(CONTROL_REG_7, newDirection ? 0x01 : 0x00);
    _currentDirection = newDirection;
    
    // Small delay to let direction change take effect
    delay(20);
    
    // Apply consistent minimum power
    int minPower = 50;
    TCC0->CCB[0].reg = minPower;
    while (TCC0->SYNCBUSY.bit.CC0);
    
    // Short delay before ramping up
    delay(30);
    
    // Now ramp up in small steps
    for (int i = minPower; i < abs(speed) && i < 200; i += 10) {
      TCC0->CCB[0].reg = i;
      while (TCC0->SYNCBUSY.bit.CC0);
      delay(5);
    }
    
    // Final setting
    TCC0->CCB[0].reg = abs(speed);
    while (TCC0->SYNCBUSY.bit.CC0);
  }
  // No direction change - apply direct or with minimal ramping
  else {
    // Apply with minimal ramping if large change
    int currentAbsSpeed = abs(_currentSpeed);
    int targetAbsSpeed = abs(speed);
    
    if (abs(targetAbsSpeed - currentAbsSpeed) > 50) {
      int step = (targetAbsSpeed > currentAbsSpeed) ? 10 : -10;
      for (int i = currentAbsSpeed; 
           (step > 0) ? (i < targetAbsSpeed) : (i > targetAbsSpeed); 
           i += step) {
        TCC0->CCB[0].reg = i;
        while (TCC0->SYNCBUSY.bit.CC0);
        delay(2); // Fast ramp but not instant
      }
    }
    
    // Final setting
    TCC0->CCB[0].reg = targetAbsSpeed;
    while (TCC0->SYNCBUSY.bit.CC0);
  }
  
  _currentSpeed = speed;
}

void MotorDriver::updatePID() {
  unsigned long currentTime = micros();
  float deltaT = (currentTime - _lastPIDUpdate) / 1000000.0;
  
  if (!_motorRunning || deltaT <= 0 || !_pidEnabled) {
    return;
  }
  
  _lastPIDUpdate = currentTime;
  
  // Update RPM and acceleration
  _lastRPM = _currentRPM;
  _currentRPM = applyFilters(_rawRPM);
  _currentAcc = (_currentRPM - _lastRPM) / deltaT;
  
  // Hysteresis control for zero crossing region
  static const float ZERO_BAND = 50.0f;  // RPM
  static const float HYSTERESIS = 100.0f; // RPM
  
  // Check if we're entering or leaving the zero region
  if (!_inZeroRegion && abs(_currentRPM) < ZERO_BAND) {
    _inZeroRegion = true;
    // Reset integral term when entering zero region
    _integral = 0;
  }
  else if (_inZeroRegion && abs(_currentRPM) > (ZERO_BAND + HYSTERESIS)) {
    _inZeroRegion = false;
  }
  
  // Update torque-based target if in torque mode
  if (_torqueMode && deltaT < 0.01) {
    if (abs(_targetRPM + _targetTorque * deltaT) < 15000) {
      _targetRPM = _targetRPM + _targetTorque * deltaT;
    }
  }
  
  // Special control in zero region
  if (_inZeroRegion) {
    // Handle very small target RPM as a stop command
    if (abs(_targetRPM) < 10.0f) {
      setSpeed(0);
      return;
    }
    
    // For zero crossing, use a simpler control approach
    int direction = _targetRPM >= 0 ? 1 : -1;
    
    // Calculate simple proportional control with minimum power
    int minPower = 40; // Minimum power needed to overcome static friction
    float zeroError = _targetRPM - _currentRPM;
    
    // Apply deadband around zero
    if (abs(zeroError) < 20.0f) {
      zeroError = 0;
    }
    
    // Simple proportional control with low gain in zero region
    float output = 0.03f * zeroError;
    
    // Apply output with minimum power if needed
    int power = direction * max(minPower, (int)abs(output));
    power = constrain(power, -150, 150); // Limit max power in zero region
    
    // Apply rate limiting
    int maxChange = 20;
    power = constrain(power, _lastOutput - maxChange, _lastOutput + maxChange);
    _lastOutput = power;
    
    setSpeed(power);
    return;
  }
  
  // Regular PID control outside zero region
  float error = _targetRPM - _currentRPM;
  
  // Select PID gains based on speed region
  float kp, ki, kd;
  if (abs(_currentRPM) < 200) { 
    // Low speed region - use gentler gains
    kp = _kp_low;
    ki = _ki_low;
    kd = _kd_low;
  } else {
    // Normal speed region - use standard gains
    kp = _kp;
    ki = _ki;
    kd = _kd;
  }
  
  // Update integral with anti-windup
  _integral += error * deltaT;
  float integralLimit = max(abs(_targetRPM) * 0.3f, 300.0f);
  _integral = constrain(_integral, -integralLimit, integralLimit);
  
  // Calculate derivative with filtering
  float derivative = (error - _lastError) / deltaT;
  static float filteredDerivative = 0;
  filteredDerivative = 0.7f * filteredDerivative + 0.3f * derivative;
  
  // Calculate output
  float output = kp * error + ki * _integral + kd * filteredDerivative;
  
  // Apply rate limiting to prevent command jumps
  float maxChange = 50.0f; // Limit maximum change per cycle
  output = constrain(output, _lastOutput - maxChange, _lastOutput + maxChange);
  _lastOutput = output;
  
  int pwmValue = constrain(output, _minSpeed, _maxSpeed);
  _lastError = error;
  setSpeed(pwmValue);
}

float MotorDriver::applyFilters(float rawRPM) {
  // Two separate filter paths based on speed range
  static float lowSpeedFiltered = 0;
  static float normalSpeedFiltered = 0;
  
  // Determine speed range
  bool isLowSpeed = abs(rawRPM) < 100;
  
  // Apply appropriate filtering based on speed
  if (isLowSpeed) {
    // Very aggressive filtering at low speeds
    lowSpeedFiltered = 0.95f * lowSpeedFiltered + 0.05f * rawRPM;
    
    // Force to zero if very close to zero 
    if (abs(lowSpeedFiltered) < 15.0f) {
      lowSpeedFiltered = 0;
    }
  } else {
    // Less aggressive filtering at normal speeds
    normalSpeedFiltered = 0.8f * normalSpeedFiltered + 0.2f * rawRPM;
  }
  
  // Blend between the two filtered values
  float blendFactor = constrain((abs(rawRPM) - 50) / 50.0f, 0.0f, 1.0f);
  float blendedRPM = (1.0f - blendFactor) * lowSpeedFiltered + blendFactor * normalSpeedFiltered;
  
  // Additional median filtering to reject outliers
  for (int i = 4; i > 0; i--) {
    _recentValues[i] = _recentValues[i-1];
  }
  _recentValues[0] = blendedRPM;
  
  // Simple insertion sort for median
  float sortedValues[5];
  memcpy(sortedValues, _recentValues, sizeof(_recentValues));
  for (int i = 1; i < 5; i++) {
    float key = sortedValues[i];
    int j = i - 1;
    while (j >= 0 && sortedValues[j] > key) {
      sortedValues[j + 1] = sortedValues[j];
      j--;
    }
    sortedValues[j + 1] = key;
  }
  
  // Return median value with additional notch and low-pass filtering
  float medianValue = sortedValues[2];
  
  // Update notch filter parameters if needed
  updateNotchFilters(abs(medianValue) > 50 ? medianValue : _currentRPM);
  
  // Apply multi-stage filtering pipeline
  float stage1 = _primaryNotchFilter->process(medianValue);
  float stage2 = _secondaryNotchFilter->process(stage1);
  float stage3 = movingAverage(stage2);
  float stage4 = _lowPassFilter->process(stage3);
  
  return stage4;
}

//==============================================================================
// Speed Measurement and Filtering
//==============================================================================

void MotorDriver::processFGOUTpulse() {
  unsigned long currentTime = micros();
  unsigned long currentPeriod = 0;

  if (_lastFgPulseTime > 0) {
    currentPeriod = currentTime - _lastFgPulseTime;
    
    // ADDED: Guard against overflow/underflow and unrealistic values
    if (currentPeriod == 0 || currentPeriod > 1000000) {  // Invalid period (>1 second or 0)
      _lastFgPulseTime = currentTime;
      _pulseCount++;
      return;  // Skip this pulse - don't update RPM
    }
    
    if (isValidPulsePeriod(currentPeriod)) {
      // ADDED: Additional protection against division by very small numbers
      if (currentPeriod < 100) {  // Unrealistically short period (would give massive RPM)
        _rawRPM = 0;  // Instead of letting it calculate a huge value
      } else {
        // Calculate RPM with bounds checking
        float calculatedRPM = (60.0 * 1000000.0) / (currentPeriod * _pulsesPerRevolution);
        
        // ADDED: Sanity check on RPM value
        if (calculatedRPM > 30000) {
          calculatedRPM = 0;  // Reset if unrealistic
        }
        
        _rawRPM = calculatedRPM;
      }
      
      // Apply sign based on current direction
      if (_currentDirection) {  // If direction is reverse (true)
        _rawRPM = -_rawRPM;
      }
    }
    
    _lastValidPulsePeriod = currentPeriod;
  }

  _lastFgPulseTime = currentTime;
  _pulseCount++;
}

bool MotorDriver::isValidPulsePeriod(unsigned long period) {
  // Skip validation for the first pulse
  if (_lastValidPulsePeriod == 0) {
    return true;
  }
  if (abs(_currentRPM) < 500) {
    return true;
  }

  // Check if pulse period is too short (possible double-count)
  if (period < _lastValidPulsePeriod * (1.0 - _maxPulseDeviation)) {
    return false;
  }

  // Check if pulse period is too long (possible missed pulse)
  if (period > _lastValidPulsePeriod * (1.0 + _maxPulseDeviation)) {
    // Special case: if it's almost exactly 2x, likely a missed pulse
    return false;
  }

  return true;
}

void MotorDriver::checkPulseTimeout() {
  // Only check when motor is supposed to be running
  if (_motorRunning) {
    unsigned long currentTime = micros();
    // If no pulse received for too long
    if ((currentTime - _lastFgPulseTime) > _pulseTimeout) {
      // Adjust timeout based on current speed (longer timeout at lower speeds)
      _pulseTimeout = max(100000UL, (unsigned long)(60000000.0 / (_currentRPM * _pulsesPerRevolution) * 3));

      // When in timeout and speed is very low, assume motor is stopped
      if (abs(_currentRPM) < 20) {
        _rawRPM = 0;
      }
    }
  }
}

void MotorDriver::updateNotchFilters(float rpm) {
  // Only update if RPM has changed significantly
  if (abs(rpm - _lastFilterUpdateRPM) > 100) {
    // Calculate noise frequency based on RPM
    float rotationFrequency = rpm / 60.0;  // Revolutions per second

    // For this motor with 4 pole pairs and 3 hall sensors,
    // we expect noise at rotation frequency and electrical frequency
    float primaryNoiseFreq = rotationFrequency * _freqMultiplier;
    float secondaryNoiseFreq = rotationFrequency * _secondaryFreqMultiplier;
    float effectiveSampleRate = 1000 * 2;

    // Normalize frequencies (between 0-0.5)
    float primaryNormFreq = primaryNoiseFreq / (effectiveSampleRate / 2);
    float secondaryNormFreq = secondaryNoiseFreq / (effectiveSampleRate / 2);

    // Constrain to valid range
    primaryNormFreq = constrain(primaryNormFreq, 0.05, 0.45);
    secondaryNormFreq = constrain(secondaryNormFreq, 0.05, 0.45);

    // Update filter parameters
    _primaryNotchFilter->setFc(primaryNormFreq);
    _secondaryNotchFilter->setFc(secondaryNormFreq);

    _lastFilterUpdateRPM = rpm;
  }
}

float MotorDriver::movingAverage(float newValue) {
  // Add new value to history
  _rpmHistory[_historyIndex] = newValue;
  _historyIndex = (_historyIndex + 1) % MOVING_AVG_SIZE;

  // Calculate average
  float sum = 0;
  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    sum += _rpmHistory[i];
  }
  return sum / MOVING_AVG_SIZE;
}

float MotorDriver::medianFilter(float newValue) {
  // Shift values
  for (int i = 4; i > 0; i--) {
    _lastFilteredValues[i] = _lastFilteredValues[i - 1];
  }
  _lastFilteredValues[0] = newValue;

  // Copy to temporary array for sorting
  float temp[5];
  for (int i = 0; i < 5; i++) {
    temp[i] = _lastFilteredValues[i];
  }

  // Simple bubble sort
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (temp[j] > temp[j + 1]) {
        float tmp = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = tmp;
      }
    }
  }

  // Return middle value
  return temp[2];
}


void MotorDriver::resetDriver() {
  // Brief pulse on nSLEEP pin for reset
  digitalWrite(_nSleepPin, LOW);
  delayMicroseconds(10);  // 10μs reset pulse
  digitalWrite(_nSleepPin, HIGH);
  delay(10);  // Give time for reset to complete

  // Initialize motor state
  _motorRunning = false;
  _currentSpeed = 0;
  _currentDirection = false;  // CW

  // Reset speed measurement variables
  _pulseCount = 0;
  _lastFgPulseTime = 0;
  _currentRPM = 0;
  _lastValidPulsePeriod = 0;
  
  // Reset control variables
  _integral = 0;
  _lastError = 0;
  _inZeroRegion = true;
  _lastOutput = 0;
}

// Advanced callback methods
void MotorDriver::attachFGCallback(void (*callback)(void)) {
  // This would attach a callback for FGOUT pulses if needed
  // Not implemented in this version
}

void MotorDriver::attachFaultCallback(void (*callback)(void)) {
  // This would attach a callback for fault detection if needed
  // Not implemented in this version
}

// Initialize motor driver with optimal settings
bool MotorDriver::initializeDriver() {
  Serial.println("Initializing MCT8316Z...");

  // Unlock registers
  if (!unlockRegisters()) {
    Serial.println("Failed to unlock registers!");
    return false;
  }

  // Clear any existing faults
  clearFaults();


  // Configure slew rate for clean switching, async PWM
  if (!writeRegister(CONTROL_REG_2A, 0x0A)) {
    Serial.println("Failed to set slew rate!");
    return false;
  }

  // Enable Active Synchronous Rectification (ASR)
  // ASR improves efficiency
  // if (!writeRegister(CONTROL_REG_5, 0x02)) {
  //   Serial.println("Failed to configure rectification mode!");
  //   return false;
  // }

  // Configure OCP (overcurrent protection) for auto-retry
  // OCP_MODE = 0x01 (auto-retry)
  // OCP_LVL = 0x00 (16A limit)
  // OCP_RETRY = 0x00 (5ms retry)
  // OCP_DEG = 0x01 (0.6μs deglitch)
  if (!writeRegister(CONTROL_REG_4, 0x13)) {
    Serial.println("Failed to configure overcurrent protection!");
    return false;
  }

  // Configure Motor Lock detection
  // MTR_LOCK_MODE = 0x01 (auto-retry)
  // MTR_LOCK_TDET = 0x01 (500ms detection time)
  if (!writeRegister(CONTROL_REG_8, 0x45)) {
    Serial.println("Failed to configure motor lock detection!");
    return false;
  }



  // Configure phase advance for better performance
  if (!writeRegister(CONTROL_REG_9, 0x04)) {
    Serial.println("Failed to set phase advance!");
    return false;
  }

  // Configure Buck regulator with 5V output
  writeRegister(CONTROL_REG_6, 0x02);

  return true;

  // setup other stuff
  //writeRegister(CONTROL_REG_3, 0x01); // Current amp gain
}

// Unlock registers
bool MotorDriver::unlockRegisters() {
  Serial.print("Unlocking registers... ");

  // Write the unlock code (0x03) to CONTROL_REG_1
  if (!writeRegister(CONTROL_REG_1, 0x03)) {
    Serial.println("Failed to write to CONTROL_REG_1!");
    return false;
  }

  // Verify unlock was successful
  SpiResponse resp = readRegisterFull(CONTROL_REG_1);
  if ((resp.data & 0x07) == 0x03) {
    Serial.println("Success!");
    return true;
  } else {
    Serial.print("Failed! Register value: 0x");
    Serial.println(resp.data, HEX);
    return false;
  }
}

// Write to a register via SPI
bool MotorDriver::writeRegister(uint8_t reg, uint8_t value) {
  // Format: 0 (write) + 6-bit address + parity + 8-bit data
  uint16_t command = ((reg & 0x3F) << 9) | (value & 0xFF);

  // Calculate and set parity bit
  uint8_t parity = calculateParity(command);
  command |= (parity << 8);

  uint8_t highByte = (command >> 8) & 0xFF;
  uint8_t lowByte = command & 0xFF;

  // SPI transaction
  digitalWrite(_csPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_csPin, LOW);
  delayMicroseconds(5);

  SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE1));
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  SPI.endTransaction();

  digitalWrite(_csPin, HIGH);
  delayMicroseconds(5);

  // Verify write (optional but recommended for reliability)
  SpiResponse resp = readRegisterFull(reg);
  return (resp.data == value);
}

// Read a register via SPI and return both status and data bytes
SpiResponse MotorDriver::readRegisterFull(uint8_t reg) {
  // Format: 1 (read) + 6-bit address + parity + 8 zeros
  uint16_t command = (1 << 15) | ((reg & 0x3F) << 9);

  // Calculate and set parity bit
  uint8_t parity = calculateParity(command);
  command |= (parity << 8);

  uint8_t highByte = (command >> 8) & 0xFF;
  uint8_t lowByte = command & 0xFF;

  SpiResponse resp;

  // SPI transaction
  digitalWrite(_csPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_csPin, LOW);
  delayMicroseconds(5);

  SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE1));
  resp.status = SPI.transfer(highByte);
  resp.data = SPI.transfer(lowByte);
  SPI.endTransaction();

  digitalWrite(_csPin, HIGH);
  delayMicroseconds(5);

  return resp;
}

// Calculate even parity for SPI command
uint8_t MotorDriver::calculateParity(uint16_t command) {
  uint8_t parity = 0;

  for (int i = 0; i < 16; i++) {
    if (i != 8) {  // Skip bit 8 (parity bit position)
      parity ^= (command >> i) & 0x01;
    }
  }

  return parity;
}
