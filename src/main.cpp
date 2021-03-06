#include <Arduino.h>
#include <QTRSensors.h>

#define _pinM1I 4
#define _pinM1D 5
#define _pinM1PWM 3

#define _pinM2I 7
#define _pinM2D 8
#define _pinM2PWM 6

#define _pinBuzz 9
#define _stdby 2
#define _pinbutton 11
#define _pinemitter 10

#define _calibsound 440
#define _curvsound 1000

int _maxspeed = 155;
int _minspeed = 70;

#define ethreshold 200

unsigned long srate = 100;

int _sM1I = 0;
int _sM1D = 1;
int _sM2I = 1;
int _sM2D = 0;

float Kp = 0.01;
float Kd = 0.1;
float Ki = 0.01;

int position = 0;
int error = 0;
float lasterror = 0;
float derror = 0;
int ierror = 0;
int ithreshold = 100;
int leftspeed = 0;
int rightspeed = 0;
float sign = 0;

unsigned long tactual = 0;

unsigned long debouncetime = 0;
unsigned long lastdebounceleft = 0;
unsigned long lastdebounceright = 0;

uint16_t rightmarkervalue = 0;
uint16_t lastrightmarkervalue = 0;
uint16_t leftmarkervalue = 0;
uint16_t lastleftmarkervalue = 0;

boolean _incurvemarker = false;
boolean _incurve = false;
boolean _inracemarker = true;
int rmarkercount = 0;
boolean _readonceright = false;
boolean _readonceleft = false;

QTRSensors _front;
const uint8_t frontsensors = 6;
uint16_t frontsensorValues[frontsensors];

QTRSensors _left;
const uint8_t leftsensors = 1;
uint16_t leftsensorValues[leftsensors];
QTRSensors _right;
const uint8_t rightsensors = 1;
uint16_t rightsensorValues[rightsensors];

void pidout()
{
  error = 2500 - position;
  derror = float((lasterror - error)) / srate;
  if (abs(error) < ithreshold)
  {
    ierror = ierror + error;
  }
  else
  {
    ierror = 0;
  }
  sign = Kp * error + Ki * ierror + Kd * derror;
  lasterror=error;
  leftspeed = leftspeed - sign;
  rightspeed = rightspeed + sign;

  if (_incurve)
  {
    _maxspeed = 80;
  }
  else
  {
    _maxspeed = 155;
  }

  leftspeed = constrain(leftspeed, _minspeed, _maxspeed);
  rightspeed = constrain(rightspeed, _minspeed, _maxspeed);
  Serial.print(leftspeed);
  Serial.print("     ");
  Serial.println(rightspeed);

}

void move()
{
  digitalWrite(_pinM1D, _sM1D);
  digitalWrite(_pinM1I, _sM1I);
  analogWrite(_pinM1PWM, leftspeed);
  digitalWrite(_pinM2D, _sM2D);
  digitalWrite(_pinM2I, _sM2I);
  analogWrite(_pinM2PWM, rightspeed);
}

void end()
{
  if (_inracemarker == 1 && _readonceright == false && _incurvemarker == false)
  {
    _readonceright = true;
  }
  if (_inracemarker == 0 && _readonceright == true)
  {
    rmarkercount = rmarkercount + 1;
    _readonceright = false;
  }
  if (rmarkercount == 2)
  {

    digitalWrite(_stdby, LOW);
  }
}

boolean debounce(int value, int lastvalue, unsigned long lastdebounce, boolean state)
{

  if (value != lastvalue)
  {
    lastdebounce = millis();
  }
  if ((millis() - lastdebounce) > debouncetime)
  {

    if (value != state)
    {
      state = value;
    }
  }
  lastvalue = value;
  return state;
}

int makedigital(int value)
{
  if (value < 500)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void turning()
{
  if (_incurvemarker == 1 && _readonceleft == false && _inracemarker == false)
  {
    _readonceleft = true;
  }
  if (_incurvemarker == 0 && _readonceleft == true)
  {
    _incurve = !_incurve;
    _readonceleft = false;
  }
}

void sensefront()
{
  position = _front.readLineWhite(frontsensorValues);
}

void sensesides()
{

  _right.readLineBlack(rightsensorValues);
  rightmarkervalue = makedigital(rightsensorValues[0]);

  if (rightmarkervalue != lastrightmarkervalue)
  {
    lastdebounceright = millis();
  }
  if ((millis() - lastdebounceright) > debouncetime)
  {

    if (rightmarkervalue != _inracemarker)
    {
      _inracemarker = rightmarkervalue;
    }
  }
  lastrightmarkervalue = rightmarkervalue;

  _left.readLineBlack(leftsensorValues);
  leftmarkervalue = makedigital(leftsensorValues[0]);

  if (leftmarkervalue != lastleftmarkervalue)
  {
    lastdebounceleft = millis();
  }
  if ((millis() - lastdebounceleft) > debouncetime)
  {

    if (leftmarkervalue != _incurvemarker)
    {
      _incurvemarker = leftmarkervalue;
    }
  }
  lastleftmarkervalue = leftmarkervalue;
}

void setupSensors()
{

  tone(_pinBuzz, _calibsound);
  delay(1000);
  noTone(_pinBuzz);

  _right.setTypeAnalog();
  _right.setSensorPins((const uint8_t[]){A7}, rightsensors);
  _left.setTypeAnalog();
  _left.setSensorPins((const uint8_t[]){A6}, leftsensors);
  _front.setTypeAnalog();
  _front.setSensorPins((const uint8_t[]) {A0,A1,A2,A3,A4,A5}, frontsensors);

  for (int i = 0; i < 200; i++)
  {
    _front.calibrate();
    _right.calibrate();
    _left.calibrate();
  }

  tone(_pinBuzz, _calibsound);
  delay(500);
  noTone(_pinBuzz);
  delay(500);
  tone(_pinBuzz, _calibsound);
  delay(500);
  noTone(_pinBuzz);
}

void setupWheels()
{
  pinMode(_pinM1I, OUTPUT);
  pinMode(_pinM1D, OUTPUT);
  pinMode(_pinM1PWM, OUTPUT);
  pinMode(_pinM2I, OUTPUT);
  pinMode(_pinM2D, OUTPUT);
  pinMode(_pinM2PWM, OUTPUT);
  pinMode(_pinBuzz, OUTPUT);
  pinMode(_stdby, OUTPUT);
  digitalWrite(_stdby, HIGH);
  digitalWrite(_pinM1D,_sM1D);
  digitalWrite(_pinM1I,_sM1I);
  digitalWrite(_pinM2D,_sM2D);
  digitalWrite(_pinM2I, _sM2I);
}

void waitbutton()
{
  while (digitalRead(_pinbutton) == false)
  {
  }
  tone(_pinBuzz, 2000);
  delay(3000);
}

void setup()
{
  Serial.begin(9600);
  setupWheels();
  setupSensors();
  pinMode(_pinBuzz, OUTPUT);
  pinMode(_pinbutton, OUTPUT);
  //waitbutton();
}

void loop()
{
  if (millis() - tactual >= srate)
  {
    sensesides();
    turning();
    end();
    sensefront();
    pidout();
    move();
    tactual = millis();
  }
}
