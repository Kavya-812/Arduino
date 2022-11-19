#define FL_MOTOR 3
#define FR_MOTOR 9
#define BR_MOTOR 10
#define BL_MOTOR 11

double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput; 

//Define the aggressive and conservative Tuning Parameters<br>double consKp = 1, consKi = 0.05, consKd = 0.25;
PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);

void setup() {

  pitchInput = 0.0;
  rollInput = 0.0;  
  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;
  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);

  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
  } 

  pinMode(FL_MOTOR, OUTPUT);
  pinMode(FR_MOTOR, OUTPUT);
  pinMode(BR_MOTOR, OUTPUT);
  pinMode(BL_MOTOR, OUTPUT);

void loop() {
  pitchPID.Compute();
  rollPID.Compute();
  int actSpeed[4];
  stabilise (targetSpeed, actSpeed, rollOutput, pitchOutput);
  //    targetSpeed = actSpeed; // should this be here or not
}
void stabilise (int* currSpeed, int* actSpeed, float rollDiff, float pitchDiff) {
  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  actSpeed[0] = (int) currSpeed[0] + (rollDiff / 2) - (pitchDiff / 2);
  actSpeed[1] = (int) currSpeed[1] + (rollDiff / 2) + (pitchDiff / 2);
  actSpeed[2] = (int) currSpeed[2] - (rollDiff / 2) + (pitchDiff / 2);
  actSpeed[3] = (int) currSpeed[3] - (rollDiff / 2) - (pitchDiff / 2);
  for (int i = 0; i < 4; i ++) {
    if (actSpeed[i] < 0) actSpeed[i] = 0;  
  }
}
void runIndividual (int* actSpeed) {
  analogWrite(FL_MOTOR, actSpeed[0]);
  analogWrite(FR_MOTOR, actSpeed[1]);
  analogWrite(BR_MOTOR, actSpeed[2]);
  analogWrite(BL_MOTOR, actSpeed[3]);
}
