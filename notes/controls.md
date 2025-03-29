# Controller Bindings

## Driver

| Button | State | Action |
| ------ | ----- | ------ |
| Left Trigger | IDLE | Begin coral intake sequence -> CORAL_PREINTAKE |
| Left Bumper | CORAL_READY | Align with a left handed branch, -> CORAL_PRESCORE |
| Right Bumper | CORAL_READY | Align with a right handed branch -> CORAL_PRESCORE |
| Left Trigger | CORAL_PRESCORE | Raise to currently set coral target -> CORAL_CONFIRM_[LEVEL] |
| Right Trigger | CORAL_CONFIRM_[LEVEL] | Score coral on confirmed level -> IDLE |
| Either Bumper | IDLE | Align with algae on selected reef -> ALGAE_INTAKE_[LEVEL] |
| Left Trigger | ALGAE_INTAKE_[LEVEL] | Intake algae with current request -> ALGAE_READY |
| Either Bumper | ALGAE_READY | Align with net -> ALGAE_PRESCORE_AN  |
| Right Trigger | ALGAE_CONFIRM_[LEVEL] | Score algae on confirmed level -> IDLE |
| A | IDLE | Begin climb -> CLIMB_PREPULL |
| B | READY_ALGAE | ALGAE_OUTTAKE -> IDLE |
| X | READY_CORAL | CORAL_OUTTAKE -> IDLE |
| Y | any | Zero gyro |
| Right Trigger | CLIMB_PREPULL | Climb -> CLIMB_PULL |
| Right Bumper | CLIMB_PULL | Cancel climb -> CLIMB_PREPULL |

## Operator

| Button | Action |
| ------ | ------ |
| A | Set target coral level to L1, set algae target to processor  |
| X | Set target coral level to L2 |
| B | Set target coral level to L3 |
| Y | Set target coral level to L4, set algae target to net |
| Right Trigger + Left Joystick | Enable manual elevator control |
