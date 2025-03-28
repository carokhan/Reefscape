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
| Either Bumper | ALGAE_READY | Align with net |
| Right Trigger | ALGAE_CONFIRM_[LEVEL] | Score algae on confirmed level, return to IDLE |
| A | any | Reverse hopper |
| B | READY_ALGAE | SPIT_ALGAE |
| X | READY_CORAL | CORAL_OUTTAKE |
| Y | any | Zero gyro |

| Right Trigger | CLIMB_PREPULL | Climb |
| Left Trigger | CLIMB_PULL | Cancel Climb |

## Operator

| Button | Action |
| ------ | ------ |
| A | Set target coral level to L1, set algae target to processor  |
| X | Set target coral level to L2 |
| B | Set target coral level to L3 |
| Y | Set target coral level to L4, set algae target to net |
