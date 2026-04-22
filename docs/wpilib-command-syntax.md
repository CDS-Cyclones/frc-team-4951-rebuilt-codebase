# WPILib Command Syntax

WPILib command syntax can look weird at first because Java is passing little pieces of behavior around as objects. Most of the strange-looking parts are lambdas, method references, command factories, and trigger bindings.

## The Big Idea

A command is a robot action that the scheduler can run.

Examples:

- run the intake while a button is held
- drive with joysticks
- spin up a shooter
- move an arm to a target
- run an autonomous sequence

Subsystems are the robot parts. Commands tell subsystems what to do.

## What `() ->` Means

`() ->` is a Java lambda. It means "here is a small function."

This:

```java
() -> intake.stop()
```

means "when WPILib asks, run `intake.stop()`."

This:

```java
() -> -controller.getLeftY()
```

means "when WPILib asks, read the controller's left Y value and return the negative of it."

The code after the arrow does not always run immediately. It runs later when the command scheduler or command asks for it.

## Why Commands Use Lambdas

Commands often need live values, not one-time values.

For driving, this is good:

```java
() -> -controller.getLeftY()
```

The joystick value is read every loop while the command is running.

This is usually wrong:

```java
-controller.getLeftY()
```

That reads the joystick once when the command is created. The robot would not keep updating from the joystick.

## Method References

This:

```java
intake::stop
```

is a method reference. It is shorthand for:

```java
() -> intake.stop()
```

Use method references when you are calling a method with no arguments and the shorter version is clear.

## Common Command Factories

WPILib has helper methods in `Commands`.

### `Commands.runOnce`

Runs something once, then finishes.

```java
Commands.runOnce(() -> intake.stop(), intake)
```

Use for one-time actions like resetting a gyro, stopping a motor, or setting a mode.

### `Commands.run`

Runs something every scheduler loop until interrupted or until a condition ends it.

```java
Commands.run(() -> intake.run(0.5), intake)
```

Use for actions that need to keep updating.

### `Commands.runEnd`

Runs one thing while active, then runs cleanup when the command ends.

```java
Commands.runEnd(
    () -> intake.run(0.5),
    () -> intake.stop(),
    intake);
```

Use this for motors. Start the motor while the command is active, stop it when the command ends.

### `Commands.sequence`

Runs commands one after another.

```java
Commands.sequence(
    Commands.runOnce(() -> shooter.setVelocityRPM(3000), shooter),
    Commands.waitSeconds(1.0),
    Commands.runEnd(() -> feeder.run(0.7), feeder::stop, feeder).withTimeout(2.0));
```

Use this for autos or multi-step actions.

### `Commands.parallel`

Runs commands at the same time.

```java
Commands.parallel(
    Commands.runEnd(() -> intake.run(0.5), intake::stop, intake),
    Commands.runEnd(() -> kicker.run(0.5), kicker::stop, kicker));
```

Use this when mechanisms should run together.

## Subsystem Requirements

The subsystem names at the end of command factories are requirements:

```java
Commands.runEnd(
    () -> intake.run(0.5),
    () -> intake.stop(),
    intake);
```

The `intake` at the end tells WPILib, "this command uses the intake subsystem."

Requirements matter because WPILib prevents two commands from controlling the same subsystem at the same time. If a new command requiring `intake` starts, the old intake command is interrupted.

Always list every subsystem the command controls.

## Trigger Bindings

Controller bindings usually live in `RobotContainer.configureButtonBindings()`.

### `whileTrue`

Runs while the button is held. Ends when released.

```java
operatorController.leftTrigger()
    .whileTrue(ManipulationCommands.shoot(intake, shooter, kicker));
```

Use for actions the driver should hold.

### `onTrue`

Starts once when the button is pressed.

```java
controller.start()
    .onTrue(Commands.runOnce(drive::zeroYaw, drive));
```

Use for reset buttons, mode changes, or one-shot actions.

### `toggleOnTrue`

Press once to start, press again to stop.

```java
operatorController.a()
    .toggleOnTrue(ManipulationCommands.toggleIntake(intake, kicker));
```

Use carefully. Toggles are useful, but they can confuse drivers if there is no clear feedback.

## Command Chaining

Commands can be modified with chain methods.

### `withTimeout`

Stops a command after a time limit.

```java
ManipulationCommands.shoot(intake, shooter, kicker)
    .withTimeout(3.0)
```

### `until`

Stops when a condition becomes true.

```java
Commands.run(() -> arm.moveToTarget(), arm)
    .until(() -> arm.atTarget())
```

### `onlyIf`

Only runs if a condition is true at the start.

```java
ManipulationCommands.deployArm(arm)
    .onlyIf(() -> !climber.isExtended())
```

### `andThen`

Runs another command after this one finishes.

```java
DriveCommands.driveToPose(drive, vision, () -> targetPose)
    .andThen(Commands.runOnce(drive::stop, drive))
```

## Suppliers

A supplier is a function that returns a value.

Common supplier types:

- `DoubleSupplier`: returns a `double`
- `BooleanSupplier`: returns a `boolean`
- `Supplier<T>`: returns some object type `T`

Example:

```java
() -> Constants.ShooterConstants.kShootRPM.getAsDouble()
```

This returns the current shooter RPM setting when the command asks for it. That is useful for tunable values because the value can change while the program is running.

## Static Command Classes

This project uses command factory classes like:

- `DriveCommands`
- `ManipulationCommands`
- `ClimbCommands`
- `TestCommands`

These classes usually have static methods that return commands:

```java
public static Command holdIntake(Intake intake, Kicker kicker) {
  return Commands.runEnd(
      () -> {
        intake.run(0.5);
        kicker.run(0.5);
      },
      () -> {
        intake.stop();
        kicker.stop();
      },
      intake,
      kicker);
}
```

This style keeps `RobotContainer` cleaner. `RobotContainer` should bind buttons. It should not contain every detail of how a mechanism action works.

## Reading a Command Out Loud

Example:

```java
operatorController.a()
    .toggleOnTrue(ManipulationCommands.toggleIntake(intake, kicker));
```

Read it as:

"When the operator presses A, toggle the intake command. That command controls the intake and kicker."

Example:

```java
Commands.runEnd(
    () -> intake.run(Constants.IntakeConstants.intakeSpeed),
    () -> intake.stop(),
    intake);
```

Read it as:

"While this command is active, run the intake. When the command ends, stop the intake. This command requires the intake subsystem."

## Common Mistakes

- Forgetting subsystem requirements.
- Reading a joystick once instead of using `() ->`.
- Putting too much logic directly in `RobotContainer`.
- Using `toggleOnTrue` when `whileTrue` would be safer.
- Forgetting to stop motors in the end action.
- Creating two commands that fight over the same subsystem.
- Assuming code inside `() ->` runs immediately.

## Rule of Thumb

If the value should update while the command runs, use a lambda like `() -> value`.

If a motor should stop when the command ends, use `Commands.runEnd`.

If a command controls a subsystem, list that subsystem as a requirement.

Next: [Adding New Robot Features](adding-new-features.md)
