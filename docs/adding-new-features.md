# Adding New Robot Features

Use this checklist when adding a new mechanism, sensor, command, or autonomous behavior.

## New Mechanism Subsystem

1. Add constants in `Constants.java`.
2. Create a new package under `src/main/java/frc/robot/subsystems`.
3. Create an IO interface with an `@AutoLog` inputs class.
4. Create the subsystem class.
5. Create a real IO implementation.
6. Create a sim IO implementation if the mechanism affects development or autos.
7. Instantiate the subsystem in all three `RobotContainer` mode branches.
8. Add commands in `src/main/java/frc/robot/commands`.
9. Bind controller buttons or register autonomous named commands.
10. Build and test.

## New Sensor

If the sensor belongs to one mechanism, put it in that mechanism's IO layer.

If it is shared across the whole robot, create a dedicated subsystem or utility only if there is a clear owner.

Always log raw values before building decisions on top of them.

## New Command

Command factories usually belong in a file under `commands/`.

Use commands for actions like:

- run intake while held
- spin up a scoring mechanism then feed a game piece
- drive to a pose
- climb for a fixed time
- run a test routine

Keep commands small and reusable. If a command needs the same calculation many times, move that calculation into a subsystem method.

If the command syntax looks confusing, read [WPILib Command Syntax](wpilib-command-syntax.md) before writing more command code.

## New Controller Binding

Add bindings in `RobotContainer.configureButtonBindings()`.

Choose the trigger style intentionally:

- `whileTrue`: action should stop when released.
- `onTrue`: one-shot action.
- `toggleOnTrue`: persistent action toggled by button press.

Make sure the command requires the subsystems it controls.

## New Autonomous Action

1. Build the command in `commands/`.
2. Register it with `NamedCommands.registerCommand(...)`.
3. Add the matching event marker in PathPlanner.
4. Test in sim.
5. Check logs.

## Before Real Robot Testing

Check:

- CAN IDs.
- motor inversion.
- current limits.
- idle modes.
- sensor units.
- limit conditions.
- command interruption behavior.
- dashboard tunables.
- logs in AdvantageScope.

## Build Command

Use:

```bash
./gradlew build
```

For a quicker compile check:

```bash
./gradlew -q compileJava
```

Next: [External Code and Templates](external-code-and-templates.md)
