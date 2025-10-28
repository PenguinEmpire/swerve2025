## Quick summary

This is an FRC-style robot project (WPILib + GradleRIO) implementing a swerve-drive robot using the Command-based framework. Key libraries: swervelib (custom swerve implementation), PathPlanner, and dev.alphagame utilities (logging/memory monitor).

Keep guidance concise and actionable — reference files and patterns below when making edits or generating code.

## Big-picture architecture

- Entry point: `org.penguinempire.Main` (do not add static initialization here). Build-time `BuildInfo` is auto-generated (see `build.gradle` -> `generateBuildInfo`).
- Robot lifecycle: `org.penguinempire.Robot` (TimedRobot) calls into `org.penguinempire.RobotContainer`.
- Composition: subsystems are in `src/main/java/org/penguinempire/subsystems`, commands in `org/penguinempire/commands`, and reusable pieces in `org/penguinempire/modules`.
- Swerve drive runtime: `SwerveSubsystem` parses JSON from `deploy/swerve` (deploy path: `src/main/deploy/swerve`) via `swervelib.parser.SwerveParser` and exposes path-following/autonomous commands using PathPlanner.

Why this structure: Command-based pattern keeps side-effects in subsystems and composes behavior with Commands/Sequences in `RobotContainer`.

## Key files and examples (explicit references)

- `Main.java` — program entry, starts `Robot::new` and Rampage memory monitor.
- `Robot.java` — scheduler run loop and mode transitions.
- `RobotContainer.java` — subsystem instances, default commands and controller bindings. Example default command: `drivebase.setDefaultCommand(new SwerveDriveCommand(...))`.
- `Constants.java` — hardware IDs, tuning constants. Prefer adding constant values here and referencing them instead of hardcoding numbers.
- `SwerveSubsystem.java` — shows PathPlanner + SwerveParser usage and the `driveFieldOriented()` helper.
- `src/main/deploy` — path files, JSON swerve configs and PathPlanner settings (these files are deployed to the RoboRIO under `/home/lvuser/deploy`).

## Build / test / deploy workflows (project-specific)

- Build & run unit tests locally: `./gradlew build` and `./gradlew test` (JUnit 5 configured in `build.gradle`).
- Generate BuildInfo: automatically run as part of `./gradlew build` (see `generateBuildInfo` task in `build.gradle`).
- Deploy to RoboRIO: `./gradlew deploy` (GradleRIO handles target and artifacts; team number is taken from WPILib preferences or Gradle properties). See `deploy { ... }` in `build.gradle` for artifact mapping and `src/main/deploy` copy behavior.
- Simulation: the project enables simulation GUI (see `wpi.sim.addGui()`), use the GradleRIO simulation tasks per GradleRIO docs when you need to run desktop simulation.

If you need to change how the team number is supplied, update WPILib preferences or pass the appropriate Gradle property per GradleRIO docs.

## Project-specific conventions & patterns

- Package layout: `org.penguinempire.subsystems`, `.commands`, `.modules` — follow this structure for new code.
- Constants: put all IDs and tuning values in `Constants.java` (e.g., motor IDs under `Constants.Intake`, `Constants.Shooter`).
- Subsystems expose Command-returning helpers for default/autonomous behaviors, e.g. `driveFieldOriented(Supplier<ChassisSpeeds>)` and `getAutonomousCommand(String)`.
- Use `SendableChooser` in `RobotContainer` for selecting autos (example in `RobotContainer.getAutonomousCommand()`).
- Logging: use `dev.alphagame.trailblazer.LogManager` for consistent logging across the codebase.

## Integration & external dependencies to watch

- swervelib: swerve parser, telemetry and drive primitives — configuration comes from `src/main/deploy/swerve/*.json`.
- PathPlanner: paths live under `src/main/deploy/pathplanner`, and PathPlanner auto commands are created in `SwerveSubsystem.setupPathPlanner()`.
- Vendor libs referenced in `vendordeps` and declared in `build.gradle` — do not duplicate these in new code; depend on their public APIs only.

## Safe editing rules (do not invent runtime expectations)

- Do not add static initialization to `Main` or `Robot` outside of their established patterns.
- `BuildInfo` is generated at build time; do not add a checked-in `BuildInfo.java` in `src`.
- When touching hardware constants, update `Constants.java` and add a comment if the ID hasn't been validated on bench hardware.

## How to help in PRs or code-generation tasks

- When proposing code changes, include:
  - The minimal code diff plus the small test or simulation verification you used (e.g., `./gradlew test` results or simulation run notes).
  - Any modified deploy JSONs under `src/main/deploy` with a short note about team-tested values.
- For code generation: prefer small, focused commits that add one subsystem/command at a time and wire it into `RobotContainer` with clear default command behavior.

## When you need more context (where to look)

- `src/main/java/org/penguinempire/RobotContainer.java` — button bindings and default command wiring.
- `src/main/java/org/penguinempire/subsystems/SwerveSubsystem.java` — path following, swerve initialization and examples.
- `src/main/deploy` — robot JSONs and PathPlanner assets that must be preserved in deployments.
- `build.gradle` — GradleRIO setup, BuildInfo generation and deploy artifact configuration.

If anything is unclear or you need specific examples (e.g., how to add a new command tied to a controller mapping), ask for the exact scenario and I will add a short recipe and examples referencing the concrete files above.
