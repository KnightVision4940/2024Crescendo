# Improvements for 2024-2025 Season / Things We Did Wrong
- [ ] READ DOCUMENTATION!
- [ ] When updating the shuffleboard with the gyro angle, our code would indefinitely
      create a new shuffleboard element, burning our CPU usage.
- [ ] Subsystems should be kept concise and consistent. This includes naming conventions,
      paramater inputs (especially positive vs. negative values), and instances of commands.
   - Eg. Conveyor should ONLY be in Conveyor.java
- [ ] Name methods better. run() and cancel() NEED to be more descriptive (found in commands)
- [ ] Have a clear reference of which speeds are positive and negative, and
      DO NOT change constants in Constants.java to get belts turning the right way
- [ ] Regularly Commit and Push code to keep it updated. Worst case scenario,
      we just go back a generation
- [ ] Troubleshoot efficiently so Engineering/Design has more time to build.
- [ ] Be able to recognize where code would not work or wouldn't make sense even
      before running it; think of how the code would run in theory.
- [ ] FPGATime is in Seconds, not milliseconds.

# Things We Did Good
-

# Checklist for 2024-2025 Season
- [ ] Update WPILib to 2025
- [ ] Make working code
- [ ] Make sure to put comments so you do not get lost
- [ ] Use coding Bible (If it is lost, go find it)
- [ ] Use PathPlannerLib to make your autos (see 2024Crescendo Project in PathPlanner for examples)
