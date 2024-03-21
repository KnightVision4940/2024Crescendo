# Improvements for 2024-2025 Season / Things We Did Wrong
- [ ] READ DOCUMENTATION!
- [ ] The shuffleboard code that returns the Gyro Angle would run a process once, then double 
      everytime as it repeats that command (eg. 1-->2-->4...). 
      - This ate a lot of our CPU power during comp.
- [ ] Subsystems should be kept concise and consistent. This includes naming conventions, 
      paramater inputs (especially positive vs. negative values), and instances of commands.
   - Eg. Conveyor should not be in Intake.java, Conveyor.java, AND Shooter.java
- [ ] Name methods better. run() and cancel() NEED to be more descriptive (found in commands)
- [ ] Have a clear reference of which speeds are positive and negative, and 
      DO NOT change constants in Constants.java to get belts turning the right way
- [ ] Regularly Commit and Push code to keep it updated. Worst case scenario, 
      we just go back a generation
- [ ] Troubleshoot efficiently so Engineering/Design has more time to build. 
- [ ] Be able to recognize where code would not work or wouldn't make sense even 
      before running it; think of how the code would run in theory. 

# Things We Did Good
- The robot can move

# Checklist for 2024-2025 Season
- [ ] Update WPILib to 2025
- [ ] Make working code
- [ ] Make sure to put comments so you do not get lost
- [ ] Use coding Bible (If it is lost, go find it)