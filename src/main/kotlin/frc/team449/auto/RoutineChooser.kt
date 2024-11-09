package frc.team449.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Robot
import frc.team449.auto.routines.DoNothing
import frc.team449.auto.routines.ExampleRoutine
import frc.team449.auto.routines.HighOne

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "RedDoNothing" to DoNothing(robot).createCommand(),
      "BlueDoNothing" to DoNothing(robot).createCommand(),
      "RedExample" to ExampleRoutine(robot, true).createCommand(),
      "BlueExample" to ExampleRoutine(robot, false).createCommand(),
      "RedHighOne" to HighOne(robot, true).createCommand(),
      "BlueHighOne" to HighOne(robot, false).createCommand(),
    )
  }

  fun createOptions() {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")
    this.addOption("The Example Auto", "Example")
    this.addOption("High 1", "HighOne")
  }
}
