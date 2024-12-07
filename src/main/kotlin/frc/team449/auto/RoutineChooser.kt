package frc.team449.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Robot
import frc.team449.auto.routines.*

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "RedDoNothing" to DoNothing(robot).createCommand(),
      "BlueDoNothing" to DoNothing(robot).createCommand(),
      "RedTaxi" to Taxi(robot, true).createCommand(),
      "BlueTaxi" to Taxi(robot, false).createCommand(),
      "RedPreload" to Preload(robot, true).createCommand(),
      "BluePreload" to Preload(robot, false).createCommand(),
      "RedHighOne" to PreloadHighAndPickup(robot, true).createCommand(),
      "BlueHighOne" to PreloadHighAndPickup(robot, false).createCommand(),
      "RedStack3" to Stack3(robot, true).createCommand(),
      "BlueStack3" to Stack3(robot, false).createCommand(),
      "RedFullPath" to FullPath(robot, true).createCommand(),
      "BlueFullPath" to FullPath(robot, false).createCommand()
    )
  }

  fun createOptions() {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")
    this.addOption("Taxi", "Taxi")
    this.addOption("Drop pre-load", "Preload")
    this.addOption("Preload High and Pickup", "HighOne")
    this.addOption("Score Stack at Pos 3 and pickup at bucket 1", "Stack3")
    this.addOption("Do all of the Stacks, FullPath", "FullPath")
  }
}
