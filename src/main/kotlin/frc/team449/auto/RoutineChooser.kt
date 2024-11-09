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
      "RedHighOne" to HighOne(robot, true).createCommand(),
      "BlueHighOne" to HighOne(robot, false).createCommand(),
    )
  }

  fun createOptions() {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")
    this.addOption("Taxi", "Taxi")
    this.addOption("Drop pre-load", "Preload")
    this.addOption("High 1", "HighOne")
  }
}
