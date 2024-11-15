package frc.team449.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Robot
import frc.team449.auto.routines.DoNothing
import frc.team449.auto.routines.Preload
import frc.team449.auto.routines.PreloadHighAndPickup
import frc.team449.auto.routines.Taxi

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
    )
  }

  fun createOptions() {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")
    this.addOption("Taxi", "Taxi")
    this.addOption("Drop pre-load", "Preload")
    this.addOption("Preload High and Pickup", "HighOne")
  }
}
