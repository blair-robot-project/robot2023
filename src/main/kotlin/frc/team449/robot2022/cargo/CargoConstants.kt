package frc.team449.robot2022.cargo

object CargoConstants {
  const val INTAKE_LEADER_PORT = 8
  const val INTAKE_FOLLOWER_PORT = 10
  const val SPITTER_PORT = 9
  const val SHOOTER_PORT = 12
  const val INTAKE_PISTON_FWD_CHANNEL = 3
  const val INTAKE_PISTON_REV_CHANNEL = 2
  const val HOOD_PISTON_FWD_CHANNEL = 5
  const val HOOD_PISTON_REV_CHANNEL = 4
  const val INTAKE_CURR_LIM = 20
  const val SHOOTER_GEARING = 1.0
  const val SPITTER_GEARING = 1.0

  /** Spitter feedforward  */
  const val SPITTER_KS = -0.15654
  const val SPITTER_KV = .12658
  const val SPITTER_KA = .017184 // 0.171731 ks

  /** Shooter feedforward  */
  const val SHOOTER_KS = -0.15654
  const val SHOOTER_KV = .12658
  const val SHOOTER_KA = .017184 // 0.171731 ks

  /** How much is given to feeder and intake motors  */
  const val FEEDER_OUTPUT = 0.75

  /** How much is given to the spitter when actually spitting  */
  const val SPITTER_SPIT_SPEED_RPS = 22.0

  /** The speed of the spitter when actually intaking, in RPS  */
  const val SPITTER_INTAKE_SPEED_RPS = 31.0

  /** The speed of the spitter when shooting, in RPS  */
  const val SPITTER_SHOOT_SPEED_RPS = 41.5

  /** The speed of the shooter flywheel when shooting high, in RPS  */
  const val SHOOTER_SPEED_RPS = 46.5

  /** The speed of the spitter when shooting high in auto, in RPS  */
  const val AUTO_SPITTER_SHOOT_SPEED = 42.5

  /** The speed of the shooter flywheel when shooting high in auto, in RPS  */
  const val AUTO_SHOOTER_SPEED = 47.5

  /** The speed of the spitter flywheel when shooting high on the same side as low goal  */
  const val SPITTER_SHOOT_SPIT_SIDE_SPEED = 100.0

  /** The speed of the shooter flywheel when shooting high on the same side as low goal  */
  const val SHOOTER_SHOOT_SPIT_SIDE_SPEED = 50.0

  /** Seconds to wait for flywheel to reach target velocity when shooting high  */
  const val SHOOT_HIGH_SPINUP_TIME = 1.0

  /** How many seconds to reverse the intake before spinning up and shooting  */
  const val REVERSE_BEFORE_SHOOT_TIME = .07

  /* Speed of the intake while doing the high shooter sequence */
  const val INTAKE_SPEED_HIGH_SEQUENCE = 0.8

  /* Tolerance for the speed (shooter and spitter)*/
  const val SHOOTER_TOLERANCE = 2.5
}
