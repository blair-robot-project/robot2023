package frc.team449.robot2022.auto

import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile

object AutoConstants {

  /** Driving Config */
  const val MAX_VEL = .5
  const val MAX_ACC = 2.0
  private const val MAX_ROTVEL = 2.0
  private const val MAX_ROTACC = 2.0

  /** PID gains */
  const val TRANSLATION_KP = .02
  const val ROTATION_KP = 2.0

  /** PID controllers for each degree of freedom */
  val ROT_CONTROLLER = ProfiledPIDController(
    ROTATION_KP,
    .0,
    .0,
    TrapezoidProfile.Constraints(MAX_ROTVEL, MAX_ROTACC)
  )
  val xController = PIDController(TRANSLATION_KP, .0, .0)
  val yController = PIDController(TRANSLATION_KP, .0, .0)

  /** Controller used to calculate Pose error when following path */
  val controller = HolonomicDriveController(
    xController,
    yController,
    ROT_CONTROLLER
  )

  var points = arrayOf(doubleArrayOf(0.0, 0.0, 0.0, -1.57, -0.0, -0.0, 0.0), doubleArrayOf(0.0025, -0.0, -0.0, -1.57, -0.0, 0.0, -0.0446), doubleArrayOf(0.005, -0.0, 0.0, -1.5701, -0.0, 0.0, -0.0892), doubleArrayOf(0.0075, -0.0, 0.0, -1.5703, -0.0, 0.0, -0.1338), doubleArrayOf(0.01, -0.0, 0.0, -1.5707, -0.0, 0.0, -0.1784), doubleArrayOf(0.0126, -0.0, 0.0, -1.5711, -0.0, 0.0, -0.223), doubleArrayOf(0.0151, -0.0, 0.0, -1.5717, -0.0, 0.0, -0.2675), doubleArrayOf(0.0176, -0.0, 0.0, -1.5724, -0.0, 0.0, -0.3121), doubleArrayOf(0.0201, -0.0, 0.0, -1.5731, -0.0, 0.0, -0.3567), doubleArrayOf(0.0226, -0.0, 0.0, -1.574, -0.0, 0.0, -0.4013), doubleArrayOf(0.0251, -0.0, 0.0, -1.575, -0.0, 0.0, -0.4459), doubleArrayOf(0.0276, -0.0, 0.0, -1.5762, -0.0, 0.0, -0.4905), doubleArrayOf(0.0301, -0.0, 0.0, -1.5774, -0.0, 0.0, -0.5351), doubleArrayOf(0.0327, -0.0, 0.0, -1.5787, -0.0, 0.0, -0.5797), doubleArrayOf(0.0352, -0.0, 0.0, -1.5802, -0.0, 0.0, -0.6243), doubleArrayOf(0.0377, -0.0, 0.0, -1.5818, -0.0, 0.0, -0.6689), doubleArrayOf(0.0402, -0.0, 0.0, -1.5834, -0.0, 0.0, -0.7134), doubleArrayOf(0.0427, -0.0, 0.0, -1.5852, -0.0, 0.0, -0.758), doubleArrayOf(0.0452, -0.0, 0.0, -1.5871, -0.0, 0.0, -0.8026), doubleArrayOf(0.0477, -0.0, 0.0, -1.5892, -0.0, 0.0, -0.8472), doubleArrayOf(0.0502, -0.0, 0.0, -1.5913, -0.0, 0.0, -0.8918), doubleArrayOf(0.0527, -0.0, 0.0, -1.5935, -0.0, 0.0, -0.9364), doubleArrayOf(0.0553, -0.0, 0.0, -1.5959, -0.0, 0.0, -0.981), doubleArrayOf(0.0578, -0.0, 0.0, -1.5983, -0.0, 0.0, -1.0256), doubleArrayOf(0.0603, -0.0, 0.0, -1.6009, -0.0, 0.0, -1.0702), doubleArrayOf(0.0628, -0.0, 0.0, -1.6036, -0.0, 0.0, -1.1148), doubleArrayOf(0.0653, -0.0, 0.0, -1.6064, -0.0, 0.0, -1.1593), doubleArrayOf(0.0678, -0.0, 0.0, -1.6093, -0.0, 0.0, -1.2039), doubleArrayOf(0.0703, -0.0, 0.0, -1.6123, -0.0, 0.0, -1.2485), doubleArrayOf(0.0728, -0.0, 0.0, -1.6155, -0.0, 0.0, -1.2931), doubleArrayOf(0.0754, -0.0, 0.0, -1.6187, -0.0, 0.0, -1.3377), doubleArrayOf(0.0779, -0.0, 0.0, -1.6221, -0.0, 0.0, -1.3823), doubleArrayOf(0.0804, -0.0, 0.0, -1.6256, -0.0, 0.0, -1.4269), doubleArrayOf(0.0829, -0.0, 0.0, -1.6291, -0.0, 0.0, -1.4715), doubleArrayOf(0.0854, -0.0, 0.0, -1.6328, -0.0, 0.0, -1.5161), doubleArrayOf(0.0879, -0.0, 0.0, -1.6366, -0.0, 0.0, -1.5607), doubleArrayOf(0.0904, -0.0, 0.0, -1.6406, -0.0, 0.0, -1.6052), doubleArrayOf(0.0929, -0.0, 0.0, -1.6446, -0.0, 0.0, -1.6498), doubleArrayOf(0.0954, -0.0, 0.0, -1.6487, -0.0, 0.0, -1.6944), doubleArrayOf(0.098, -0.0, 0.0, -1.653, -0.0, 0.0, -1.739), doubleArrayOf(0.1005, -0.0, 0.0, -1.6574, -0.0, 0.0, -1.7836), doubleArrayOf(0.103, -0.0, 0.0, -1.6618, -0.0, 0.0, -1.8282), doubleArrayOf(0.1055, -0.0, 0.0, -1.6664, -0.0, 0.0, -1.8728), doubleArrayOf(0.108, -0.0, 0.0, -1.6711, -0.0, 0.0, -1.9174), doubleArrayOf(0.1105, -0.0, 0.0, -1.676, -0.0, 0.0, -1.962), doubleArrayOf(0.113, -0.0, 0.0, -1.6809, -0.0, 0.0, -2.0066), doubleArrayOf(0.1155, -0.0, 0.0, -1.6859, -0.0, 0.0, -2.0511), doubleArrayOf(0.1181, -0.0, 0.0, -1.6911, -0.0, 0.0, -2.0957), doubleArrayOf(0.1206, -0.0, 0.0, -1.6963, -0.0, 0.0, -2.1403), doubleArrayOf(0.1231, -0.0, 0.0, -1.7017, -0.0, 0.0, -2.1849), doubleArrayOf(0.1256, -0.0, 0.0, -1.7072, 0.0, 0.0, -2.2295), doubleArrayOf(0.1281, -0.0, 0.0, -1.7128, 0.0, 0.0, -2.1849), doubleArrayOf(0.1306, -0.0, 0.0, -1.7183, 0.0, 0.0, -2.1403), doubleArrayOf(0.1331, -0.0, 0.0, -1.7237, 0.0, 0.0, -2.0957), doubleArrayOf(0.1356, -0.0, 0.0, -1.7289, 0.0, 0.0, -2.0511), doubleArrayOf(0.1381, -0.0, 0.0, -1.7341, 0.0, 0.0, -2.0066), doubleArrayOf(0.1407, -0.0, 0.0, -1.7391, 0.0, 0.0, -1.962), doubleArrayOf(0.1432, -0.0, 0.0, -1.744, 0.0, 0.0, -1.9174), doubleArrayOf(0.1457, -0.0, 0.0, -1.7489, 0.0, 0.0, -1.8728), doubleArrayOf(0.1482, -0.0, 0.0, -1.7536, 0.0, 0.0, -1.8282), doubleArrayOf(0.1507, -0.0, 0.0, -1.7582, 0.0, 0.0, -1.7836), doubleArrayOf(0.1532, -0.0, 0.0, -1.7626, 0.0, 0.0, -1.739), doubleArrayOf(0.1557, -0.0, 0.0, -1.767, 0.0, 0.0, -1.6944), doubleArrayOf(0.1582, -0.0, 0.0, -1.7713, 0.0, 0.0, -1.6498), doubleArrayOf(0.1608, -0.0, 0.0, -1.7754, 0.0, 0.0, -1.6052), doubleArrayOf(0.1633, -0.0, 0.0, -1.7794, 0.0, 0.0, -1.5607), doubleArrayOf(0.1658, -0.0, 0.0, -1.7834, 0.0, 0.0, -1.5161), doubleArrayOf(0.1683, -0.0, 0.0, -1.7872, 0.0, 0.0, -1.4715), doubleArrayOf(0.1708, -0.0, 0.0, -1.7909, 0.0, 0.0, -1.4269), doubleArrayOf(0.1733, -0.0, 0.0, -1.7944, 0.0, 0.0, -1.3823), doubleArrayOf(0.1758, -0.0, 0.0, -1.7979, 0.0, 0.0, -1.3377), doubleArrayOf(0.1783, -0.0, 0.0, -1.8013, 0.0, 0.0, -1.2931), doubleArrayOf(0.1808, -0.0, 0.0, -1.8045, 0.0, 0.0, -1.2485), doubleArrayOf(0.1834, -0.0, 0.0, -1.8077, 0.0, 0.0, -1.2039), doubleArrayOf(0.1859, -0.0, 0.0, -1.8107, 0.0, 0.0, -1.1593), doubleArrayOf(0.1884, -0.0, 0.0, -1.8136, 0.0, 0.0, -1.1148), doubleArrayOf(0.1909, -0.0, 0.0, -1.8164, 0.0, 0.0, -1.0702), doubleArrayOf(0.1934, -0.0, 0.0, -1.8191, 0.0, 0.0, -1.0256), doubleArrayOf(0.1959, -0.0, 0.0, -1.8217, 0.0, 0.0, -0.981), doubleArrayOf(0.1984, -0.0, 0.0, -1.8241, 0.0, 0.0, -0.9364), doubleArrayOf(0.2009, -0.0, 0.0, -1.8265, 0.0, 0.0, -0.8918), doubleArrayOf(0.2035, -0.0, 0.0, -1.8287, 0.0, 0.0, -0.8472), doubleArrayOf(0.206, -0.0, 0.0, -1.8308, 0.0, 0.0, -0.8026), doubleArrayOf(0.2085, -0.0, 0.0, -1.8329, 0.0, 0.0, -0.758), doubleArrayOf(0.211, -0.0, 0.0, -1.8348, 0.0, 0.0, -0.7134), doubleArrayOf(0.2135, -0.0, 0.0, -1.8366, 0.0, 0.0, -0.6689), doubleArrayOf(0.216, -0.0, 0.0, -1.8382, 0.0, 0.0, -0.6243), doubleArrayOf(0.2185, -0.0, 0.0, -1.8398, 0.0, 0.0, -0.5797), doubleArrayOf(0.221, -0.0, 0.0, -1.8413, 0.0, 0.0, -0.5351), doubleArrayOf(0.2235, -0.0, 0.0, -1.8426, 0.0, 0.0, -0.4905), doubleArrayOf(0.2261, -0.0, 0.0, -1.8438, 0.0, 0.0, -0.4459), doubleArrayOf(0.2286, -0.0, 0.0, -1.845, 0.0, 0.0, -0.4013), doubleArrayOf(0.2311, -0.0, 0.0, -1.846, 0.0, 0.0, -0.3567), doubleArrayOf(0.2336, -0.0, 0.0, -1.8469, 0.0, 0.0, -0.3121), doubleArrayOf(0.2361, -0.0, 0.0, -1.8476, 0.0, 0.0, -0.2675), doubleArrayOf(0.2386, -0.0, 0.0, -1.8483, 0.0, 0.0, -0.223), doubleArrayOf(0.2411, -0.0, 0.0, -1.8489, 0.0, 0.0, -0.1784), doubleArrayOf(0.2436, -0.0, 0.0, -1.8493, 0.0, 0.0, -0.1338), doubleArrayOf(0.2462, -0.0, 0.0, -1.8497, 0.0, 0.0, -0.0892), doubleArrayOf(0.2487, -0.0, 0.0, -1.8499, 0.0, 0.0, -0.0446), doubleArrayOf(0.2512, 0.0, 0.0, -1.85, 0.0, 0.0, 0.0))
}
