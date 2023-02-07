package frc.team449.robot2022.indexer

import frc.team449.system.motor.WrappedMotor

class Indexer(
  private val indexMotor: WrappedMotor,
) {

  init {
    indexMotor.setVoltage(IndexerConstants.DESIRED_VOLTAGE)
  }
}
