package spinal.lib.bus.amba4.axi

import spinal.core._
import spinal.lib._

/**
 * Created by HTTDES on 19/05/2019.
 */

case class Axi4SharedToIO(config : Axi4Config) extends Component{
  val io = new Bundle{
    val axi     = slave(Axi4Shared(config))
    val conduit = slave(Axi4(config)).flip
  }

  io.conduit <> io.axi.toAxi4()
}
