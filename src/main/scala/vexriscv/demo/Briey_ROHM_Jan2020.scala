package vexriscv.demo

import vexriscv.plugin._
import vexriscv._
import vexriscv.ip.{DataCacheConfig, InstructionCacheConfig}

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.amba4.axi._
import spinal.lib.com.spi._
import spinal.lib.com.jtag.Jtag
import spinal.lib.com.uart.{Apb3UartCtrl, Uart, UartCtrlGenerics, UartCtrlMemoryMappedConfig}
import spinal.lib.graphic.RgbConfig
import spinal.lib.graphic.vga.{Axi4VgaCtrl, Axi4VgaCtrlGenerics, Vga}
import spinal.lib.io.TriStateArray
import spinal.lib.memory.sdram._
import spinal.lib.misc.HexTools
import spinal.lib.soc.pinsec.{PinsecTimerCtrl, PinsecTimerCtrlExternal}
import spinal.lib.system.debugger.{JtagAxi4SharedDebugger, JtagBridge, SystemDebugger, SystemDebuggerConfig}

import scala.collection.mutable.ArrayBuffer

case class Briey_ROHM_Config(
			bootRomSize : BigInt,
			onChipRamSize : BigInt,
			spiCtrlConfig : SpiMasterCtrlGenerics,
			cpuPlugins : ArrayBuffer[Plugin[VexRiscv]],
			uartCtrlConfig : UartCtrlMemoryMappedConfig)

object Briey_ROHM_Config{
  def default = {
    val config = Briey_ROHM_Config(
      bootRomSize = 8 kB,
      onChipRamSize = 16 kB,

      spiCtrlConfig = SpiMasterCtrlGenerics(
        ssWidth = 1,
        timerWidth = 16,
        dataWidth = 8
      ),

      uartCtrlConfig = UartCtrlMemoryMappedConfig(
        uartCtrlConfig = UartCtrlGenerics(
          dataWidthMax      = 8,
          clockDividerWidth = 20,
          preSamplingSize   = 1,
          samplingSize      = 5,
          postSamplingSize  = 2
        ),
        txFifoDepth = 16,
        rxFifoDepth = 16
      ),

      cpuPlugins = ArrayBuffer(
        new PcManagerSimplePlugin(0x80000000l, false),

//      new IBusSimplePlugin(
//        interfaceKeepData = false,
//        catchAccessFault = true
//      ),

        new IBusCachedPlugin(
          resetVector = 0x40000000l,
          prediction = STATIC,
          config = InstructionCacheConfig(
            cacheSize = 256,
            bytePerLine =32,
            wayCount = 1,
            addressWidth = 32,
            cpuDataWidth = 32,
            memDataWidth = 32,
            catchIllegalAccess = true,
            catchAccessFault = true,
            asyncTagMemory = false,
            twoCycleRam = true,
            twoCycleCache = true
          )
//        askMemoryTranslation = true,
//        memoryTranslatorPortConfig = MemoryTranslatorPortConfig(portTlbSize = 4)
        ),

//      new DBusSimplePlugin(
//        catchAddressMisaligned = true,
//        catchAccessFault = true
//      ),

        new DBusCachedPlugin(
          config = new DataCacheConfig(
            cacheSize        = 256,
            bytePerLine      = 32,
            wayCount         = 1,
            addressWidth     = 32,
            cpuDataWidth     = 32,
            memDataWidth     = 32,
            catchAccessError = true,
            catchIllegal     = true,
            catchUnaligned   = true
          ),
          memoryTranslatorPortConfig = null
//        memoryTranslatorPortConfig = MemoryTranslatorPortConfig(portTlbSize = 6)
        ),

        new StaticMemoryTranslatorPlugin(ioRange = _(31 downto 28) === 0xF),

        new DecoderSimplePlugin(catchIllegalInstruction = true),

        new RegFilePlugin(
          regFileReadyKind = plugin.SYNC,
          zeroBoot = false
        ),

        new IntAluPlugin,

        new SrcPlugin(
          separatedAddSub = false,
          executeInsertion = true
        ),

        new FullBarrelShifterPlugin,
        new MulPlugin,
        new DivPlugin,

        new HazardSimplePlugin(
          bypassExecute           = true,
          bypassMemory            = true,
          bypassWriteBack         = true,
          bypassWriteBackBuffer   = true,
          pessimisticUseSrc       = false,
          pessimisticWriteRegFile = false,
          pessimisticAddressMatch = false
        ),

        new BranchPlugin(
          earlyBranch = false,
          catchAddressMisaligned = true
        ),

        new CsrPlugin(
          config = CsrPluginConfig(
            catchIllegalAccess = false,
            mvendorid          = null,
            marchid            = null,
            mimpid             = null,
            mhartid            = null,
            misaExtensionsInit = 66,
            misaAccess         = CsrAccess.NONE,
            mtvecAccess        = CsrAccess.NONE,
            mtvecInit          = 0x80000020l,
            mepcAccess         = CsrAccess.READ_WRITE,
            mscratchGen        = false,
            mcauseAccess       = CsrAccess.READ_ONLY,
            mbadaddrAccess     = CsrAccess.READ_ONLY,
            mcycleAccess       = CsrAccess.NONE,
            minstretAccess     = CsrAccess.NONE,
            ecallGen           = false,
            wfiGenAsWait       = false,
            ucycleAccess       = CsrAccess.NONE
          )
        ),

        new YamlPlugin("cpu0.yaml")
      )
    )
    config
  }
}

class Briey_ROHM_Jan2020(config: Briey_ROHM_Config) extends Component{
  import config._
  val debug = true
  val interruptCount = 4
  
  val io = new Bundle{
    //Clocks / reset
    val asyncReset = in Bool()
    val axiClk     = in Bool()
    
    //Main components IO
    val jtag  = slave(Jtag())
    
    //Peripherals IO
    val gpioA         = master(TriStateArray(16 bits))
    val spi           = master(SpiMaster(config.spiCtrlConfig.ssWidth))
    val uart          = master(Uart())
    val timerExternal = in(PinsecTimerCtrlExternal())
    val coreInterrupt = in Bool()
  }

  val resetCtrlClockDomain = ClockDomain(
    clock = io.axiClk,
    config = ClockDomainConfig(resetKind = BOOT)
  )

  val resetCtrl = new ClockingArea(resetCtrlClockDomain) {
    val systemResetUnbuffered  = False
//  val coreResetUnbuffered = False

    //Implement an counter to keep the reset axiResetOrder high 64 cycles
    // Also this counter will automaticly do a reset when the system boot.
    val systemResetCounter = Reg(UInt(6 bits)) init(0)
    when(systemResetCounter =/= U(systemResetCounter.range -> true)) {
      systemResetCounter := systemResetCounter + 1
      systemResetUnbuffered := True
    }
    when(BufferCC(io.asyncReset)) {systemResetCounter := 0}

    //Create all reset used later in the design
    val systemReset  = RegNext(systemResetUnbuffered)
    val axiReset     = RegNext(systemResetUnbuffered)
  }

  val axiClockDomain = ClockDomain(
    clock = io.axiClk,
    reset = resetCtrl.axiReset
  )

  val debugClockDomain = ClockDomain(
    clock = io.axiClk,
    reset = resetCtrl.systemReset
  )

  val axi = new ClockingArea(axiClockDomain) {
    val rom = Axi4SharedOnChipRam(
      dataWidth = 32,
      byteCount = bootRomSize,
      idWidth = 4
    )

    val ram = Axi4SharedOnChipRam(
      dataWidth = 32,
      byteCount = onChipRamSize,
      idWidth = 4
    )

    val apbBridge = Axi4SharedToApb3Bridge(
      addressWidth = 20,
      dataWidth    = 32,
      idWidth      = 4
    )

    val gpioACtrl = Apb3Gpio(
      gpioWidth = 16,
      withReadSync = true
    )

    val timerCtrl = PinsecTimerCtrl()

    val spiCtrl = Apb3SpiMasterCtrl(new SpiMasterCtrlMemoryMappedConfig(
                                      ctrlGenerics = config.spiCtrlConfig,
                                      cmdFifoDepth = 32,
                                      rspFifoDepth = 32 ))

    val uartCtrl = Apb3UartCtrl(uartCtrlConfig)

    val core = new Area {
      val config = VexRiscvConfig(plugins = cpuPlugins += new DebugPlugin(debugClockDomain))
      val cpu = new VexRiscv(config)
      var iBus : Axi4ReadOnly = null
      var dBus : Axi4Shared = null
      for(plugin <- config.plugins) plugin match {
        case plugin : IBusSimplePlugin => iBus = plugin.iBus.toAxi4ReadOnly()
        case plugin : IBusCachedPlugin => iBus = plugin.iBus.toAxi4ReadOnly()
        case plugin : DBusSimplePlugin => dBus = plugin.dBus.toAxi4Shared()
        case plugin : DBusCachedPlugin => dBus = plugin.dBus.toAxi4Shared(true)
        case plugin : CsrPlugin        => {
          plugin.externalInterrupt := BufferCC(io.coreInterrupt)
          plugin.timerInterrupt := timerCtrl.io.interrupt
        }
        case plugin : DebugPlugin      => debugClockDomain{
          resetCtrl.axiReset setWhen(RegNext(plugin.io.resetOut))
          io.jtag <> plugin.io.bus.fromJtag()
        }
        case _ =>
      }
    }

    val axiCrossbar = Axi4CrossbarFactory()

    axiCrossbar.addSlaves(
      rom.io.axi       -> (0x40000000L,   bootRomSize),
      ram.io.axi       -> (0x80000000L,   onChipRamSize),
      apbBridge.io.axi -> (0xF0000000L,   1 MB)
    )

    axiCrossbar.addConnections(
      core.iBus       -> List(rom.io.axi, ram.io.axi),
      core.dBus       -> List(rom.io.axi, ram.io.axi, apbBridge.io.axi)
    )

    axiCrossbar.addPipelining(apbBridge.io.axi)((crossbar,bridge) => {
      crossbar.sharedCmd.halfPipe() >> bridge.sharedCmd
      crossbar.writeData.halfPipe() >> bridge.writeData
      crossbar.writeRsp             << bridge.writeRsp
      crossbar.readRsp              << bridge.readRsp
    })

    axiCrossbar.addPipelining(rom.io.axi)((crossbar,ctrl) => {
      crossbar.sharedCmd.halfPipe()  >>  ctrl.sharedCmd
      crossbar.writeData            >/-> ctrl.writeData
      crossbar.writeRsp              <<  ctrl.writeRsp
      crossbar.readRsp               <<  ctrl.readRsp
    })

    axiCrossbar.addPipelining(ram.io.axi)((crossbar,ctrl) => {
      crossbar.sharedCmd.halfPipe()  >>  ctrl.sharedCmd
      crossbar.writeData            >/-> ctrl.writeData
      crossbar.writeRsp              <<  ctrl.writeRsp
      crossbar.readRsp               <<  ctrl.readRsp
    })

    axiCrossbar.addPipelining(core.dBus)((cpu,crossbar) => {
      cpu.sharedCmd             >>  crossbar.sharedCmd
      cpu.writeData             >>  crossbar.writeData
      cpu.writeRsp              <<  crossbar.writeRsp
      cpu.readRsp               <-< crossbar.readRsp //Data cache directly use read responses without buffering, so pipeline it for FMax
    })

    axiCrossbar.build()

    val apbDecoder = Apb3Decoder(
      master = apbBridge.io.apb,
      slaves = List(
        gpioACtrl.io.apb -> (0x00000, 4 kB),
	spiCtrl.io.apb   -> (0x02000, 4 kB),
        uartCtrl.io.apb  -> (0x10000, 4 kB),
        timerCtrl.io.apb -> (0x20000, 4 kB)
      )
    )
  }

  io.gpioA         <> axi.gpioACtrl.io.gpio
  io.timerExternal <> axi.timerCtrl.io.external
  io.spi           <> axi.spiCtrl.io.spi
  io.uart          <> axi.uartCtrl.io.uart
}

object Briey_ROHM_Jan2020{
  def main(args: Array[String]) {
    val config = SpinalConfig()
    config.generateVerilog({
      val toplevel = new Briey_ROHM_Jan2020(Briey_ROHM_Config.default)
      toplevel
    })
  }
}

