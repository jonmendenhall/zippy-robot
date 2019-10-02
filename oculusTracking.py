# implements the UVC webcam protocol
# heavily based on Oliver Kreylos's code for capturing frames from the Oculus CV1 Sensor
# https://github.com/Doc-Ok/OculusRiftCV1Camera
# loosely-translated by Jonathan Mendenhall to Python

import usb.core
import usb.util
from enum import IntEnum
import struct
import usb.backend.libusb1 as libusb1
import numpy as np
from ctypes import *
import time



def setCur(device, interface, entity, selector, data):
    requestType = (0x01 << 5) | (0x01)
    request = 0x01
    return device.ctrl_transfer(requestType & (~0x80 & 0xff), request, selector << 8, (entity << 8) | interface, data)

def getCur(device, interface, entity, selector, length):
    requestType = (0x01 << 5) | (0x01)
    request = 0x81
    return device.ctrl_transfer(requestType | 0x80, request, selector << 8, (entity << 8) | interface, length)


class ESP770U:

    class _Entities(IntEnum):
        ExtensionUnit = 4

    class _Selectors(IntEnum):
        I2C = 2
        Register = 3
        Counter = 10
        Control = 11
        Data = 12

    def __init__(self, device):
        self.device = device

    def setGetCur(self, selector, data):
        setCur(self.device, 0, self._Entities.ExtensionUnit, selector, data)
        return getCur(self.device, 0, self._Entities.ExtensionUnit, selector, len(data))

    def readRegister(self, register):
        cmd = [0x82, register >> 8 & 0xff, register & 0xff, 0x00]
        ret = self.setGetCur(self._Selectors.Register, cmd)
        if ret[0] != cmd[0] or ret[2] != 0x00:
            raise Exception('Invalid command buffer')
        return ret[1]

    def writeRegister(self, register, value):
        cmd = [0x02, register >> 8 & 0xff, register & 0xff, value]
        ret = self.setGetCur(self._Selectors.Register, cmd)
        if ret[0] != cmd[0] or ret[1] != cmd[1] or ret[2] != cmd[2] or ret[3] != value:
            raise Exception('Invalid command buffer')
        return True

    def queryFirmwareVersion(self):
        cmd = [0xa0, 0x03, 0x00, 0x00]
        ret = self.setGetCur(self._Selectors.Register, cmd)
        if ret[0] != cmd[0] or ret[2] != cmd[2] or ret[3] != cmd[3]:
            raise Exception('Invalid command buffer')
        return ret[1] 

    def initController(self):
        v = self.readRegister(0xf05a)
        if v != 0x01 and v != 0x03:
            print(f'register 0xf05a incorrect: {v}')
        self.writeRegister(0xf05a, 0x01)

        v = self.readRegister(0xf018)
        if v != 0x0e:
            print(f'register 0xf018 incorrect: {v}')
        self.writeRegister(0xf018, 0x0f)

        v = self.readRegister(0xf017)
        if v != 0xec and v != 0xed:
            print(f'register 0xf017 incorrect: {v}')
        self.writeRegister(0xf017, v | 0x01)
        self.writeRegister(0xf017, v & (~0x01 & 0xff))

        self.writeRegister(0xf018, 0x0e)

    def getCounter(self):
        return getCur(self.device, 0, self._Entities.ExtensionUnit, self._Selectors.Counter, 1)[0]

    def setCounter(self, counter):
        return setCur(self.device, 0, self._Entities.ExtensionUnit, self._Selectors.Counter, [counter])

    def readMemory(self, address, length):
        counter = self.getCounter()
        cmd = [0] * 16
        cmd[0] = counter
        cmd[1] = 0x41
        cmd[2] = 0x03
        cmd[3] = 0x01
        cmd[5] = address >> 16 & 0xff
        cmd[6] = address >> 8 & 0xff
        cmd[7] = address & 0xff
        cmd[8] = length >> 8 & 0xff
        cmd[9] = length & 0xff
        setCur(self.device, 0, self._Entities.ExtensionUnit, self._Selectors.Control, cmd)
        ret = getCur(self.device, 0, self._Entities.ExtensionUnit, self._Selectors.Data, length)
        self.setCounter(counter)
        return ret

    def readI2C(self, address, register):
        cmd = [0x86, address, register >> 8 & 0xff, register & 0xff, 0x00, 0x00]
        ret = self.setGetCur(self._Selectors.I2C, cmd)
        if ret[0] != 0x86 or ret[4] != 0x00 or ret[5] != 0x00:
            raise Exception('Invalid return buffer')
        return ret[2] << 8 | ret[1]

    def writeI2C(self, address, register, value):
        cmd = [0x06, address, register >> 8 & 0xff, register & 0xff, value >> 8 & 0xff, value & 0xff]
        ret = self.setGetCur(self._Selectors.I2C, cmd)
        if ret[0] != 0x06 or ret[1] != address or ret[2] != cmd[2] or ret[3] != cmd[3] or ret[4] != cmd[4] or ret[5] != cmd[5]:
            raise Exception('Invalid return buffer')
        return True





class AR0134:

    class Addresses(IntEnum):
        I2CAddress = 0x20

    class Registers(IntEnum):
        ChipVersionReg = 0x3000
        YAddrStart = 0x3002
        XAddrStart = 0x3004
        YAddrEnd = 0x3006
        XAddrEnd = 0x3008
        FrameLengthLines = 0x300a
        LineLengthPck = 0x300c
        RevisionNumber = 0x300e
        CoarseIntegrationTime = 0x3012
        FineIntegrationTime = 0x3014
        CoarseIntegrationTimeCb = 0x3016
        FineIntegrationTimeCb = 0x3018
        ResetRegister = 0x301a
        DataPedestal = 0x301e
        GpiStatus = 0x3026
        RowSpeed = 0x3028
        VtPixClkDiv = 0x302a
        VtSysClkDiv = 0x302c
        PrePllClkDiv = 0x302e
        PllMultiplier = 0x3030
        DigitalBinning = 0x3032
        FrameCount = 0x303a
        FrameStatus = 0x303c
        ReadMode = 0x3040
        DarkControl = 0x3044
        Flash = 0x3046
        Green1Gain = 0x3056
        BlueGain = 0x3058
        RedGain = 0x305a
        Green2Gain = 0x305c
        GlobalGain = 0x305e
        EmbeddedDataControl = 0x3064
        DatapathSelect = 0x306e
        TestPatternMode = 0x3070
        TestDataRed = 0x3072
        TestDataGreenr = 0x3074
        TestDataBlue = 0x3076
        TestDataGreenb = 0x3078
        TestRawMode = 0x307a
        SeqDataPort = 0x3086
        SeqCtrlPort = 0x3088
        XAddrStartCb = 0x308a
        YAddrStartCb = 0x308c
        XAddrEndCb = 0x308e
        YAddrEndCb = 0x3090
        XEvenInc = 0x30a0
        XOddInc = 0x30a2
        YEvenInc = 0x30a4
        YOddInc = 0x30a6
        YOddIncCb = 0x30a8
        FrameLengthLinesCb = 0x30aa
        FrameExposure = 0x30ac
        DigitalTest = 0x30b0
        TempsensData = 0x30b2
        TempsensCtrl = 0x30b4
        Green1GainCb = 0x30bc
        BlueGainCb = 0x30be
        RedGainCb = 0x30c0
        Green2GainCb = 0x30c2
        GlobalGainCb = 0x30c4
        TempsensCalib1 = 0x30c6
        TempsensCalib2 = 0x30c8
        TempsensCalib3 = 0x30ca
        TempsensCalib4 = 0x30cc
        ColumnCorrection = 0x30d4
        AeCtrlReg = 0x3100
        AeLumaTargetReg = 0x3102
        AeMinEvStepReg = 0x3108
        AeMaxEvStepReg = 0x310a
        AeDampOffsetReg = 0x310c
        AeDampGainReg = 0x310e
        AeDampMaxReg = 0x3110
        AeMaxExposureReg = 0x311c
        AeMinExposureReg = 0x311e
        AeDarkCurThreshReg = 0x3124
        AeCurrentGains = 0x312a
        AeRoiXStartOffset = 0x3140
        AeRoiYStartOffset = 0x3142
        AeRoiXSize = 0x3144
        AeRoiYSize = 0x3146
        AeMeanL = 0x3152
        AeCoarseIntegrationTime = 0x3164
        AeAgExposureHi = 0x3166
        AeAgExposureLo = 0x3168
        DeltaDkLevel = 0x3188
        HispiTiming = 0x31c0
        HispiControlStatus = 0x31c6
        HispiCrc0 = 0x31c8
        HispiCrc1 = 0x31ca
        HispiCrc2 = 0x31cc
        HispiCrc3 = 0x31ce
        StatFrameId = 0x31d2
        I2cWrtChecksum = 0x31d6
        HorizontalCursorPosition =0x31e8
        VerticalCursorPosition = 0x31ea
        HorizontalCursorWidth = 0x31ec
        VerticalCursorWidth = 0x31ee
        I2cIds = 0x31fc

    class ResetRegisterFlags(IntEnum):
        Reset = 0x0001
        Restart = 0x0002
        Stream = 0x0004
        LockReg = 0x0008
        StdbyEof = 0x0010
        DrivePins = 0x0040
        ParallelEn = 0x0080
        GpiEn = 0x0100
        MaskBad = 0x0200
        RestartBad = 0x0400
        ForcedPllOn = 0x0800
        SmiaSerialiserDis = 0x1000
        GroupedParameterHold =0x8000

    class GpiStatusFlags(IntEnum):
        Saddr = 0x0001
        OeN = 0x0002
        Trigger = 0x0004
        Standby = 0x0008

    class DigitalBinningFlags(IntEnum):
        DigitalBinningMask = 0x0003
        HorizontalOnlyBinning = 0x0001
        HorizontalAndVerticalBinning = 0x0002
        DigitalBinningCbMask = 0x0030
        HorizontalOnlyBinningCb = 0x0010
        HorizontalAndVerticalBinningCb = 0x0020

    class FrameStatusFlags(IntEnum):
        Framesync = 0x0001
        StandbyStatus = 0x0002

    class ReadModeFlags(IntEnum):
        HorizMirror = 0x4000
        VertFlip = 0x8000

    class DarkControlFlags(IntEnum):
        ShowDarkCols = 0x0200
        RowNoiseCorrectionEn = 0x0400
        ShowDarkExtraRows = 0x0800
        ShowColcorrRows = 0x1000

    class FlashFlags(IntEnum):
        InvertFlash = 0x0080
        EnFlash = 0x0100
        Triggered = 0x4000
        Strobe = 0x8000

    class EmbeddedDataFlags(IntEnum):
        EmbeddedStatsEn = 0x0080
        EmbeddedData = 0x0100

    class DatapathSelectFlags(IntEnum):
        SpecialLineValidMask = 0x0003
        LineValid = 0x0001
        LineValidXorFrameValid = 0x0002
        TrueBayer = 0x0010
        PostscalerDataSel = 0x0100
        SlewRateCtrlPixclkMask = 0x1c00
        SlewRateCtrlParallelMask = 0xe000

    class TestPatternModes(IntEnum):
        NormalOperation = 0
        SolidColor = 1
        ColorBar = 2
        FadeToGray = 3
        Walking1s = 256

    class DigitalTestFlags(IntEnum):
        ColGainMask = 0x0030
        MonoChrome = 0x0080
        ColGainCbMask = 0x0300
        EnableShortLlpck = 0x0400
        ContextB = 0x2000
        PllCompleteBypass = 0x4000

    class ColumnCorrectionFlags(IntEnum):
        ColcorrRowsMask = 0x000f
        DoubleSamples = 0x2000
        DoubleRange = 0x4000
        Enable = 0x8000

    class AeCtrlRegFlags(IntEnum):
        AeEnable = 0x0001
        AutoAgEn = 0x0002
        AutoDgEn = 0x0010
        MinAnaGainMask = 0x0060

    class AeCurrentGainsFlags(IntEnum):
        AeDigGainMask = 0x00ff
        AeAnaGainMask = 0x0300

    def __init__(self, esp770u):
        self.esp770u = esp770u

    def readRegister(self, register):
        return self.esp770u.readI2C(self.Addresses.I2CAddress, register)

    def writeRegister(self, register, value):
        return self.esp770u.writeI2C(self.Addresses.I2CAddress, register, value)

    def dumpRegisters(self):
        print('AR0134 registers:')
        print(f'\tCapture window: {self.readRegister(self.Registers.XAddrStart)}, {self.readRegister(self.Registers.YAddrStart)}, {self.readRegister(self.Registers.XAddrEnd)}, {self.readRegister(self.Registers.YAddrEnd)}')
        print(f'\tFrameLengthLines: {self.readRegister(self.Registers.FrameLengthLines)}')
        print(f'\tLineLengthPck: {self.readRegister(self.Registers.LineLengthPck)}')
        print(f'\tCoarseIntegrationTime: {self.readRegister(self.Registers.CoarseIntegrationTime)}')
        print(f'\tFineIntegrationTime: {self.readRegister(self.Registers.FineIntegrationTime)}')
        
        resetRegister = self.readRegister(self.Registers.ResetRegister)
        print(f'\tResetRegister: {hex(resetRegister)}')
        print(f'\t\tStream: {"on" if (resetRegister & self.ResetRegisterFlags.Stream != 0) else "off"}')
        print(f'\t\tLockReg: {"on" if (resetRegister & self.ResetRegisterFlags.LockReg != 0) else "off"}')
        print(f'\t\tStdbyEof: {"on" if (resetRegister & self.ResetRegisterFlags.StdbyEof != 0) else "off"}')
        print(f'\t\tDrivePins: {"on" if (resetRegister & self.ResetRegisterFlags.DrivePins != 0) else "off"}')
        print(f'\t\tParallelEn: {"on" if (resetRegister & self.ResetRegisterFlags.ParallelEn != 0) else "off"}')
        print(f'\t\tGpiEn: {"on" if (resetRegister & self.ResetRegisterFlags.GpiEn != 0) else "off"}')
        print(f'\t\tMaskBad: {"on" if (resetRegister & self.ResetRegisterFlags.MaskBad != 0) else "off"}')
        print(f'\t\tRestartBad: {"on" if (resetRegister & self.ResetRegisterFlags.RestartBad != 0) else "off"}')
        print(f'\t\tForcedPllOn: {"on" if (resetRegister & self.ResetRegisterFlags.ForcedPllOn != 0) else "off"}')
        print(f'\t\tSmiaSerialiserDis: {"on" if (resetRegister & self.ResetRegisterFlags.SmiaSerialiserDis != 0) else "off"}')
        print(f'\t\tGroupedParameterHold: {"on" if (resetRegister & self.ResetRegisterFlags.GroupedParameterHold != 0) else "off"}')
        
        print(f'\tDataPedestal: {self.readRegister(self.Registers.DataPedestal)}')
        print(f'\tTestPatternMode: {self.readRegister(self.Registers.TestPatternMode)}')

        gpiStatus = self.readRegister(self.Registers.GpiStatus)
        print(f'\tGpiStatus: {hex(gpiStatus)}')
        print(f'\t\tSaddr: {"on" if (gpiStatus & self.GpiStatusFlags.Saddr != 0) else "off"}')
        print(f'\t\tOeN: {"on" if (gpiStatus & self.GpiStatusFlags.OeN != 0) else "off"}')
        print(f'\t\tTrigger: {"on" if (gpiStatus & self.GpiStatusFlags.Trigger != 0) else "off"}')
        print(f'\t\tStandby: {"on" if (gpiStatus & self.GpiStatusFlags.Standby != 0) else "off"}')
        
        print(f'\tRowSpeed: {self.readRegister(self.Registers.RowSpeed)}')
        print(f'\tDigitalBinning: {self.readRegister(self.Registers.DigitalBinning)}')
        print(f'\tFrameCount: {self.readRegister(self.Registers.FrameCount)}')
        print(f'\tFrameStatus: {self.readRegister(self.Registers.FrameStatus)}')

        readMode = self.readRegister(self.Registers.ReadMode)
        print(f'\tReadMode: {hex(readMode)}')
        print(f'\t\tHorizontal mirroring: {"on" if (readMode & self.ReadModeFlags.HorizMirror != 0) else "off"}')
        print(f'\t\tVertical flipping: {"on" if (readMode & self.ReadModeFlags.VertFlip != 0) else "off"}')
        
        print(f'\tDarkControl: {self.readRegister(self.Registers.DarkControl)}')
        print(f'\tFlash: {self.readRegister(self.Registers.Flash)}')
        print(f'\tGlobalGain: {self.readRegister(self.Registers.GlobalGain)}')

        embeddedDataControl = self.readRegister(self.Registers.EmbeddedDataControl)
        print(f'\tEmbeddedDataControl: {hex(embeddedDataControl)}')
        print(f'\t\tEmbeddedStatsEn: {"on" if (embeddedDataControl & self.EmbeddedDataFlags.EmbeddedStatsEn != 0) else "off"}')
        print(f'\t\tEmbeddedData: {"on" if (embeddedDataControl & self.EmbeddedDataFlags.EmbeddedData != 0) else "off"}')
        
        print(f'\tDatapathSelect: {self.readRegister(self.Registers.DatapathSelect)}')

        digitalTest = self.readRegister(self.Registers.DigitalTest)
        print(f'\tDigitalTest: {hex(digitalTest)}')
        print(f'\t\tColGain: {(digitalTest & self.DigitalTestFlags.ColGainMask) >> 4}')
        print(f'\t\tMonoChrome: {"on" if (digitalTest & self.DigitalTestFlags.MonoChrome != 0) else "off"}')
        print(f'\t\tColGainCb: {(digitalTest & self.DigitalTestFlags.ColGainCbMask) >> 8}')
        print(f'\t\tEnableShortLlpck: {"on" if (digitalTest & self.DigitalTestFlags.EnableShortLlpck != 0) else "off"}')
        print(f'\t\tContextB: {"on" if (digitalTest & self.DigitalTestFlags.ContextB != 0) else "off"}')
        print(f'\t\tPllCompleteBypass: {"on" if (digitalTest & self.DigitalTestFlags.PllCompleteBypass != 0) else "off"}')
        
        print(f'\tTempsensData: {self.readRegister(self.Registers.TempsensData)}')
        print(f'\tColumnCorrection: {self.readRegister(self.Registers.ColumnCorrection)}')


        aeCtrlReg = self.readRegister(self.Registers.AeCtrlReg)
        print(f'\tAeCtrlReg: {hex(aeCtrlReg)}')
        print(f'\t\tAeEnable: {"on" if (aeCtrlReg & self.AeCtrlRegFlags.AeEnable != 0) else "off"}')
        print(f'\t\tAutoAgEn: {"on" if (aeCtrlReg & self.AeCtrlRegFlags.AutoAgEn != 0) else "off"}')
        print(f'\t\tAutoDgEn: {"on" if (aeCtrlReg & self.AeCtrlRegFlags.AutoDgEn != 0) else "off"}')
        print(f'\t\tMinAnaGain: {(aeCtrlReg & self.AeCtrlRegFlags.MinAnaGainMask)  >> 5}')
        
        print(f'\tAeLumaTargetReg: {self.readRegister(self.Registers.AeLumaTargetReg)}')
        print(f'\tAeMinEvStepReg: {self.readRegister(self.Registers.AeMinEvStepReg)}')
        print(f'\tAeMaxEvStepReg: {self.readRegister(self.Registers.AeMaxEvStepReg)}')
        print(f'\tAeMinExposureReg: {self.readRegister(self.Registers.AeMinExposureReg)}')
        print(f'\tAeMaxExposureReg: {self.readRegister(self.Registers.AeMaxExposureReg)}')


    def init(self):
        version = self.readRegister(self.Registers.ChipVersionReg)
        revision = self.readRegister(self.Registers.RevisionNumber)
        if version != 0x2406 or revision != 0x1300:
            raise Exception(f'Unsupported chip version {version}.{revision}')

        testMode = self.readRegister(self.Registers.DigitalTest)
        if testMode != self.DigitalTestFlags.MonoChrome:
            raise Exception('Unexpected camera mode')

        edc = self.readRegister(self.Registers.EmbeddedDataControl)
        self.writeRegister(self.Registers.EmbeddedDataControl, edc | self.EmbeddedDataFlags.EmbeddedStatsEn | self.EmbeddedDataFlags.EmbeddedData)

    def setTestPatternMode(self, mode):
        self.writeRegister(self.Registers.TestPatternMode, mode)

    def getHorizontalFlip(self):
        return (self.readRegister(self.Registers.ReadMode) & self.ReadModeFlags.HorizMirror) != 0x0
    
    def getVerticalFlip(self):
        return (self.readRegister(self.Registers.ReadMode) & self.ReadModeFlags.VertFlip) != 0x0

    def setFlip(self, hF, vF):
        readMode = self.readRegister(self.Registers.ReadMode)
        readMode &= ~(self.ReadModeFlags.HorizMirror | self.ReadModeFlags.VertFlip) & 0xffff
        if hF:
            readMode |= self.ReadModeFlags.HorizMirror
        if vF:
            readMode |= self.ReadModeFlags.VertFlip
        self.writeRegister(self.Registers.ReadMode, readMode)

    def getAutoExposure(self):
        return (self.readRegister(self.Registers.AeCtrlReg) & self.AeCtrlRegFlags.AeEnable) != 0x0

    def setAutoExposure(self, enable, adjustAnalogGain, adjustDigitalGain):
        ae = self.readRegister(self.Registers.AeCtrlReg)
        ae &= ~(self.AeCtrlRegFlags.AeEnable | self.AeCtrlRegFlags.AutoAgEn | self.AeCtrlRegFlags.AutoDgEn) & 0xffff
        if enable:
            ae |= self.AeCtrlRegFlags.AeEnable
        if adjustAnalogGain:
            ae |= self.AeCtrlRegFlags.AutoAgEn
        if adjustDigitalGain:
            ae |= self.AeCtrlRegFlags.AutoDgEn
        self.writeRegister(self.Registers.AeCtrlReg, ae)

    def getGain(self):
        return self.readRegister(self.Registers.GlobalGain)

    def setGain(self, gain):
        return self.writeRegister(self.Registers.GlobalGain, gain)

    def setWindow(self, x, y, w, h):
        self.writeRegister(self.Registers.XAddrStart, x)
        self.writeRegister(self.Registers.YAddrStart, y)
        self.writeRegister(self.Registers.XAddrEnd, x + w - 1)
        self.writeRegister(self.Registers.YAddrEnd, y + h - 1)

    def getTotalWidth(self):
        return self.readRegister(self.Registers.LineLengthPck)

    def getTotalHeight(self):
        return self.readRegister(self.Registers.FrameLengthLines)

    def setFrameTimings(self, minBlank):
        self.setWindow(0, 0, 1280, 960)
        self.writeRegister(self.Registers.LineLengthPck, 1280 + (108 if minBlank else 300))
        dt = self.readRegister(self.Registers.DigitalTest)
        if minBlank:
            dt |= self.DigitalTestFlags.EnableShortLlpck
        else:
            dt &= ~self.DigitalTestFlags.EnableShortLlpck & 0xffff
        self.writeRegister(self.Registers.DigitalTest, dt)
        self.writeRegister(self.Registers.FrameLengthLines, 960 + (23 if minBlank else 70))

    def getCoarseExposureTime(self):
        return self.readRegister(self.Registers.CoarseIntegrationTime)

    def setCoarseExposureTime(self, coarseExposure):
        return self.writeRegister(self.Registers.CoarseIntegrationTime, coarseExposure)

    def getFineExposureTime(self):
        return self.readRegister(self.Registers.FineIntegrationTime)

    def setFineExposureTime(self, fineExposure):
        return self.writeRegister(self.Registers.FineIntegrationTime, fineExposure)

    def getExposureTime(self):
        frameWidth = self.getTotalWidth()
        coarseExposure = self.getCoarseExposureTime()
        fineExposure = self.getFineExposureTime()
        return coarseExposure * frameWidth + fineExposure

    def setExposureTime(self, exposureTime):
        frameWidth = self.getTotalWidth()
        self.setCoarseExposureTime(int(exposureTime / frameWidth) & 0xffff)
        self.setFineExposureTime(int(exposureTime % frameWidth) & 0xffff)

    def setSync(self, enable):
        r = self.readRegister(self.Registers.ResetRegister)
        r &= ~(self.ResetRegisterFlags.Stream | self.ResetRegisterFlags.GpiEn | self.ResetRegisterFlags.ForcedPllOn) & 0xffff
        if enable:
            r |= self.ResetRegisterFlags.GpiEn | self.ResetRegisterFlags.ForcedPllOn
        else:
            r |= self.ResetRegisterFlags.Stream
        self.writeRegister(self.Registers.ResetRegister, r)





class TransferPool:

    def __init__(self, numTransfers=0, numPackets=0, packetSize=0):
        self.numTransfers = numTransfers
        self.numPackets = numPackets
        self.packetSize = packetSize
        self.transferSize = numPackets * packetSize
        self.transfers = []
        self.buffer = usb.util.create_buffer(self.numTransfers * self.transferSize)
        self.cancelled = False

        for t in range(numTransfers):
            self.transfers.append(libusb1._lib.libusb_alloc_transfer(numPackets))
    

    def submit(self, device, endpoint, callback):
        self.cancelled = False
        self.callback = callback
        baseAddr, _ = self.buffer.buffer_info()
        for t in range(self.numTransfers):
            libusb1._lib.libusb_fill_iso_transfer(self.transfers[t], device._ctx.handle.handle, endpoint, cast(baseAddr + t * self.transferSize, POINTER(c_ubyte)), self.transferSize, self.numPackets, libusb1._libusb_transfer_cb_fn_p(self._transferCallback), t, 0)
            libusb1._lib.libusb_set_iso_packet_lengths(self.transfers[t], self.packetSize)

        for t in range(self.numTransfers):
            libusb1._lib.libusb_submit_transfer(self.transfers[t])

    def cancel(self):
        for transfer in self.transfers:
            libusb1._lib.libusb_cancel_transfer(transfer)


    def _transferCallback(self, transfer):
        tI = transfer.contents.user_data
        if not self.cancelled:
            libusb1._lib.libusb_submit_transfer(transfer)
            if transfer.contents.status == 0 and self.callback is not None:
                transferAddr = tI * self.transferSize
                self.callback(transfer, tI, self.buffer[transferAddr:transferAddr+self.transferSize])



class CV1Sensor:

    def __init__(self, index=0, numTransfers=7):
        devices = list(usb.core.find(idVendor=0x2833, idProduct=0x0211, find_all=True))
        if index >= len(devices):
            raise Exception(f'Sensor at index={index} not found')
        self.device = devices[index]
        self.device.set_configuration()
        if self.device.is_kernel_driver_active(0):
            self.device.detach_kernel_driver(0)

        usb.util.claim_interface(self.device, 0)

        self.controller = ESP770U(self.device)
        self.controller.initController()

        calData = self.controller.readMemory(0x1d000, 128)
        fx = struct.unpack('f', calData[0x30:0x30+4])[0]
        fy = fx
        cx = struct.unpack('f', calData[0x34:0x34+4])[0]
        cy = struct.unpack('f', calData[0x38:0x38+4])[0]
        k = struct.unpack('ffff', calData[0x48:0x48+16])
        self.calData = (fx, fy, cx, cy, k)

        self.sensor = AR0134(self.controller)
        self.sensor.init()

        usb.util.claim_interface(self.device, 1)
        self.device.set_interface_altsetting(1, 2)

        uvcProbe = [0] * 26
        uvcProbe[2] = 1
        uvcProbe[3] = 4
        uvcProbe[4] = 200000 & 0xff
        uvcProbe[5] = 200000 >> 8 & 0xff
        uvcProbe[6] = 200000 >> 16 & 0xff
        uvcProbe[7] = 200000 >> 24 & 0xff
        dwMaxVideoFrameSize = 1280 * 960
        uvcProbe[18] = dwMaxVideoFrameSize & 0xff
        uvcProbe[19] = dwMaxVideoFrameSize >> 8 & 0xff
        uvcProbe[20] = dwMaxVideoFrameSize >> 16 & 0xff
        uvcProbe[21] = dwMaxVideoFrameSize >> 24 & 0xff
        uvcProbe[22] = 3072 & 0xff
        uvcProbe[23] = 3072 >> 8 & 0xff
        uvcProbe[24] = 3072 >> 16 & 0xff
        uvcProbe[25] = 3072 >> 24 & 0xff

        setCur(self.device, 1, 0, 1, uvcProbe)
        uvcProbe = getCur(self.device, 1, 0, 1, 26)
        # print(struct.unpack('HBBIHHHHH', bytes(uvcProbe)[:-8]) + struct.unpack('II', bytes(uvcProbe)[-8:]))
        setCur(self.device, 1, 0, 2, uvcProbe)

        self._pool = TransferPool(numTransfers=numTransfers, numPackets=24, packetSize=9000)
        self._frameId = 0
        self._frameSize = 1280*960
        self._frameBuf = np.zeros(shape=(self._frameSize,), dtype=np.uint8)
        self._frameBufRemainder = 0
        self._frameDataPtr = 0
        self._frame = np.zeros(shape=(960, 1280), dtype=np.uint8)
        self._newFrame = False

    def serialNumber(self):
        return self.device.serial_number

    def start(self):
        self._pool.submit(self.device, 0x81, self._isoTransferCallback)

    def stop(self):
        self._pool.cancel()
        self.device.reset()

    def hasNewFrame(self):
        libusb1._lib.libusb_handle_events(self.device._ctx.backend.ctx)
        return self._newFrame

    def latestFrame(self):
        self._newFrame = False
        return self._frame

    def _isoTransferCallback(self, transfer, tI, transferBuf):
        desc = transfer.contents.iso_packet_desc
        isoPackets = libusb1._get_iso_packet_list(transfer.contents)
        
        packetBaseAddr = 0
        for packet in isoPackets:
            packetLen = packet.actual_length
            packetBuf = transferBuf[packetBaseAddr:packetBaseAddr+packetLen]
            if packet.status != 0 or packetLen < 12:
                continue

            if packetBuf[0] == 12 and (packetBuf[1] & 0x40) == 0x00:
                isoFrameId = packetBuf[1] & 0x01
        
                if isoFrameId != self._frameId:
                    self._frameId = isoFrameId
                    if self._frameBufRemainder == 0:
                        frame = self._frameBuf.copy()
                        frame.shape = (960,1280)
                        self._frame = frame
                        self._newFrame = True
                    self._frameBufRemainder = self._frameSize
                    self._frameDataPtr = 0
                
                isoFrameDataLength = packetLen - 12
                if isoFrameDataLength <= self._frameBufRemainder and self._frameBufRemainder > 0:
                    self._frameBuf[self._frameDataPtr:self._frameDataPtr+isoFrameDataLength] = packetBuf[12:]
                    self._frameDataPtr += isoFrameDataLength
                    self._frameBufRemainder -= isoFrameDataLength
            
            packetBaseAddr += self._pool.packetSize










