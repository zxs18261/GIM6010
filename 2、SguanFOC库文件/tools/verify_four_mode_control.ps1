param(
    [Parameter(Mandatory = $false)]
    [string]$ProjectRoot
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($ProjectRoot)) {
    $ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
}

function Assert-Match {
    param([string]$Name, [string]$Text, [string]$Pattern)
    if ($Text -notmatch $Pattern) { throw "FAIL: $Name" }
    Write-Output "PASS: $Name"
}

function Assert-NoMatch {
    param([string]$Name, [string]$Text, [string]$Pattern)
    if ($Text -match $Pattern) { throw "FAIL: $Name" }
    Write-Output "PASS: $Name"
}

function Assert-Count {
    param([string]$Name, [string]$Text, [string]$Pattern, [int]$Expected)
    $count = [regex]::Matches($Text, $Pattern).Count
    if ($count -ne $Expected) { throw "FAIL: $Name (expected $Expected, got $count)" }
    Write-Output "PASS: $Name"
}

$libRoot = Join-Path $ProjectRoot 'SguanFOC v3.0.1'
$control = Get-Content -Raw -LiteralPath (Join-Path $libRoot 'UserData_UserControl.h')
$parser = Get-Content -Raw -LiteralPath (Join-Path $libRoot 'Sguan_printf.c')
$parserHeader = Get-Content -Raw -LiteralPath (Join-Path $libRoot 'Sguan_printf.h')
$core = Get-Content -Raw -LiteralPath (Join-Path $libRoot 'SguanFOC.c')
$coreHeader = Get-Content -Raw -LiteralPath (Join-Path $libRoot 'SguanFOC.h')
$config = Get-Content -Raw -LiteralPath (Join-Path $libRoot 'UserData_Config.h')
$motorStatus = Get-Content -Raw -LiteralPath (Join-Path $libRoot 'Sguan_MotorStatus.c')
$can = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'APP\canfd.c')
$callbacks = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'APP\app_callbacks.c')
$functions = Get-Content -Raw -LiteralPath (Join-Path $libRoot 'UserData_Function.h')
$fdcan = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'Core\Src\fdcan.c')
$irq = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'Core\Src\stm32g4xx_it.c')
$irqHeader = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'Core\Inc\stm32g4xx_it.h')
$cmdFiles = @(Get-ChildItem -LiteralPath $ProjectRoot -Recurse -File |
    Where-Object { $_.Name -ceq 'vofa+.cmds.json' })
if ($cmdFiles.Count -ne 1) {
    throw "FAIL: expected exactly one vofa+.cmds.json under ProjectRoot, got $($cmdFiles.Count)"
}
$cmdJson = Get-Content -Raw -Encoding UTF8 -LiteralPath $cmdFiles[0].FullName | ConvertFrom-Json
$parserResetMatch = [regex]::Match(
    $parser,
    '(?s)void\s+Sguan_Printf_ResetReceive\s*\(void\)\s*\{.*?\r?\n\}'
)
if (-not $parserResetMatch.Success) {
    throw 'FAIL: UART parser reset function extraction'
}
$parserReset = $parserResetMatch.Value

Assert-Match 'strict framed decimal parser' $parser '(?s)Parse_Command.*cmd\[2\]\s*!=\s*''=''.*cmd\[length\s*-\s*1U\]\s*!=\s*''\?'''
Assert-Match 'canonical BO command is decoded from its raw byte' $parser '(?s)if\s*\(\(cmd\[0\]\s*==\s*''B''\)\s*&&\s*\(cmd\[1\]\s*==\s*''O''\)\).*?length\s*!=\s*5U.*?cmd\[3\]\s*<\s*''0''.*?cmd\[3\]\s*>\s*''3''.*?User_BO_Adjust\s*\(\(float\)\(cmd\[3\]\s*-\s*''0''\)\)'
Assert-Match 'canonical CO command is decoded from its raw byte' $parser '(?s)if\s*\(\(cmd\[0\]\s*==\s*''C''\)\s*&&\s*\(cmd\[1\]\s*==\s*''O''\)\).*?length\s*!=\s*5U.*?cmd\[3\]\s*<\s*''0''.*?cmd\[3\]\s*>\s*''1''.*?User_CO_Adjust\s*\(\(float\)\(cmd\[3\]\s*-\s*''0''\)\)'
Assert-Match 'continuous AO and VO commands are routed' $parser '(?s)User_AO_Adjust\s*\(value\).*?User_VO_Adjust\s*\(value\)'
Assert-Match 'DMA fragments and multiple commands accumulate' $parser '(?s)s_command_length.*Printf_Receive'
Assert-Match 'oversized UART frame discards through terminator' $parser '(?s)s_command_discard.*if \(s_command_discard != 0U\).*ch == ''\?''.*s_command_discard = 1U'
Assert-Match 'UART receive reset has public prototype' $parserHeader 'void\s+Sguan_Printf_ResetReceive\s*\(void\)\s*;'
Assert-Match 'UART reset clears parser fragment state' $parserReset '(?s)s_command_length\s*=\s*0U.*?s_command_discard\s*=\s*0U.*?memset\s*\(s_command\s*,\s*0\s*,\s*sizeof\(s_command\)\)'
Assert-NoMatch 'UART parser module never clears the active DMA destination' $parser 'memset\s*\(\s*Sguan_PrintfBuff'

Assert-Match 'four exact modes selectable' $control '(?s)BO\s*<\s*0\.0f.*BO\s*>\s*3\.0f.*BO\s*!='
Assert-Match 'CO accepts only exact binary values' $control '(?s)User_CO_Adjust\s*\(float\s+CO\).*?Value_isnan\(CO\).*?Value_isinf\(CO\).*?CO\s*!=\s*0\.0f.*?CO\s*!=\s*1\.0f.*?if\s*\(CO\s*==\s*0\.0f\)'
Assert-Count 'invalid internal modes fail closed in float and Q31 paths' $core '(?s)else\s*\{\s*(?:/\*.*?\*/\s*)?sguan->foc\.U[qd]_in(?:_q31)?\s*=\s*(?:0\.0f|0);.*?sguan->foc\.U[qd]_in(?:_q31)?\s*=\s*(?:0\.0f|0);.*?sguan->status\s*=\s*MOTOR_STATUS_PWM_CALC_FAULT;.*?User_PwmOutputDisable\(\);\s*return;' 2
Assert-Match 'mode switch physically disables PWM' $control '(?s)static\s+inline\s+void\s+User_BO_Adjust\(float\s+BO\)\s*\{.*User_PwmOutputDisable\(\).*User_ClearMotionCommand\(\).*Sguan\.mode\s*=\s*next_mode'
Assert-Match 'open-loop speed command is independent' $control '(?s)User_VO_Adjust.*Velocity_OPEN_MODE.*Target_Speed'
Assert-NoMatch 'open-loop has no hidden default speed' $control 'USER_OPENLOOP_DEF_SHAFT_SPEED|User_SetOpenLoopDefaultSpeed|prev_mode'
Assert-Match 'CO enable clears targets before any restart' $control '(?s)User_CO_Adjust\s*\(float\s+CO\).*?User_ClearMotionCommand\(\).*?User_ResetControllerState\(\).*?if\s*\(restart_required\s*!=\s*0U\).*?g_foc_fault_snapshot\.valid\s*=\s*0U'
Assert-Match 'torque target is bounded current' $control '(?s)Current_SINGLE_MODE.*GIM6010_RATED_CURRENT_A.*Target_Iq'
Assert-Match 'speed target uses shaft-to-motor conversion' $control '(?s)VelCur_DOUBLE_MODE.*User_LimitShaftSpeed\(AO\).*GIM6010_GEAR_RATIO'
Assert-Match 'position target is finite-bounded' $control '(?s)PosVelCur_THREE_MODE.*USER_POSITION_ABS_MAX_RAD.*Target_Pos'
Assert-Match 'motion start rechecks IDLE atomically' $control '(?s)User_StartMotionFromIdle.*__disable_irq\(\).*Sguan\.status != MOTOR_STATUS_IDLE.*User_PwmOutputEnable\(\).*__enable_irq\(\)'
Assert-Match 'STOP preserves a latched hardware fault' $control '(?s)if\s*\(CO\s*==\s*0\.0f\).*User_PwmOutputDisable\(\).*__disable_irq\(\).*Sguan\.status < MOTOR_STATUS_OVERVOLTAGE.*MOTOR_STATUS_STANDBY'
Assert-Match 'CO restart uses status snapshot recheck' $control '(?s)status_snapshot = Sguan\.status.*User_FaultClearAllowed\(\).*__disable_irq\(\).*Sguan\.status != status_snapshot'

Assert-Match 'PLL position stays continuous across modes' $core '(?s)static\s+void\s+Transfer_PLL_Loop\(PLL_STRUCT\s*\*pll.*pll->is_position_mode\s*=\s*1U.*while \(pll->go.Error'
Assert-Match 'position loop counter reaches Response squared' $core '(?s)Control_PosVelCur_THREE\(.*?static uint16_t Control_Count.*?Response\*sguan->control.Response'
Assert-Match 'encoder zero averaging unwraps the 2pi boundary' $core '(?s)Offset_EncoderRead\(.*reference.*delta.*Value_PI.*Value_2PI.*Value_normalize'
Assert-Match 'repeat current calibration rebuilds offsets locally' $core '(?s)Offset_CurrentRead\(.*?int32_t\s+sum0\s*=\s*0.*?int32_t\s+sum1\s*=\s*0.*?s_current_offset_valid\s*=\s*0U.*?Current_offset0\s*=\s*sum0/24.*?Current_offset1\s*=\s*sum1/24'
Assert-Match 'run-state display update cannot overwrite ISR fault' $core '(?s)next_status = sguan->status.*__disable_irq\(\).*sguan->status > MOTOR_STATUS_IDLE.*sguan->status = next_status'
Assert-Match 'hardware faults remain latched' $core '(?s)static\s+void\s+Status_RUN_Loop\(SguanFOC_System_STRUCT\s*\*sguan\).*disabled_count.*if \(sguan->status == MOTOR_STATUS_DISABLED\).*else\s*\{\s*disabled_count = 0U;\s*\}.*MotorStatus_Loop'
Assert-Match 'invalid status fails into emergency stop' $motorStatus '(?s)else\s*\{.*MOTOR_STATUS_EMERGENCY_STOP.*status_handlers\[MOTOR_STATUS_EMERGENCY_STOP\]'
Assert-Match 'shared motor status is volatile' $coreHeader 'volatile\s+uint8_t\s+status\s*;'
Assert-NoMatch 'active core has no legacy undefined control macros' $core '(?m)^\s*#if\s+!?\s*(?:Printf_Debug|MOTOR_CONTROL)\s*$'
Assert-Match 'four-mode telemetry build rejects legacy printf debug mode' $config '(?s)#define\s+Open_Printf_Debug\s+0(?:\s|$).*?#if\s+Open_Printf_Debug.*?#error\s+"[^"]+"'
Assert-Match 'power stage stays off through preflight' $core '(?s)Sguan_PreflightFaultStatus.*User_InitialInit\(\).*preflight_fault.*User_PwmOutputEnable\(\).*Sguan_Positioning_Set'
Assert-Match 'high loop rejects stale encoder' $core '(?s)SguanFOC_High_Loop.*BSP_MT6816_IsSampleFresh\(\).*MOTOR_STATUS_ENCODER_ERROR'
Assert-Match 'open-loop angle state has one global definition' $core 'float\s+g_user_openloop_eangle\s*=\s*0\.0f\s*;'
Assert-Match 'open-loop angle state is extern at call sites' $control 'extern\s+float\s+g_user_openloop_eangle\s*;'

Assert-Match 'FDCAN has at least one standard filter slot' $fdcan 'StdFiltersNbr\s*=\s*[1-9][0-9]*\s*;'
Assert-Match 'FDCAN nominal timing is 1 Mbps before HAL init' $fdcan '(?s)NominalPrescaler\s*=\s*10\s*;.*NominalSyncJumpWidth\s*=\s*1\s*;.*NominalTimeSeg1\s*=\s*12\s*;.*NominalTimeSeg2\s*=\s*4\s*;.*HAL_FDCAN_Init'
Assert-Match 'FDCAN RX IRQ is enabled' $fdcan '(?s)HAL_NVIC_SetPriority\(FDCAN1_IT0_IRQn,\s*3,\s*0\).*HAL_NVIC_EnableIRQ\(FDCAN1_IT0_IRQn\)'
Assert-Match 'FDCAN IRQ is declared' $irqHeader 'void\s+FDCAN1_IT0_IRQHandler\s*\(void\)\s*;'
Assert-Match 'FDCAN IRQ dispatches into HAL' $irq '(?s)void\s+FDCAN1_IT0_IRQHandler\s*\(void\).*HAL_FDCAN_IRQHandler\(&hfdcan1\)'
Assert-Match 'CAN validates complete classic data frame before enqueue' $can '(?s)header\.Identifier != CANFD_ID_CMD.*header\.IdType != FDCAN_STANDARD_ID.*header\.RxFrameType != FDCAN_DATA_FRAME.*header\.FDFormat != FDCAN_CLASSIC_CAN.*header\.DataLength != FDCAN_DLC_BYTES_8.*memcpy\(s_rx_queue'
Assert-Match 'CAN disable is independently latched in ISR' $can '(?s)cmd & CANFD_CMD_DIS.*User_PwmOutputDisable\(\).*s_stop_sequence\+\+.*continue;'
Assert-Match 'CAN queue overflow is fail-safe' $can '(?s)next == s_rx_tail.*s_rx_drop_count\+\+.*User_PwmOutputDisable\(\).*s_stop_sequence\+\+'
Assert-Match 'blocking alignment polls CAN stop latch' $functions 'CANFD_BlockingSafetyPoll\(\)'
Assert-Match 'CAN reserved control bits are rejected' $can 'cmd & CANFD_CMD_RESERVED_MASK'
Assert-Match 'CAN exposes torque target' $can '(?s)Current_SINGLE_MODE.*target_i32.*CANFD_MILLI_PER_UNIT'
Assert-Match 'CAN continuous position conversion is finite checked and saturated' $can '(?s)CANFD_EncodePositionUrad\s*\(float\s+shaft_rad\).*?Value_isnan\(shaft_rad\).*?Value_isinf\(shaft_rad\).*?\(double\)shaft_rad.*?INT32_MAX.*?INT32_MIN.*?return\s+\(int32_t\)scaled'
Assert-Match 'CAN RPM conversion is finite checked and saturated' $can '(?s)CANFD_EncodeRpm\s*\(float\s+shaft_rpm\).*?Value_isnan\(shaft_rpm\).*?Value_isinf\(shaft_rpm\).*?INT16_MAX.*?INT16_MIN.*?return\s+\(int16_t\)shaft_rpm'
Assert-Match 'CAN telemetry uses checked narrowing helpers' $can '(?s)pos_urad\s*=\s*CANFD_EncodePositionUrad\(shaft_rad\).*?rpm\s*=\s*CANFD_EncodeRpm\(shaft_rpm\)'

Assert-Match 'UART error ISR immediately disables PWM and latches stop' $callbacks '(?s)HAL_UART_ErrorCallback.*User_PwmOutputDisable\(\).*s_uart_stop_sequence\+\+.*s_uart_rx_restart_needed = 1U'
Assert-Match 'UART queue overflow is fail-safe' $callbacks '(?s)g_uart_rx_drop_cnt\+\+.*User_PwmOutputDisable\(\).*s_uart_stop_sequence\+\+'
Assert-Match 'UART restart failure keeps retry request set' $callbacks '(?s)APP_UartRxStart\(void\).*s_uart_rx_restart_needed = 0U.*HAL_UARTEx_ReceiveToIdle_DMA.*!= HAL_OK.*s_uart_rx_restart_needed = 1U'
Assert-Match 'UART stop service flushes queue and parser before stopping' $callbacks '(?s)APP_UartServiceStop\s*\(void\).*?s_uart_rx_tail\s*=\s*s_uart_rx_head.*?Sguan_Printf_ResetReceive\(\).*?User_CO_Adjust\(0\.0f\).*?s_uart_stop_serviced\s*=\s*stop_sequence'
Assert-Match 'UART drain rechecks stop around command parsing' $callbacks '(?s)while\s*\(s_uart_rx_tail\s*!=\s*s_uart_rx_head\).*?APP_UartStopPending\(\).*?APP_UartServiceStop\(\).*?break;.*?__DMB\(\).*?APP_UartStopPending\(\).*?APP_UartServiceStop\(\).*?break;.*?SguanFOC_Printf_Loop.*?APP_UartStopPending\(\).*?APP_UartServiceStop\(\).*?break;'
Assert-Match 'UART abort and rearm run in main poll' $callbacks '(?s)APP_UartCommandPoll\(void\).*HAL_UART_AbortReceive.*Sguan_Printf_ResetReceive\(\).*APP_UartRxStart\(\)'
Assert-Count 'UART abort occurs only in the main-poll recovery path' $callbacks 'HAL_UART_AbortReceive' 1
Assert-Match 'UART DMA TX uses stable double buffer' $functions '(?s)s_tx_buffer\[2\].*memcpy\(s_tx_buffer\[s_tx_index\].*HAL_UART_Transmit_DMA.*s_tx_index \^= 1U'

if ($cmdJson.ctx.subCmds.Count -lt 8) { throw 'FAIL: VOFA command panel entries' }
Write-Output 'PASS: VOFA command panel entries'
Write-Output 'Four-mode static control checks passed.'
