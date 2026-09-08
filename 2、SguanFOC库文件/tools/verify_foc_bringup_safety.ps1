param(
    [Parameter(Mandatory = $false)]
    [string]$ProjectRoot
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($ProjectRoot)) {
    $ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
}

function Assert-Match {
    param(
        [string]$Name,
        [string]$Text,
        [string]$Pattern
    )
    if ($Text -notmatch $Pattern) {
        throw "FAIL: $Name"
    }
    Write-Output "PASS: $Name"
}

function Assert-NoMatch {
    param(
        [string]$Name,
        [string]$Text,
        [string]$Pattern
    )
    if ($Text -match $Pattern) {
        throw "FAIL: $Name"
    }
    Write-Output "PASS: $Name"
}

function Assert-Count {
    param(
        [string]$Name,
        [string]$Text,
        [string]$Pattern,
        [int]$Expected
    )
    $count = [regex]::Matches($Text, $Pattern).Count
    if ($count -ne $Expected) {
        throw "FAIL: $Name (expected $Expected, got $count)"
    }
    Write-Output "PASS: $Name"
}

$configPath = Join-Path $ProjectRoot 'SguanFOC v3.0.1\UserData_Config.h'
$functionPath = Join-Path $ProjectRoot 'SguanFOC v3.0.1\UserData_Function.h'
$telemetryPath = Join-Path $ProjectRoot 'SguanFOC v3.0.1\UserData_UserControl.h'
$corePath = Join-Path $ProjectRoot 'SguanFOC v3.0.1\SguanFOC.c'
$callbacksPath = Join-Path $ProjectRoot 'APP\app_callbacks.c'
$adcPath = Join-Path $ProjectRoot 'BSP\BSP_ADC.c'
$adcHeaderPath = Join-Path $ProjectRoot 'BSP\BSP_ADC.h'
$motorPath = Join-Path $ProjectRoot 'SguanFOC v3.0.1\UserData_Motor.h'
$irqPath = Join-Path $ProjectRoot 'Core\Src\stm32g4xx_it.c'
$mainPath = Join-Path $ProjectRoot 'Core\Src\main.c'

$config = Get-Content -Raw -LiteralPath $configPath
$functions = Get-Content -Raw -LiteralPath $functionPath
$telemetry = Get-Content -Raw -LiteralPath $telemetryPath
$core = Get-Content -Raw -LiteralPath $corePath
$callbacks = Get-Content -Raw -LiteralPath $callbacksPath
$adc = Get-Content -Raw -LiteralPath $adcPath
$adcHeader = Get-Content -Raw -LiteralPath $adcHeaderPath
$motor = Get-Content -Raw -LiteralPath $motorPath
$irq = Get-Content -Raw -LiteralPath $irqPath
$main = Get-Content -Raw -LiteralPath $mainPath
$faultCleanupMatch = [regex]::Match(
    $core,
    '(?s)if\s*\(Sguan\.status\s*>=\s*MOTOR_STATUS_OVERVOLTAGE\)\s*\{.*?fault_status\s*=\s*Sguan\.status;\s*\r?\n\s*\}'
)
if (-not $faultCleanupMatch.Success) {
    throw 'FAIL: FOC fault cleanup block extraction'
}
$faultCleanup = $faultCleanupMatch.Value
$vbusOverrangeMatch = [regex]::Match(
    $adcHeader,
    '#define\s+BSP_ADC_VBUS_OVERRANGE_V\s+\((?<value>[0-9]+(?:\.[0-9]+)?)f\)'
)
$vbusSaturationMatch = [regex]::Match(
    $adcHeader,
    '#define\s+BSP_ADC_VBUS_SAT_RAW\s+\((?<value>[0-9]+)U\)'
)
$adcFullScaleMatch = [regex]::Match(
    $adcHeader,
    '#define\s+BSP_ADC_FULL_SCALE\s+\((?<value>[0-9]+(?:\.[0-9]+)?)f\)'
)
$vbusMaxMatch = [regex]::Match(
    $motor,
    'Sguan\.safe\.VBUS_MAX\s*=\s*(?<value>[0-9]+(?:\.[0-9]+)?)f\s*;'
)
if ((-not $vbusOverrangeMatch.Success) -or (-not $vbusSaturationMatch.Success) -or
    (-not $adcFullScaleMatch.Success) -or (-not $vbusMaxMatch.Success)) {
    throw 'FAIL: VBUS overrange/range/threshold extraction'
}
$invariantCulture = [System.Globalization.CultureInfo]::InvariantCulture
$vbusOverrange = [double]::Parse($vbusOverrangeMatch.Groups['value'].Value, $invariantCulture)
$vbusSaturation = [double]::Parse($vbusSaturationMatch.Groups['value'].Value, $invariantCulture)
$adcFullScale = [double]::Parse($adcFullScaleMatch.Groups['value'].Value, $invariantCulture)
$vbusMax = [double]::Parse($vbusMaxMatch.Groups['value'].Value, $invariantCulture)
if ($vbusOverrange -le $vbusMax) {
    throw "FAIL: VBUS ADC overrange sentinel must exceed configured maximum ($vbusOverrange <= $vbusMax)"
}
Write-Output 'PASS: VBUS ADC overrange sentinel exceeds configured maximum'
if (($adcFullScale - $vbusSaturation) -lt 64.0) {
    throw "FAIL: VBUS ADC saturation margin is too small ($adcFullScale - $vbusSaturation < 64 codes)"
}
Write-Output 'PASS: VBUS ADC saturation threshold retains fail-closed margin'

Assert-Match 'current feedforward disabled' $config '#define\s+USER_CURRENT_FEEDFORWARD_ENABLE\s+0(?:\s|$)'
Assert-Match 'closed-loop DQ limit is 0.30 V' $config '#define\s+USER_CLOSED_LOOP_DQ_VOLTAGE_LIMIT_V\s+0\.30f'
Assert-Match 'ADC high-loop timeout is configured' $config '#define\s+USER_ADC_HIGH_LOOP_TIMEOUT_MS\s+\([1-9][0-9]*U\)'
Assert-Match 'three consecutive FOC deadline misses are configured' $config '#define\s+USER_FOC_DEADLINE_MISS_LIMIT\s+\(3U\)'
Assert-Match 'unreviewed Q31 integration is compile-time disabled' $config '(?s)#define\s+Open_Q31_Calculate\s+0(?:\s|$).*?#if\s+Open_Q31_Calculate.*?#error\s+"[^"]+"'
Assert-Match 'alignment has an independent current limit' $config '#define\s+USER_ALIGN_CURRENT_LIMIT_A\s+[0-9]+(?:\.[0-9]+)?f'
Assert-Match 'VBUS near-full-scale ADC input fails closed' $adc '(?s)BSP_ADC_VbusRawToVolts\s*\(uint32_t\s+raw\).*?raw\s*>=\s*BSP_ADC_VBUS_SAT_RAW.*?return\s+BSP_ADC_VBUS_OVERRANGE_V.*?return\s+\(float\)raw\s*\*\s*BSP_ADC_VBUS_PER_LSB'
Assert-Count 'fast and slow VBUS paths share saturated conversion' $adc 's_vbus_(?:fast|slow)_v\s*=\s*BSP_ADC_VbusRawToVolts\(' 2
Assert-Match 'current channel 0 reads IV/Rank1' $functions '(?s)case\s+0\s*:.*?ADC_InjectedValues\[0\].*?break\s*;'
Assert-Match 'current channel 1 reads IW/Rank2' $functions '(?s)case\s+1\s*:.*?ADC_InjectedValues\[1\].*?break\s*;'
Assert-NoMatch 'ordinary conditional MOE disable is not used' $functions '__HAL_TIM_MOE_DISABLE\s*\('
Assert-NoMatch 'power PWM HAL start cannot auto-enable MOE during init' $functions '(?m)^\s*(?:HAL_TIM_PWM_Start|HAL_TIMEx_PWMN_Start)\s*\('
Assert-Match 'ADC is safely stopped calibrated armed then triggered' $functions '(?s)uint8_t\s+User_InitialInit\s*\(void\).*?CLEAR_BIT\(htim1\.Instance->CR1,\s*TIM_CR1_CEN\).*?HAL_ADCEx_InjectedStop_IT.*?!=\s*HAL_OK.*?HAL_ADC_Stop.*?!=\s*HAL_OK.*?LL_ADC_REG_IsConversionOngoing.*?LL_ADC_INJ_IsConversionOngoing.*?LL_ADC_IsEnabled.*?HAL_ADCEx_Calibration_Start.*?!=\s*HAL_OK.*?HAL_ADCEx_InjectedStart_IT.*?!=\s*HAL_OK.*?__HAL_TIM_ENABLE\(&htim1\)'
Assert-Match 'ADC init failure is latched as sensor fault' $core '(?s)User_InitialInit\(\)\s*==\s*0U.*?User_PwmOutputDisable\(\).*?MOTOR_STATUS_SENSOR_ERROR'
Assert-Match 'offsets publish atomically from local sums' $core '(?s)Offset_CurrentRead\(.*?int32_t\s+sum0\s*=\s*0.*?s_current_offset_valid\s*=\s*0U.*?status\s*!=\s*MOTOR_STATUS_CALIBRATING.*?sum0\s*\+=.*?__disable_irq\(\).*?Current_offset0\s*=\s*sum0/24.*?s_current_offset_valid\s*=\s*1U'
Assert-Match 'alignment checks all three phase currents' $core '(?s)Current_AlignmentOverLimit\(.*?Real_Ia.*?Real_Ib.*?Real_Ic.*?SguanFOC_High_Loop.*?MOTOR_STATUS_CALIBRATING.*?s_current_offset_valid.*?Current_AlignmentOverLimit.*?MOTOR_STATUS_OVERCURRENT.*?User_PwmOutputDisable'
Assert-Count 'float and Q31 normal overcurrent checks are run-state gated' $core 'status\s*>\s*MOTOR_STATUS_IDLE\)\s*&&\s*\n?\s*\(sguan->status\s*<\s*MOTOR_STATUS_OVERVOLTAGE\)\s*&&\s*\n?\s*\(\(sguan->current\.Real_Id' 2
Assert-Match 'fault cleanup clears stale float dq feedback' $core '(?s)g_foc_fault_snapshot\.id\s*=\s*Sguan\.current\.Real_Id.*?Sguan\.current\.Real_Id\s*=\s*0\.0f.*?Sguan\.current\.Real_Iq\s*=\s*0\.0f'
Assert-Match 'fault cleanup clears stale Q31 dq feedback' $core '(?s)Sguan\.current\.Real_Id_q31\s*=\s*0;.*?Sguan\.current\.Real_Iq_q31\s*=\s*0;'
Assert-Match 'fault cleanup directly zeros duty and all CCR values' $faultCleanup '(?s)Sguan\.foc\.Duty_u\s*=\s*0U.*?Sguan\.foc\.Duty_v\s*=\s*0U.*?Sguan\.foc\.Duty_w\s*=\s*0U.*?User_PwmDuty_Set\(0U,\s*0U,\s*0U\)'
Assert-NoMatch 'fault cleanup never divides by VBUS or reruns SVPWM' $faultCleanup 'SVPWM_Tick|/\s*Sguan\.motor\.VBUS|IQmath_Q31_div'
Assert-Match 'ADC heartbeat fail-safe is active from calibration onward' $callbacks '(?s)APP_CheckAdcHighLoopHeartbeat.*?g_high_loop_cnt.*?USER_ADC_HIGH_LOOP_TIMEOUT_MS.*?MOTOR_STATUS_SENSOR_ERROR.*?User_PwmOutputDisable.*?HAL_TIM_PeriodElapsedCallback.*?APP_CheckAdcHighLoopHeartbeat\(\)'
Assert-Match 'each fast VBUS overrange sample immediately latches fault and disables output' $callbacks '(?s)BSP_ADC_FeedVBUSFast\(ADC_InjectedValues\[2\]\).*?ADC_InjectedValues\[2\]\s*>=\s*BSP_ADC_VBUS_SAT_RAW.*?User_PwmOutputDisable\(\).*?Sguan\.status\s*=\s*MOTOR_STATUS_OVERVOLTAGE.*?SguanFOC_High_Loop\(\)'
Assert-Match 'three active-state deadline misses latch PWM fault and disable output' $callbacks '(?s)used_cycles\s*>=\s*deadline_cycles.*?g_high_loop_deadline_miss_cnt\+\+.*?status\s*>=\s*MOTOR_STATUS_CALIBRATING.*?status\s*<\s*MOTOR_STATUS_OVERVOLTAGE.*?s_high_loop_deadline_miss_streak\+\+.*?s_high_loop_deadline_miss_streak\s*>=\s*USER_FOC_DEADLINE_MISS_LIMIT.*?MOTOR_STATUS_PWM_CALC_FAULT.*?User_PwmOutputDisable\(\).*?s_high_loop_deadline_miss_streak\s*=\s*0U'
Assert-Match 'legacy PWM watchdog faults and disables at its threshold' $core '(?s)PWM_watchdog_counter\+\+.*?PWM_watchdog_counter\s*>=\s*Sguan\.flag\.PWM_watchdog_limit.*?MOTOR_STATUS_PWM_CALC_FAULT.*?User_PwmOutputDisable\(\).*?PWM_watchdog_counter\s*=\s*0'
Assert-Match 'ADC HAL error callback fails safe' $callbacks '(?s)HAL_ADC_ErrorCallback.*?ADC1.*?APP_LatchAdcSensorFault'
Assert-Match 'ADC injected queue overflow fails safe' $callbacks '(?s)HAL_ADCEx_InjectedQueueOverflowCallback.*?ADC1.*?APP_LatchAdcSensorFault'
Assert-Match 'live current0 telemetry follows IV' $telemetry 'current0_live\s*=.*ADC_InjectedValues\[0\]'
Assert-Match 'live current1 telemetry follows IW' $telemetry 'current1_live\s*=.*ADC_InjectedValues\[1\]'
Assert-Match 'non-finite DQ commands are cleared' $core 'Value_isnan\(sguan->foc\.Ud_in\).*Value_isinf\(sguan->foc\.Ud_in\)'
Assert-Match 'final closed-loop limiter runs before SVPWM' $core '(?s)Limit_DqVoltageForBringup\(sguan\);\s*}\s*SVPWM_Tick\(sguan'
Assert-Match 'fault snapshot current0 follows IV' $core 'g_foc_fault_snapshot\.current0\s*=.*ADC_InjectedValues\[0\]'
Assert-Match 'fault snapshot current1 follows IW' $core 'g_foc_fault_snapshot\.current1\s*=.*ADC_InjectedValues\[1\]'
Assert-Count 'float and Q31 feedforward share one safety macro each' $core '#if\s+USER_CURRENT_FEEDFORWARD_ENABLE' 2
Assert-Match 'CPU fault handlers force TIM1 MOE off' $irq '(?s)NMI_Handler.*?APP_FORCE_TIM1_MOE_OFF.*?HardFault_Handler.*?APP_FORCE_TIM1_MOE_OFF.*?MemManage_Handler.*?APP_FORCE_TIM1_MOE_OFF.*?BusFault_Handler.*?APP_FORCE_TIM1_MOE_OFF.*?UsageFault_Handler.*?APP_FORCE_TIM1_MOE_OFF'
Assert-Match 'Error_Handler clears TIM1 MOE before halting' $main '(?s)void\s+Error_Handler\s*\(void\).*?TIM1->BDTR\s*&=\s*~TIM_BDTR_MOE.*?__disable_irq\(\).*?while\s*\(1\)'

Write-Output 'FOC bring-up static safety checks passed.'
