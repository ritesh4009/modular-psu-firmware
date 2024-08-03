/*
 * EEZ Modular Firmware
 * Copyright (C) 2015-present, Envox d.o.o.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <new>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#if defined(EEZ_PLATFORM_STM32)
#include <main.h>
#include <crc.h>
#include <eez/platform/stm32/spi.h>
#include <memory.h>
#include <stdlib.h>

#endif

#include <eez/firmware.h>
#include <eez/system.h>

#include <eez/scpi/regs.h>

#include <eez/gui/gui.h>

#include <eez/modules/psu/psu.h>
#include <eez/modules/psu/profile.h>
#include <eez/modules/psu/channel_dispatcher.h>
#include <eez/modules/psu/trigger.h>
#include <eez/modules/psu/gui/psu.h>
#include <eez/modules/psu/gui/edit_mode.h>

#include <eez/modules/bp3c/comm.h>

#include <eez/modules/dib-dcm242/dib-dcm242.h>

/// ADC conversion should be finished after ADC_CONVERSION_MAX_TIME_MS milliseconds.

#define CONF_FALLING_EDGE_OVP_PERCENTAGE 2.0f

#define CONF_FALLING_EDGE_HW_OVP_DELAY_MS 2
#define CONF_FALLING_EDGE_SW_OVP_DELAY_MS 5
#define CONF_FALLING_EDGE_DP_OFF_DELAY_MS 3000

#define CONF_OVP_SW_OVP_AT_START_DURATION_MS 5
#define CONF_OVP_SW_OVP_AT_START_U_SET_THRESHOLD 1.2f
#define CONF_OVP_SW_OVP_AT_START_U_PROTECTION_LEVEL 1.55f

#define SIGNIFICANT_INPUT_VOLTAGE_WHEN_CHANNEL_IS_OFF 0.1f
#define ERROR_INPUT_VOLTAGE_WHEN_CHANNEL_IS_OFF 0.5f

#define CURRENT_WHEN_CHANNEL_IS_OFF 0.002f

//volatile float g_uSet;
namespace eez {

//using namespace gui;
using namespace psu;

namespace dcm242 {

static const uint16_t MODULE_REVISION_DCM242_R1B1  = 0x0242;

static const uint16_t DAC_MIN = 0;
static const uint16_t DAC_MAX = 4095;

static const uint16_t ADC_MIN = 0;
static const uint16_t ADC_MAX = 65535;



#define BUFFER_SIZE 14

static const float PTOT = 40.0f;
static const float I_MON_RESOLUTION = 0.02f;

#define REG0_OE_MASK      (1 << 0)
#define REG0_CC_MASK      (1 << 1)
#define REG0_PWRGOOD_MASK (1 << 2)
#define REG0_DP_MASK	  (1 << 3)
#define REG0_R_SENSE_MASK	  (1 << 4)



struct DcmChannel : public Channel {
		bool outputEnable;
		bool r_sense;
		bool dpOn;
		uint32_t dpNegMonitoringTimeMs = 0;

		float uBeforeBalancing = NAN;
		float iBeforeBalancing = NAN;


		#if defined(EEZ_PLATFORM_STM32)
			uint16_t uSet;
			uint16_t iSet;
		#endif

		#if defined(EEZ_PLATFORM_SIMULATOR)
			float uMon;
			float iMon;
			float uSet;
			float iSet;
		#endif

	    uint16_t uMonAdc = 0;
	    uint16_t iMonAdc = 0;

	    float temperature = 25.0f;

		float I_MAX_FOR_REMAP;

		float U_CAL_POINTS[2];
		float I_CAL_POINTS[2];

		bool ccMode = false;
	    DcmChannel(uint8_t slotIndex, uint8_t channelIndex, uint8_t subchannelIndex)
	        : Channel(slotIndex, channelIndex, subchannelIndex)
	    {
	        channelHistory = new ChannelHistory(*this);
	    }

		void getParams(uint16_t moduleRevision) override {

			params.U_MIN = 1.0f;
			params.U_DEF = 5.0f;
			params.U_MAX = 20.0f;

			params.U_MIN_STEP = 0.01f;
			params.U_DEF_STEP = 0.1f;
			params.U_MAX_STEP = 5.0f;

			U_CAL_POINTS[0] = 2.0f;
			U_CAL_POINTS[1] = 18.0f;
			params.U_CAL_NUM_POINTS = 2;
			params.U_CAL_POINTS = U_CAL_POINTS;
			params.U_CAL_I_SET = 1.0f;

			params.I_MIN = 0.01f;
			params.I_DEF = 0.01f;
			params.I_MAX = 2.0f;

	    	params.I_MON_MIN = 0.01f;

			params.I_MIN_STEP = 0.01f;
			params.I_DEF_STEP = 0.01f;
			params.I_MAX_STEP = 1.0f;

	        I_CAL_POINTS[0] = 0.1f;
	        I_CAL_POINTS[1] = 0.2f;
	        I_CAL_POINTS[2] = 0.3f;
	        params.I_CAL_NUM_POINTS = 3;
			params.I_CAL_POINTS = I_CAL_POINTS;
			params.I_CAL_U_SET = 20.0f;

			params.OVP_DEFAULT_STATE = false;
			params.OVP_MIN_DELAY = 0.0f;
			params.OVP_DEFAULT_DELAY = 0.0f;
			params.OVP_MAX_DELAY = 10.0f;

			params.OCP_DEFAULT_STATE = false;
			params.OCP_MIN_DELAY = 0.0f;
			params.OCP_DEFAULT_DELAY = 0.02f;
			params.OCP_MAX_DELAY = 10.0f;

			params.OPP_DEFAULT_STATE = false;
			params.OPP_MIN_DELAY = 1.0f;
			params.OPP_DEFAULT_DELAY = 10.0f;
			params.OPP_MAX_DELAY = 300.0f;
			params.OPP_MIN_LEVEL = 0.0f;
			params.OPP_DEFAULT_LEVEL = 80.0f;

			params.PTOT = MIN(params.U_MAX * params.I_MAX, 40.0f);

			params.U_RESOLUTION = 0.01f;
			params.U_RESOLUTION_DURING_CALIBRATION = 0.001f;
			params.I_RESOLUTION = 0.01f;
			params.I_RESOLUTION_DURING_CALIBRATION = 0.001f;
			params.P_RESOLUTION = 0.001f;

			params.VOLTAGE_GND_OFFSET = 0;
			params.CURRENT_GND_OFFSET = 0;

			params.CALIBRATION_DATA_TOLERANCE_PERCENT = 15.0f;

			params.CALIBRATION_MID_TOLERANCE_PERCENT = 3.0f;

			params.features = CH_FEATURE_VOLT | CH_FEATURE_CURRENT | CH_FEATURE_POWER | CH_FEATURE_OE;

			params.MON_REFRESH_RATE_MS = 500;

			params.DAC_MAX = DAC_MAX;
			params.ADC_MAX = ADC_MAX;

			I_MAX_FOR_REMAP = 2.0000f;

			params.U_RAMP_DURATION_MIN_VALUE = 0.002f;

	        params.OCP_TRIP_LEVEL_PERCENT = 90.0f;
			params.OCP_TRIP_LEVEL_PERCENT_MIN_VALUE_HIGH_RANGE = 0.1f;
		}

	    void onPowerDown() override;

	    void reset(bool resetLabelAndColor) override {
			Channel::reset(resetLabelAndColor);

			uSet = 0;
			iSet = 0;

	        dpOn = false;
	        r_sense = false;
			uBeforeBalancing = NAN;
			iBeforeBalancing = NAN;
		}

	    bool test() override;
	    void tickSpecific() override;

		bool isInCcMode() override {
			#if defined(EEZ_PLATFORM_STM32)
					return ccMode;
			#endif

			#if defined(EEZ_PLATFORM_SIMULATOR)
					return simulator::getCC(channelIndex);
			#endif
		}

		bool isInCvMode() override {
			return !isInCcMode();
		}

		bool isOvpEnabled() override {
				if (prot_conf.flags.u_state) {
					auto &slot = *g_slots[slotIndex];
					if (slot.moduleRevision <= MODULE_REVISION_DCM242_R1B1) {
						auto triggerMode = getVoltageTriggerMode();
						return triggerMode != TRIGGER_MODE_LIST && triggerMode != TRIGGER_MODE_FUNCTION_GENERATOR;
					}
					return true;
				}
				return false;
			}

		bool isHwOvpEnabled() {
			    return isOvpEnabled() && prot_conf.flags.u_type && !flags.rprogEnabled;
		    }

		void adcMeasureUMon() override {
		}

		void adcMeasureIMon() override {
		}

		void adcMeasureMonDac() override {
		}

		void adcMeasureAll() override {
		}

		bool shouldDisableDP() {
			// disable DP if low current range and current of 10 mA or less is set
			if (flags.currentCurrentRange == 1 && i.set <= 10E-3f) {
				return true;
			}

			// in parallel coupling, only DP on channel 1 could be enabled
			if (channelIndex == 1 && channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_PARALLEL) {
				return true;
			}

			return false;
		}

		void setDpEnable(bool enable) {
			if (enable && shouldDisableDP()) {
				return;
			}

			// DP bit is active low
			//ioexp.changeBit(IOExpander::IO_BIT_OUT_DP_ENABLE, !enable);

			//setOperBits(OPER_ISUM_DP_OFF, !enable);
			dpOn = enable;
		}

		void setOutputEnable(bool enable, uint16_t tasks) override {
			outputEnable = enable;
	        u.resetMonValues();
	        i.resetMonValues();

	        /*if (enable) {
				// OVP
				if (tasks & OUTPUT_ENABLE_TASK_OVP) {
					if (isHwOvpEnabled()) {
						if (dac.isOverHwOvpThreshold()) {
							// OVP has to be enabled after OE activation
							prot_conf.flags.u_hwOvpDeactivated = 0;
							ioexp.changeBit(IOExpander::IO_BIT_OUT_OVP_ENABLE, true);
						}
					}
				}

				// DP
				if (tasks & OUTPUT_ENABLE_TASK_DP) {
					if (flags.dprogState == DPROG_STATE_ON) {
						// enable DP
						delayed_dp_off = false;
						setDpEnable(true);
					}

					adc.start(ADC_DATA_TYPE_U_MON);
				}

				dpNegMonitoringTimeMs = 0;
	        }

	        else {
				// OVP
				if (tasks & OUTPUT_ENABLE_TASK_OVP) {
					if (isHwOvpEnabled()) {
						// OVP has to be disabled before OE deactivation
						prot_conf.flags.u_hwOvpDeactivated = 1;
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OVP_ENABLE, false);
					}
				}

				// DAC
				if (tasks & OUTPUT_ENABLE_TASK_DAC) {
					dac.setDacVoltage(0);

					dac.setCurrent(getCalibratedCurrent(CURRENT_WHEN_CHANNEL_IS_OFF)); // set to prevent both CC and CV leds on when channel is off
				}

				// OE
				if (tasks & OUTPUT_ENABLE_TASK_OE) {
					ioexp.changeBit(IOExpander::IO_BIT_OUT_OUTPUT_ENABLE, false);

					u.resetMonValues();
					i.resetMonValues();
				}

				// Current range
				if (tasks & OUTPUT_ENABLE_TASK_CURRENT_RANGE) {
					doSetCurrentRange();
				}

				// DP
				if (tasks & OUTPUT_ENABLE_TASK_DP) {
					if (flags.dprogState == DPROG_STATE_ON) {
						// turn off DP after some delay
						delayed_dp_off = true;
						delayed_dp_off_start = millis();
					}

					adc.start(ADC_DATA_TYPE_U_MON);
				}
			}

	        if (tasks & OUTPUT_ENABLE_TASK_FINALIZE) {
					if (channelIndex == 0 && channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_PARALLEL) {
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_UNCOUPLED_LED, false);
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_COUPLED_LED, enable);
					} else if (channelIndex == 1 && channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_PARALLEL) {
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_UNCOUPLED_LED, false);
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_COUPLED_LED, false);
					} else if (channelIndex == 0 && channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_SERIES) {
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_UNCOUPLED_LED, false);
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_COUPLED_LED, enable);
					} else if (channelIndex == 1 && channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_SERIES) {
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_UNCOUPLED_LED, false);
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_COUPLED_LED, false);
					} else if (channelIndex < 2 && channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_SPLIT_RAILS) {
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_UNCOUPLED_LED, false);
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_COUPLED_LED, enable);
					} else if (channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_COMMON_GND) {
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_UNCOUPLED_LED, false);
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_COUPLED_LED, enable);
					} else {
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_UNCOUPLED_LED, enable);
						ioexp.changeBit(IOExpander::IO_BIT_OUT_OE_COUPLED_LED, false);
					}

					restoreVoltageToValueBeforeBalancing(*this);
					restoreCurrentToValueBeforeBalancing(*this);
				}*/


	    }

		/*void setDprogState(DprogState dprogState) override {
				if (!isPsuThread()) {
					sendMessageToPsu(PSU_MESSAGE_SET_DPROG_STATE, (channelIndex << 8) | dprogState);
				} else {
					Channel::setDprogState(dprogState);

					if (dprogState == DPROG_STATE_OFF) {
						setDpEnable(false);
					} else {
						setDpEnable(isOk() && ioexp.testBit(IOExpander::IO_BIT_OUT_OUTPUT_ENABLE));
					}
					delayed_dp_off = false;
				}
		    }*/

		void setRemoteSense(bool enable) override {
				//ioexp.changeBit(IOExpander::IO_BIT_OUT_R_SENSE, enable);
			if (enable) {
				r_sense = enable;
			}

			return;
		}

		/*void setRemoteProgramming(bool enable) override {
			ioexp.changeBit(IOExpander::IO_BIT_OUT_REMOTE_PROGRAMMING, enable);
		}*/

	    void setDacVoltage(uint16_t value) override {

	#if defined(EEZ_PLATFORM_STM32)
	        value = (uint16_t)clamp((float)value, (float)DAC_MIN, (float)DAC_MAX);
	        uSet = clamp(value, DAC_MIN, DAC_MAX);
	#endif

	#if defined(EEZ_PLATFORM_SIMULATOR)
			uSet = remap(clamp((float)value, (float)DAC_MIN, (float)DAC_MAX), (float)DAC_MIN, 0, (float)DAC_MAX, params.U_MAX);
	#endif
		}

		void setDacVoltageFloat(float value) override {
	#if defined(EEZ_PLATFORM_STM32)
	        value = remap(value, 0, (float)DAC_MIN, params.U_MAX, (float)DAC_MAX);
	        printf("voltage value is x\n");
	        uSet = (uint16_t)clamp(round(value), DAC_MIN, DAC_MAX);
	        printf("Back\n");
	#endif
		}

		void setDacCurrent(uint16_t value) override {
	#if defined(EEZ_PLATFORM_STM32)
	        value = (uint16_t)clamp((float)value, (float)DAC_MIN, (float)DAC_MAX);
	        iSet = value;
	#endif
	    }

		void setDacCurrentFloat(float value) override {
	#if defined(EEZ_PLATFORM_STM32)
	        value = remap(value, /*params.I_MIN*/ 0, (float)DAC_MIN, /*params.I_MAX*/ I_MAX_FOR_REMAP, (float)DAC_MAX);
	        iSet = (uint16_t)clamp(round(value), DAC_MIN, DAC_MAX);
	#endif
		}

		bool isDacTesting() override {
			return false;
		}

		/*bool isVoltageBalanced() const {
		        return !isNaN(uBeforeBalancing);
		    }

		bool isCurrentBalanced() const {
			return !isNaN(iBeforeBalancing);
		}

		float getUSet() const override {
        return isVoltageBalanced() ? uBeforeBalancing : u.set;
    	}

		float getISet() const override {
			return isCurrentBalanced() ? iBeforeBalancing : i.set;
		}

		static void voltageBalancing(psu::Channel &channel) {
        DcpChannel &dcpChannel = (DcpChannel &)channel;
        if (isNaN(dcpChannel.uBeforeBalancing)) {
            dcpChannel.uBeforeBalancing = channel.u.set;
        }
        dcpChannel.valueBalancing = true;

        channel.doSetVoltage(
        	channel.roundChannelValue(
        		UNIT_VOLT,
        		(psu::Channel::get(0).u.mon_last + psu::Channel::get(1).u.mon_last) / 2
			)
		);

        dcpChannel.valueBalancing = false;
    }

    static void currentBalancing(psu::Channel &channel) {
        DcpChannel &dcpChannel = (DcpChannel &)channel;
        if (isNaN(dcpChannel.iBeforeBalancing)) {
            dcpChannel.iBeforeBalancing = channel.i.set;
        }
        dcpChannel.valueBalancing = true;

        channel.doSetCurrent(
        	channel.roundChannelValue(
        		UNIT_AMPER,
        		(psu::Channel::get(0).i.mon_last + psu::Channel::get(1).i.mon_last) / 2
			)
		);

        dcpChannel.valueBalancing = false;
    }

    static void restoreVoltageToValueBeforeBalancing(psu::Channel &channel) {
        DcpChannel &dcpChannel = (DcpChannel &)channel;
        if (!isNaN(dcpChannel.uBeforeBalancing)) {
            // DebugTrace("Restore voltage to value before balancing: %f", uBeforeBalancing);
            channel.setVoltage(dcpChannel.uBeforeBalancing);
            dcpChannel.uBeforeBalancing = NAN;
        }
    }

    static void restoreCurrentToValueBeforeBalancing(psu::Channel &channel) {
        DcpChannel &dcpChannel = (DcpChannel &)channel;
        if (!isNaN(dcpChannel.iBeforeBalancing)) {
            // DebugTrace("Restore current to value before balancing: %f", index, iBeforeBalancing);
            channel.setCurrent(dcpChannel.iBeforeBalancing);
            dcpChannel.iBeforeBalancing = NAN;
        }
    }

	#if defined(EEZ_PLATFORM_STM32)
		void onSpiIrq() {
			uint8_t intcap = ioexp.readIntcapRegister();
			// DebugTrace("CH%d INTCAP 0x%02X\n", (int)(channelIndex + 1), (int)intcap);
			if (!(intcap & (1 << IOExpander::R2B5_IO_BIT_IN_OVP_FAULT))) {
				if (isOutputEnabled() && isHwOvpEnabled() && !io_pins::isInhibited()) {
					protectionEnter(ovp, true);
				}
			} else if (!(intcap & (1 << IOExpander::IO_BIT_IN_PWRGOOD))) {
				NVIC_SystemReset();
			}
		}
	#endif*/

		void getVoltageStepValues(StepValues *stepValues, bool calibrationMode) override {
	        static float values[] = { 0.01f, 0.1f, 0.5f, 1.0f};
			static float calibrationModeValues[] = { 0.001f, 0.01f, 0.1f, 1.0f };
	        stepValues->values = calibrationMode ? calibrationModeValues : values;
	        stepValues->count = sizeof(values) / sizeof(float);
			stepValues->unit = UNIT_VOLT;

			stepValues->encoderSettings.accelerationEnabled = true;
			stepValues->encoderSettings.range = params.U_MAX;
			stepValues->encoderSettings.step = params.U_RESOLUTION;
	        if (calibrationMode) {
	            stepValues->encoderSettings.step /= 10.0f;
	            stepValues->encoderSettings.range = stepValues->encoderSettings.step * 10.0f;
	        }
	        stepValues->encoderSettings.mode = psu::gui::edit_mode_step::g_dcmVoltageEncoderMode;
		}

	    void setVoltageEncoderMode(EncoderMode encoderMode) override {
			psu::gui::edit_mode_step::g_dcmVoltageEncoderMode = encoderMode;
	    }

		void getCurrentStepValues(StepValues *stepValues, bool calibrationMode, bool highRange) override {
	        static float values[] = { 0.01f, 0.1f, 0.25f, 0.5f   };
			static float calibrationModeValues[] = { 0.001f, 0.005f, 0.01f, 0.05f };
	        stepValues->values = calibrationMode ? calibrationModeValues : values;
	        stepValues->count = sizeof(values) / sizeof(float);
			stepValues->unit = UNIT_AMPER;

			stepValues->encoderSettings.accelerationEnabled = true;
			stepValues->encoderSettings.range = params.I_MAX;
			stepValues->encoderSettings.step = params.I_RESOLUTION;
	        if (calibrationMode) {
	            stepValues->encoderSettings.range /= 100.0f;
	            stepValues->encoderSettings.range = stepValues->encoderSettings.step * 10.0f;
	        }
	        stepValues->encoderSettings.mode = psu::gui::edit_mode_step::g_dcmCurrentEncoderMode;
		}

	    void setCurrentEncoderMode(EncoderMode encoderMode) override {
			psu::gui::edit_mode_step::g_dcmCurrentEncoderMode = encoderMode;
	    }

	    void getPowerStepValues(StepValues *stepValues) override {
	        static float values[] = { 0.01f, 0.1f, 1.0f, 10.0f };
	        stepValues->values = values;
	        stepValues->count = sizeof(values) / sizeof(float);
			stepValues->unit = UNIT_WATT;

			stepValues->encoderSettings.accelerationEnabled = true;
			stepValues->encoderSettings.range = params.PTOT;
			stepValues->encoderSettings.step = params.P_RESOLUTION;
	        stepValues->encoderSettings.mode = psu::gui::edit_mode_step::g_dcmPowerEncoderMode;
		}

	    void setPowerEncoderMode(EncoderMode encoderMode) override {
			psu::gui::edit_mode_step::g_dcmPowerEncoderMode = encoderMode;
	    }

		bool isPowerLimitExceeded(float u, float i, int *err) override {
			float power = u * i;
			if (power > channel_dispatcher::getPowerLimit(*this)) {
				if (err) {
					*err = SCPI_ERROR_POWER_LIMIT_EXCEEDED;
				}
				return true;
			}


	        /*float powerOtherChannel;
	        auto &otherChannel = Channel::get(channelIndex + (subchannelIndex == 0 ? 1 : -1));
	        if (flags.trackingEnabled && otherChannel.flags.trackingEnabled) {
	            powerOtherChannel = power;
	        } else {
	            powerOtherChannel = channel_dispatcher::getUSet(otherChannel) * channel_dispatcher::getISet(otherChannel);
	        }
	        if (power + powerOtherChannel > PTOT) {
	            if (err) {
					*err = SCPI_ERROR_MODULE_TOTAL_POWER_LIMIT_EXCEEDED;
				}
				return true;
			}*/

			return false;
		}

	    float readTemperature() override {
	#if defined(EEZ_PLATFORM_STM32)
	        return temperature;
	#else
	        return NAN;
	#endif
	    }

	    int getAdvancedOptionsPageId() override {
	    		return eez::gui::PAGE_ID_CH_SETTINGS_ADV_OPTIONS;
	    	}

		void dumpDebugVariables(scpi_t *context) override {
			char buffer[100];

			snprintf(buffer, sizeof(buffer), "CH%d U_DAC = %d", channelIndex + 1, (int)uSet);
			SCPI_ResultText(context, buffer);

			snprintf(buffer, sizeof(buffer), "CH%d U_MON = %d", channelIndex + 1, (int)uMonAdc);
			SCPI_ResultText(context, buffer);

			snprintf(buffer, sizeof(buffer), "CH%d I_DAC = %d", channelIndex + 1, (int)iSet);
			SCPI_ResultText(context, buffer);

			snprintf(buffer, sizeof(buffer), "CH%d I_MON = %d", channelIndex + 1, (int)iMonAdc);
			SCPI_ResultText(context, buffer);
	    }
	    bool isSignificantInputVoltageDetectedWhenChannellIsOff() {
	        return u.mon > SIGNIFICANT_INPUT_VOLTAGE_WHEN_CHANNEL_IS_OFF;
	    }

	    bool isErrorInputVoltageDetectedWhenChannellIsOff() override {
	        if (channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_SERIES) {
	            auto otherChannelIndex = channelIndex == 0 ? 1 : 0;
	            auto &otherChannel = Channel::get(otherChannelIndex);
	            return u.mon + otherChannel.u.mon > ERROR_INPUT_VOLTAGE_WHEN_CHANNEL_IS_OFF;
	        }
	        return u.mon > ERROR_INPUT_VOLTAGE_WHEN_CHANNEL_IS_OFF;
	    }
};

struct DcmModule : public PsuModule {
public:
    bool synchronized = false;
    int numCrcErrors = 0;
    uint8_t input[BUFFER_SIZE];
    uint8_t output[BUFFER_SIZE];

    DcmModule() {
        moduleType = MODULE_TYPE_DCM242;
        moduleName = "DCM242";
        moduleBrand = "Ritesh";
        latestModuleRevision = MODULE_REVISION_DCM242_R1B1;
        flashMethod = FLASH_METHOD_STM32_BOOTLOADER_UART;
#if defined(EEZ_PLATFORM_STM32)
        spiBaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        spiCrcCalculationEnable = false;
#else
        spiBaudRatePrescaler = 0;
        spiCrcCalculationEnable = false;
#endif
        numPowerChannels = 1;
        numOtherChannels = 0;
        isResyncSupported = false;

    	memset(output, 0, sizeof(output));
    	memset(input, 0, sizeof(input));
    }

	Module *createModule() override {
        return new DcmModule();
    }

    void initChannels() override {
        if (enabled && !synchronized) {
            setTestResult(TEST_CONNECTING);
            if (bp3c::comm::masterSynchro(slotIndex)) {
                //printf("DCM242 slot #%d firmware version %d.%d\n", slotIndex + 1, (int)firmwareMajorVersion, (int)firmwareMinorVersion);
                synchronized = true;
                numCrcErrors = 0;
            } else {
                if (g_slots[slotIndex]->firmwareInstalled) {
                    event_queue::pushEvent(event_queue::EVENT_ERROR_SLOT1_SYNC_ERROR + slotIndex);
                }
            }
        }
    }

	Channel *createPowerChannel(int slotIndex, int channelIndex, int subchannelIndex) override {
        void *buffer = malloc(sizeof(DcmChannel));
        memset(buffer, 0, sizeof(DcmChannel));
		return new (buffer) DcmChannel(slotIndex, channelIndex, subchannelIndex);
	}

/*#if defined(EEZ_PLATFORM_STM32)
	void onSpiIrq() {
		auto dcpChannel = (DcpChannel *)Channel::getBySlotIndex(slotIndex);
		dcpChannel->spiIrq = true;
	}
#endif*/

    void onPowerDown() override {
#if defined(EEZ_PLATFORM_STM32)
        if (synchronized) {
            transfer();
        }
#endif
        synchronized = false;
        setTestResult(TEST_FAILED);
    }

    void test() {
        if (!enabled) {
            setTestResult(TEST_SKIPPED);
            return;
        }

        if (!synchronized) {
            setTestResult(TEST_FAILED);
            return;
        }

#if defined(EEZ_PLATFORM_STM32)
        output[0] = 0;
        transfer();
#endif

        bool pwrGood;
        if (numCrcErrors == 0) {
#if defined(EEZ_PLATFORM_STM32)
            pwrGood = input[0] & REG0_PWRGOOD_MASK ? true : false;
#endif

#if defined(EEZ_PLATFORM_SIMULATOR)
            auto channelIndex = Channel::getBySlotIndex(slotIndex)->channelIndex;
            pwrGood = simulator::getPwrgood(channelIndex) && simulator::getPwrgood(channelIndex + 1);
#endif
        } else {
            pwrGood = true;
        }
        int subchannelIndex = 0; //added by ritesh
        //for (int subchannelIndex = 0; subchannelIndex < 2; subchannelIndex++) {
            auto &channel = *Channel::getBySlotIndex(slotIndex, subchannelIndex);
            channel.flags.powerOk = pwrGood ? 1 : 0;
        //}

        setTestResult(pwrGood ? TEST_OK : TEST_FAILED);

		if (getTestResult() == TEST_OK) {
            // test temp. sensors
			int subchannelIndex = 0; //added by ritesh
            //for (int subchannelIndex = 0; testResult == TEST_OK && subchannelIndex < 2; subchannelIndex++) {
                auto &channel = *Channel::getBySlotIndex(slotIndex, subchannelIndex);
                if (!temp_sensor::sensors[temp_sensor::CH1 + channel.channelIndex].test()) {
					setTestResult(TEST_FAILED);
                //}
            }
        }
    }

#if defined(EEZ_PLATFORM_STM32)
    void transfer() {
        auto status = bp3c::comm::transfer(slotIndex, output, input, BUFFER_SIZE);
        if (status == bp3c::comm::TRANSFER_STATUS_OK) {
            numCrcErrors = 0;
        } else {
            if (status == bp3c::comm::TRANSFER_STATUS_CRC_ERROR) {
                if (++numCrcErrors >= 4) {
                    event_queue::pushEvent(event_queue::EVENT_ERROR_SLOT1_CRC_CHECK_ERROR + slotIndex);
                    synchronized = false;
                    setTestResult(TEST_FAILED);
                } else {
                    //printf("Slot %d CRC %d\n", slotIndex + 1, numCrcErrors);
                }
            } else {
               // printf("Slot %d SPI transfer error %d\n", slotIndex + 1, status);
            }
        }
    }

    static float calcTemperature(uint16_t adcValue) {
    	 if (adcValue == 65535) {
    	        // not measured yet
    	        return 25.0f;
    	    }

    	    // MCP9701 characteristics
    	    float ADC_REF_VOLTAGE = 2.048f; // Reference voltage
    	    float ADC_MAX_VALUE = 4095.0f;  // 12-bit ADC max value
    	    float MCP9701_OFFSET_VOLTAGE = 0.4f; // Voltage at 0°C (400 mV)
    	    float MCP9701_TEMPERATURE_COEFFICIENT = 0.0195f; // 19.5 mV/°C

    	    // Calculate the voltage from ADC value
    	    float voltage = (adcValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

    	    // Calculate the temperature
    	    float temperature = (voltage - MCP9701_OFFSET_VOLTAGE) / MCP9701_TEMPERATURE_COEFFICIENT;

    	    //return temperature;
    	    return roundPrec(temperature, 1.0f);

        //return roundPrec(Tcelsius, 1.0f);
    }

    void tick(uint8_t slotIndex) {
        DcmChannel &channel1 = (DcmChannel &)*Channel::getBySlotIndex(slotIndex, 0);

        output[0] = 0x80 | (channel1.outputEnable ? REG0_OE_MASK : 0) | (channel1.dpOn ? REG0_DP_MASK : 0) | (channel1.r_sense ? REG0_R_SENSE_MASK : 0);

        output[1] = 0;

        uint16_t *outputSetValues = (uint16_t *)(output + 2);
        outputSetValues[0] = channel1.uSet;
        outputSetValues[1] = channel1.iSet;

        printf("voltage value is %d\n", output[2]);
        printf("voltage value is %d\n", outputSetValues[4]);

        transfer();

        if (numCrcErrors == 0) {
            uint16_t *inputSetValues = (uint16_t *)(input + 2);
            	int subchannelIndex = 0;
            //for (int subchannelIndex = 0; subchannelIndex < 2; subchannelIndex++) {
                auto &channel = *(DcmChannel *)Channel::getBySlotIndex(slotIndex, subchannelIndex);
                int offset = subchannelIndex * 2;

                channel.ccMode = (input[0] & REG0_CC_MASK) != 0;

                uint16_t uMonAdc = inputSetValues[offset];
                channel.uMonAdc = uMonAdc;
                float uMon = remap(uMonAdc, (float)ADC_MIN, 0, (float)ADC_MAX, channel.params.U_MAX);
                channel.onAdcData(ADC_DATA_TYPE_U_MON, uMon);

                uint16_t iMonAdc = inputSetValues[offset + 1];
                channel.iMonAdc = iMonAdc;
                const float FULL_SCALE = 2.0F;
                const float U_REF = 2.5F;
                float iMon = remap(iMonAdc, (float)ADC_MIN, 0, FULL_SCALE * ADC_MAX / U_REF, /*params.I_MAX*/ channel.I_MAX_FOR_REMAP);
                iMon = roundPrec(iMon, I_MON_RESOLUTION);
                channel.onAdcData(ADC_DATA_TYPE_I_MON, iMon);

#if !CONF_SKIP_PWRGOOD_TEST
                bool pwrGood = input[0] & REG0_PWRGOOD_MASK ? true : false;
                if (!pwrGood) {
                    generateChannelError(SCPI_ERROR_CH1_FAULT_DETECTED, channel.channelIndex);
                    powerDownOnlyPowerChannels();
                }
#endif
                uint16_t tempAdc = inputSetValues[offset + 2];
                channel.temperature = calcTemperature(tempAdc);

            //}
        }
    }
#endif

    int getSlotView(SlotViewType slotViewType, int slotIndex, int cursor) override {
    		int isVert = persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_NUMERIC || persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_VERT_BAR;
    		Channel &channel = Channel::get(cursor);

    		if (slotViewType == SLOT_VIEW_TYPE_DEFAULT) {
    			if (channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_SERIES && channel.channelIndex == 1) {
    				if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_NUMERIC || persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_VERT_BAR) {
    					return PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_COUPLED_SERIES;
    				} else {
    					return PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_COUPLED_SERIES;
    				}
    			} else if (channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_PARALLEL && channel.channelIndex == 1) {
    				if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_NUMERIC || persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_VERT_BAR) {
    					return PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_COUPLED_PARALLEL;
    				} else {
    					return PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_COUPLED_PARALLEL;
    				}
    			} else if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_NUMERIC) {
    				return channel.isOutputEnabled() ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_NUM_ON : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_OFF;
    			} else if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_VERT_BAR) {
    				return channel.isOutputEnabled() ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VBAR_ON : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_OFF;
    			} else if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_HORZ_BAR) {
    				return channel.isOutputEnabled() ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HBAR_ON : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_OFF;
    			} else if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_YT) {
    				return channel.isOutputEnabled() ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_YT_ON : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_OFF;
    			} else {
    				return isVert ? PAGE_ID_SLOT_DEF_VERT_ERROR : PAGE_ID_SLOT_DEF_HORZ_ERROR;
    			}
    		}

    		if (slotViewType == SLOT_VIEW_TYPE_DEFAULT_2COL) {
    			if (channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_SERIES && channel.channelIndex == 1) {
    				if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_NUMERIC || persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_VERT_BAR) {
    					return PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_COUPLED_SERIES_2COL;
    				} else {
    					return PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_COUPLED_SERIES_2COL;
    				}
    			} else if (channel_dispatcher::getCouplingType() == channel_dispatcher::COUPLING_TYPE_PARALLEL && channel.channelIndex == 1) {
    				if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_NUMERIC || persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_VERT_BAR) {
    					return PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_COUPLED_PARALLEL_2COL;
    				} else {
    					return PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_COUPLED_PARALLEL_2COL;
    				}
    			} else if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_NUMERIC) {
    				return channel.isOutputEnabled() ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_NUM_ON_2COL : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_OFF_2COL;
    			} else if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_VERT_BAR) {
    				return channel.isOutputEnabled() ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VBAR_ON_2COL : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_OFF_2COL;
    			} else if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_HORZ_BAR) {
    				return channel.isOutputEnabled() ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HBAR_ON_2COL : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_OFF_2COL;
    			} else if (persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_YT) {
    				return channel.isOutputEnabled() ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_YT_ON_2COL : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_OFF_2COL;
    			} else {
    				return isVert ? PAGE_ID_SLOT_DEF_VERT_ERROR_2COL : PAGE_ID_SLOT_DEF_HORZ_ERROR_2COL;
    			}
    		}

    		assert(slotViewType == SLOT_VIEW_TYPE_MAX);
    		return PAGE_ID_DIB_DCM242_SLOT_MAX;
    	}

    int getLabelsAndColorsPageId() override {
        return getTestResult() == TEST_OK ? PAGE_ID_DIB_DCM242_LABELS_AND_COLORS : PAGE_ID_NONE;
    }

	const char *getPinoutFile() override {
		return "dcm242_pinout.jpg";
	}

    void getFunctionGeneratorFrequencyInfo(int subchannelIndex, int resourceIndex, float &min, float &max, StepValues *stepValues) override {
        min = 0.01f;
        max = 10.0f;

        if (stepValues) {
            static float values[] = { 0.01f, 0.1f, 1.0f, 5.0f };
            stepValues->values = values;
            stepValues->count = sizeof(values) / sizeof(float);
            stepValues->unit = UNIT_HERTZ;
        }
    }
};

void DcmChannel::onPowerDown() {
    Channel::onPowerDown();
    if (subchannelIndex == 0) {
        ((DcmModule *)g_slots[slotIndex])->onPowerDown();
    }
}

bool DcmChannel::test() {
    if (subchannelIndex == 0) {
        ((DcmModule *)g_slots[slotIndex])->test();
    }
    return isOk();
}

void DcmChannel::tickSpecific() {
#if defined(EEZ_PLATFORM_STM32)
    if (subchannelIndex == 0) {
        ((DcmModule *)g_slots[slotIndex])->tick(slotIndex);
    }
#endif

}

static DcmModule g_dcmModule;
Module *g_module = &g_dcmModule;
} // namespace DCM242

namespace gui {

void data_dib_dcm242_slot_2ch_ch1_index(DataOperationEnum operation, Cursor cursor, Value &value) {
    data_channel_index(Channel::get(cursor), operation, cursor, value);
}

void data_dib_dcm242_slot_2ch_ch2_index(DataOperationEnum operation, Cursor cursor, Value &value) {
    data_channel_index(Channel::get(persist_conf::isMaxView() && cursor == persist_conf::getMaxChannelIndex() && Channel::get(cursor).subchannelIndex == 1 ? cursor - 1 : cursor + 1), operation, cursor, value);
}

void data_dib_dcm242_slot_def_2ch_view(DataOperationEnum operation, Cursor cursor, Value &value) {
    if (operation == DATA_OPERATION_GET) {
        Channel &channel = Channel::get(cursor);
        int isVert = persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_NUMERIC || persist_conf::devConf.channelsViewMode == CHANNELS_VIEW_MODE_VERT_BAR;
        if (g_isCol2Mode) {
            value = channel.isOutputEnabled() ?
                (isVert ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_NUM_ON_2COL : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HBAR_ON_2COL) :
                (isVert ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_OFF_2COL : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_OFF_2COL);
        } else {
            value = channel.isOutputEnabled() ?
                (isVert ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_NUM_ON : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HBAR_ON) :
                (isVert ? PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_VERT_OFF : PAGE_ID_DIB_DCM242_SLOT_DEF_1CH_HORZ_OFF);
        }
    }
}

} // namespace gui


} // namespace eez
