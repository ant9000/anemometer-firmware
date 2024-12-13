/*! \file ch101_gpr_sr.c
 *
 * \brief Chirp CH101 General Purpose Rangefinding / Short Range firmware interface
 * 
 * This file contains function definitions to interface a specific sensor firmware 
 * package to SonicLib, including the main initialization routine for the firmware.  
 * That routine initializes various fields within the \a ch_dev_t device descriptor 
 * and specifies the proper functions to implement SonicLib API calls.  Those may 
 * either be common implementations or firmware-specific routines located in this file.
 */

/*
 Copyright © 2019-2021, Chirp Microsystems.  All rights reserved.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
 */

#include "ch101_gpr_sr.h"
#include <invn/soniclib/details/ch_common.h>

static const ch_rangefinder_api_funcs_t algo_api_funcs = {
    .set_threshold    = NULL,
    .get_threshold    = NULL,
    .set_thresholds   = NULL,
    .get_thresholds   = NULL,
    .set_static_range = ch_rangefinder_set_static_range,
    .set_rx_holdoff   = ch_rangefinder_set_rx_holdoff,
    .get_rx_holdoff   = ch_rangefinder_get_rx_holdoff,
    .get_tof_us       = NULL,
};

static const ch_api_funcs_t api_funcs = {
    .set_num_samples    = ch_common_set_num_samples,
    .get_range          = ch_rangefinder_get_range,
    .get_amplitude      = ch_rangefinder_get_amplitude,
    .get_iq_data        = ch_common_get_iq_data,
    .get_amplitude_data = ch_common_get_amplitude_data,
    .mm_to_samples      = ch_common_mm_to_samples,
    .set_sample_window  = ch_common_set_sample_window,
    .get_amplitude_avg  = ch_common_get_amplitude_avg,
    .set_cal_result     = ch_common_set_cal_result,
    .get_cal_result     = ch_common_get_cal_result,
    .algo_specific_api  = &algo_api_funcs,
};

static const ch_calib_funcs_t calib_funcs = {
    .prepare_pulse_timer = ch_common_prepare_pulse_timer,
    .store_pt_result     = ch_common_store_pt_result,
    .store_op_freq       = ch_common_store_op_freq,
    .store_bandwidth     = NULL,
    .store_scalefactor   = ch_common_store_scale_factor,
    .get_locked_state    = ch_common_get_locked_state,
};

static fw_info_t self = {
    .api_funcs                   = &api_funcs,
    .calib_funcs                 = &calib_funcs,
    .fw_includes_sensor_init     = 1,
    .fw_includes_tx_optimization = 0,
    .freqCounterCycles           = CH101_COMMON_FREQCOUNTERCYCLES,
    .freqLockValue               = CH101_COMMON_READY_FREQ_LOCKED,
    .oversample                  = 2, /* This firmware uses oversampling: 4x oversampling (value is power of 2) */
    .max_samples                 = CH101_GPR_SR_MAX_SAMPLES,
};

uint8_t ch101_gpr_sr_init(ch_dev_t *dev_ptr, fw_info_t **fw_info) {
	dev_ptr->part_number = CH101_PART_NUMBER;

	/* Init firmware-specific function pointers */
	self.fw_text              = ch101_gpr_sr_fw_text;
	self.fw_text_size         = ch101_gpr_sr_text_size;
	self.fw_vec               = NULL;
	self.fw_vec_size          = 0;
	self.fw_version_string    = ch101_gpr_sr_version;
	self.ram_init             = get_ram_ch101_gpr_sr_init_ptr();
	self.get_fw_ram_init_size = get_ch101_gpr_sr_fw_ram_init_size;
	self.get_fw_ram_init_addr = get_ch101_gpr_sr_fw_ram_init_addr;

	*fw_info = &self;
	return 0;
}
