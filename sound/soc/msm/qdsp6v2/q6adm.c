/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/ratelimit.h>
#include <sound/apr_audio-v2.h>
#include <linux/qdsp6v2/apr.h>
#include <sound/q6adm-v2.h>
#include <sound/q6audio-v2.h>
#include <sound/q6afe-v2.h>
#include <sound/hw_audio_log.h>

#include <sound/asound.h>
#include "msm-dts-eagle.h"

#define TIMEOUT_MS 1000

#define RESET_COPP_ID 99
#define INVALID_COPP_ID 0xFF
/* Used for inband payload copy, max size is 4k */
/* 2 is to account for module & param ID in payload */
#define ADM_GET_PARAMETER_LENGTH  (4096 - APR_HDR_SIZE - 2 * sizeof(uint32_t))

#define ULL_SUPPORTED_BITS_PER_SAMPLE 16
#define ULL_SUPPORTED_SAMPLE_RATE 48000

#define CMD_GET_HDR_SZ 16

struct adm_copp {

	atomic_t id[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t cnt[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t topology[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t mode[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t stat[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t rate[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t bit_width[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t app_type[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t acdb_id[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	wait_queue_head_t wait[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	wait_queue_head_t adm_delay_wait[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	atomic_t adm_delay_stat[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
	uint32_t adm_delay[AFE_MAX_PORTS][MAX_COPPS_PER_PORT];
};

struct source_tracking_data {
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	struct param_outband memmap;
	int apr_cmd_status;
};

struct adm_ctl {
	void *apr;

	struct adm_copp copp;

	atomic_t matrix_map_stat;
	wait_queue_head_t matrix_map_wait;

	atomic_t adm_stat;
	wait_queue_head_t adm_wait;

	struct cal_type_data *cal_data[ADM_MAX_CAL_TYPES];

	atomic_t mem_map_handles[ADM_MEM_MAP_INDEX_MAX];
	atomic_t mem_map_index;

	struct param_outband outband_memmap;
	struct source_tracking_data sourceTrackingData;

	int set_custom_topology;
	int ec_ref_rx;
};

static struct adm_ctl			this_adm;

struct adm_multi_ch_map {
	bool set_channel_map;
	char channel_mapping[PCM_FORMAT_MAX_NUM_CHANNEL];
};

static struct adm_multi_ch_map multi_ch_map = { false,
						{0, 0, 0, 0, 0, 0, 0, 0}
					      };

static int adm_get_parameters[MAX_COPPS_PER_PORT * ADM_GET_PARAMETER_LENGTH];
static int adm_module_topo_list[
	MAX_COPPS_PER_PORT * ADM_GET_TOPO_MODULE_LIST_LENGTH];

int adm_validate_and_get_port_index(int port_id)
{
	int index;
	int ret;

	ret = q6audio_validate_port(port_id);
	if (ret < 0) {
		pr_err("%s: port validation failed id 0x%x ret %d\n",
			__func__, port_id, ret);
		return -EINVAL;
	}

	index = afe_get_port_index(port_id);
	if (index < 0 || index >= AFE_MAX_PORTS) {
		pr_err("%s: Invalid port idx %d port_id 0x%x\n",
			__func__, index,
			port_id);
		return -EINVAL;
	}
	pr_debug("%s: port_idx- %d\n", __func__, index);
	return index;
}

int adm_get_default_copp_idx(int port_id)
{
	int port_idx = adm_validate_and_get_port_index(port_id), idx;

	if (port_idx < 0) {
		pr_err("%s: Invalid port id: 0x%x", __func__, port_id);
		return -EINVAL;
	}
	pr_debug("%s: port_idx:%d\n", __func__, port_idx);
	for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++) {
		if (atomic_read(&this_adm.copp.id[port_idx][idx]) !=
			RESET_COPP_ID)
			return idx;
	}
	return -EINVAL;
}

int adm_get_topology_for_port_from_copp_id(int port_id, int copp_id)
{
	int port_idx = adm_validate_and_get_port_index(port_id), idx;
	if (port_idx < 0) {
		pr_err("%s: Invalid port id: 0x%x", __func__, port_id);
		return 0;
	}
	for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++)
		if (atomic_read(&this_adm.copp.id[port_idx][idx]) == copp_id)
			return atomic_read(&this_adm.copp.topology[port_idx]
								  [idx]);
	pr_err("%s: Invalid copp_id %d port_id 0x%x\n",
		__func__, copp_id, port_id);
	return 0;
}

int adm_get_topology_for_port_copp_idx(int port_id, int copp_idx)
{
	int port_idx = adm_validate_and_get_port_index(port_id);
	if (port_idx < 0) {
		pr_err("%s: Invalid port id: 0x%x", __func__, port_id);
		return 0;
	}
	return atomic_read(&this_adm.copp.topology[port_idx][copp_idx]);
}

int adm_get_indexes_from_copp_id(int copp_id, int *copp_idx, int *port_idx)
{
	int p_idx, c_idx;
	for (p_idx = 0; p_idx < AFE_MAX_PORTS; p_idx++) {
		for (c_idx = 0; c_idx < MAX_COPPS_PER_PORT; c_idx++) {
			if (atomic_read(&this_adm.copp.id[p_idx][c_idx])
								== copp_id) {
				if (copp_idx != NULL)
					*copp_idx = c_idx;
				if (port_idx != NULL)
					*port_idx = p_idx;
				return 0;
			}
		}
	}
	return -EINVAL;
}

static int adm_get_copp_id(int port_idx, int copp_idx)
{
	pr_debug("%s: port_idx:%d copp_idx:%d\n", __func__, port_idx, copp_idx);

	if (copp_idx < 0 || copp_idx >= MAX_COPPS_PER_PORT) {
		pr_err("%s: Invalid copp_num: %d\n", __func__, copp_idx);
		return -EINVAL;
	}
	return atomic_read(&this_adm.copp.id[port_idx][copp_idx]);
}

static int adm_get_idx_if_copp_exists(int port_idx, int topology, int mode,
				 int rate, int bit_width, int app_type)
{
	int idx;

	pr_debug("%s: port_idx-%d, topology-0x%x, mode-%d, rate-%d, bit_width-%d\n",
		 __func__, port_idx, topology, mode, rate, bit_width);

	for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++)
		if ((topology ==
			atomic_read(&this_adm.copp.topology[port_idx][idx])) &&
		    (mode == atomic_read(&this_adm.copp.mode[port_idx][idx])) &&
		    (rate == atomic_read(&this_adm.copp.rate[port_idx][idx])) &&
		    (bit_width ==
			atomic_read(&this_adm.copp.bit_width[port_idx][idx])) &&
		    (app_type ==
			atomic_read(&this_adm.copp.app_type[port_idx][idx])))
			return idx;
	return -EINVAL;
}

static int adm_get_next_available_copp(int port_idx)
{
	int idx;

	pr_debug("%s:\n", __func__);
	for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++) {
		pr_debug("%s: copp_id:0x%x port_idx:%d idx:%d\n", __func__,
			 atomic_read(&this_adm.copp.id[port_idx][idx]),
			 port_idx, idx);
		if (atomic_read(&this_adm.copp.id[port_idx][idx]) ==
								RESET_COPP_ID)
			break;
	}
	return idx;
}

int adm_dts_eagle_set(int port_id, int copp_idx, int param_id,
		      void *data, uint32_t size)
{
	struct adm_cmd_set_pp_params_v5	admp;
	int p_idx, ret = 0, *update_params_value;

	pr_debug("DTS_EAGLE_ADM - %s: port id %i, copp idx %i, param id 0x%X\n",
		__func__, port_id, copp_idx, param_id);

	port_id = afe_convert_virtual_to_portid(port_id);
	p_idx = adm_validate_and_get_port_index(port_id);
	pr_debug("DTS_EAGLE_ADM - %s: after lookup, port id %i, port idx %i\n",
		__func__, port_id, p_idx);

	if (p_idx < 0) {
		pr_err("DTS_EAGLE_ADM - %s: invalid port index %i, port id %i, copp idx %i\n",
			__func__, p_idx, port_id, copp_idx);
		return -EINVAL;
	}

	update_params_value = (int *)this_adm.outband_memmap.kvaddr;
	if (update_params_value == NULL) {
		pr_err("DTS_EAGLE_ADM - %s: NULL memmap. Non Eagle topology selected?\n",
				__func__);
		ret = -EINVAL;
		goto fail_cmd;
	}
	*update_params_value++ = AUDPROC_MODULE_ID_DTS_HPX_POSTMIX;
	*update_params_value++ = param_id;
	*update_params_value++ = size;
	memcpy(update_params_value, data, size);

	admp.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	admp.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE, sizeof(admp));
	admp.hdr.src_svc = APR_SVC_ADM;
	admp.hdr.src_domain = APR_DOMAIN_APPS;
	admp.hdr.src_port = port_id;
	admp.hdr.dest_svc = APR_SVC_ADM;
	admp.hdr.dest_domain = APR_DOMAIN_ADSP;
	admp.hdr.dest_port = atomic_read(&this_adm.copp.id[p_idx][copp_idx]);
	admp.hdr.token = p_idx << 16 | copp_idx;
	admp.hdr.opcode = ADM_CMD_SET_PP_PARAMS_V5;
	admp.payload_addr_lsw = lower_32_bits(this_adm.outband_memmap.paddr);
	admp.payload_addr_msw = upper_32_bits(this_adm.outband_memmap.paddr);
	admp.mem_map_handle = atomic_read(&this_adm.mem_map_handles[
				atomic_read(&this_adm.mem_map_index)]);
	admp.payload_size = size + 12 /*see update_params_value header above*/;

	pr_debug("DTS_EAGLE_ADM - %s: Command was sent now check Q6 - port id = %d, size %d, module id %x, param id %x.\n",
			__func__, admp.hdr.dest_port,
			admp.payload_size, AUDPROC_MODULE_ID_DTS_HPX_POSTMIX,
			param_id);

	ret = apr_send_pkt(this_adm.apr, (uint32_t *)&admp);
	if (ret < 0) {
		pr_err("DTS_EAGLE_ADM - %s: ADM enable for port %d failed\n",
			__func__, port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	ret = wait_event_timeout(this_adm.copp.wait[p_idx][copp_idx], 1,
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("DTS_EAGLE_ADM - %s: set params timed out port = %d\n",
			__func__, port_id);
		ret = -EINVAL;
	}

fail_cmd:
	return ret;
}

int adm_dts_eagle_get(int port_id, int copp_idx, int param_id,
		      void *data, uint32_t size)
{
	struct adm_cmd_get_pp_params_v5	*admp = NULL;
	int p_idx, sz, ret = 0;
	uint32_t orig_size = size;

	pr_debug("DTS_EAGLE_ADM - %s: port id %i, copp idx %i, param id 0x%X\n",
		 __func__, port_id, copp_idx, param_id);

	port_id = afe_convert_virtual_to_portid(port_id);
	p_idx = adm_validate_and_get_port_index(port_id);
	if (p_idx < 0) {
		pr_err("DTS_EAGLE_ADM - %s: invalid port index %i, port id %i, copp idx %i\n",
				__func__, p_idx, port_id, copp_idx);
		return -EINVAL;
	}

	if ((size == 0) || !data) {
		pr_err("DTS_EAGLE_ADM - %s: invalid size %u or pointer %p.\n",
			__func__, size, data);
		return -EINVAL;
	}

	size = (size+3) & 0xFFFFFFFC;

	sz = sizeof(struct adm_cmd_get_pp_params_v5) + size + CMD_GET_HDR_SZ;
	admp = kzalloc(sz, GFP_KERNEL);
	if (!admp) {
		pr_err("DTS_EAGLE_ADM - %s, adm params memory alloc failed",
			__func__);
		return -ENOMEM;
	}

	admp->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	admp->hdr.pkt_size = sz;
	admp->hdr.src_svc = APR_SVC_ADM;
	admp->hdr.src_domain = APR_DOMAIN_APPS;
	admp->hdr.src_port = port_id;
	admp->hdr.dest_svc = APR_SVC_ADM;
	admp->hdr.dest_domain = APR_DOMAIN_ADSP;
	admp->hdr.dest_port = atomic_read(&this_adm.copp.id[p_idx][copp_idx]);
	admp->hdr.token = p_idx << 16 | copp_idx;
	admp->hdr.opcode = ADM_CMD_GET_PP_PARAMS_V5;
	admp->data_payload_addr_lsw = 0;
	admp->data_payload_addr_msw = 0;
	admp->mem_map_handle = 0;
	admp->module_id = AUDPROC_MODULE_ID_DTS_HPX_POSTMIX;
	admp->param_id = param_id;
	admp->param_max_size = size + CMD_GET_HDR_SZ;
	admp->reserved = 0;

	ret = apr_send_pkt(this_adm.apr, (uint32_t *)admp);
	if (ret < 0) {
		pr_err("DTS_EAGLE_ADM - %s: Failed to get EAGLE Params on port %d\n",
			__func__, port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	ret = wait_event_timeout(this_adm.copp.wait[p_idx][copp_idx], 1,
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("DTS_EAGLE_ADM - %s: EAGLE get params timed out port = %d\n",
			__func__, port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

	if (adm_get_parameters[0] > 0 &&
	    (adm_get_parameters[0] * sizeof(int)) == orig_size) {
		ret = 0;
		memcpy(data, &adm_get_parameters[1], orig_size);
	} else {
		ret = -EINVAL;
		pr_err("DTS_EAGLE_ADM - %s: EAGLE get params problem getting data, size was %zu, expected was %i - check callback error value",
				__func__, (adm_get_parameters[0] * sizeof(int)),
				orig_size);
	}
fail_cmd:
	kfree(admp);
	return ret;
}

int srs_trumedia_open(int port_id, int copp_idx, __s32 srs_tech_id,
		      void *srs_params)
{
	struct adm_cmd_set_pp_params_inband_v5 *adm_params = NULL;
	struct adm_cmd_set_pp_params_v5 *adm_params_ = NULL;
	__s32 sz = 0, param_id, module_id = SRS_TRUMEDIA_MODULE_ID, outband = 0;
	int ret = 0, port_idx;

	ad_logd("SRS - %s", __func__);
	switch (srs_tech_id) {
	case SRS_ID_GLOBAL: {
		struct srs_trumedia_params_GLOBAL *glb_params = NULL;
		sz = sizeof(struct adm_cmd_set_pp_params_inband_v5) +
			sizeof(struct srs_trumedia_params_GLOBAL);
		adm_params = kzalloc(sz, GFP_KERNEL);
		if (!adm_params) {
			ad_loge("%s, adm params memory alloc failed\n",
				__func__);
			return -ENOMEM;
		}
		adm_params->payload_size =
			sizeof(struct srs_trumedia_params_GLOBAL) +
			sizeof(struct adm_param_data_v5);
		param_id = SRS_TRUMEDIA_PARAMS;
		adm_params->params.param_size =
				sizeof(struct srs_trumedia_params_GLOBAL);
		glb_params = (struct srs_trumedia_params_GLOBAL *)
			((u8 *)adm_params +
			sizeof(struct adm_cmd_set_pp_params_inband_v5));
		memcpy(glb_params, srs_params,
			sizeof(struct srs_trumedia_params_GLOBAL));
		ad_logd("SRS - %s: Global params - 1 = %x, 2 = %x, 3 = %x, 4 = %x, 5 = %x, 6 = %x, 7 = %x, 8 = %x\n",
				__func__, (int)glb_params->v1,
				(int)glb_params->v2, (int)glb_params->v3,
				(int)glb_params->v4, (int)glb_params->v5,
				(int)glb_params->v6, (int)glb_params->v7,
				(int)glb_params->v8);
		break;
	}
	case SRS_ID_WOWHD: {
		struct srs_trumedia_params_WOWHD *whd_params = NULL;
		sz = sizeof(struct adm_cmd_set_pp_params_inband_v5) +
			sizeof(struct srs_trumedia_params_WOWHD);
		adm_params = kzalloc(sz, GFP_KERNEL);
		if (!adm_params) {
			ad_loge("%s, adm params memory alloc failed\n",
				__func__);
			return -ENOMEM;
		}
		adm_params->payload_size =
			sizeof(struct srs_trumedia_params_WOWHD) +
			sizeof(struct adm_param_data_v5);
		param_id = SRS_TRUMEDIA_PARAMS_WOWHD;
		adm_params->params.param_size =
				sizeof(struct srs_trumedia_params_WOWHD);
		whd_params = (struct srs_trumedia_params_WOWHD *)
			((u8 *)adm_params +
			sizeof(struct adm_cmd_set_pp_params_inband_v5));
		memcpy(whd_params, srs_params,
				sizeof(struct srs_trumedia_params_WOWHD));
		ad_logd("SRS - %s: WOWHD params - 1 = %x, 2 = %x, 3 = %x, 4 = %x, 5 = %x, 6 = %x, 7 = %x, 8 = %x, 9 = %x, 10 = %x, 11 = %x\n",
			 __func__, (int)whd_params->v1,
			(int)whd_params->v2, (int)whd_params->v3,
			(int)whd_params->v4, (int)whd_params->v5,
			(int)whd_params->v6, (int)whd_params->v7,
			(int)whd_params->v8, (int)whd_params->v9,
			(int)whd_params->v10, (int)whd_params->v11);
		break;
	}
	case SRS_ID_CSHP: {
		struct srs_trumedia_params_CSHP *chp_params = NULL;
		sz = sizeof(struct adm_cmd_set_pp_params_inband_v5) +
			sizeof(struct srs_trumedia_params_CSHP);
		adm_params = kzalloc(sz, GFP_KERNEL);
		if (!adm_params) {
			ad_loge("%s, adm params memory alloc failed\n",
				__func__);
			return -ENOMEM;
		}
		adm_params->payload_size =
			sizeof(struct srs_trumedia_params_CSHP) +
			sizeof(struct adm_param_data_v5);
		param_id = SRS_TRUMEDIA_PARAMS_CSHP;
		adm_params->params.param_size =
				sizeof(struct srs_trumedia_params_CSHP);
		chp_params = (struct srs_trumedia_params_CSHP *)
			((u8 *)adm_params +
			sizeof(struct adm_cmd_set_pp_params_inband_v5));
		memcpy(chp_params, srs_params,
				sizeof(struct srs_trumedia_params_CSHP));
		ad_logd("SRS - %s: CSHP params - 1 = %x, 2 = %x, 3 = %x, 4 = %x, 5 = %x, 6 = %x, 7 = %x, 8 = %x, 9 = %x\n",
				__func__, (int)chp_params->v1,
				(int)chp_params->v2, (int)chp_params->v3,
				(int)chp_params->v4, (int)chp_params->v5,
				(int)chp_params->v6, (int)chp_params->v7,
				(int)chp_params->v8, (int)chp_params->v9);
		break;
	}
	case SRS_ID_HPF: {
		struct srs_trumedia_params_HPF *hpf_params = NULL;
		sz = sizeof(struct adm_cmd_set_pp_params_inband_v5) +
			sizeof(struct srs_trumedia_params_HPF);
		adm_params = kzalloc(sz, GFP_KERNEL);
		if (!adm_params) {
			ad_loge("%s, adm params memory alloc failed\n",
				__func__);
			return -ENOMEM;
		}
		adm_params->payload_size =
			sizeof(struct srs_trumedia_params_HPF) +
			sizeof(struct adm_param_data_v5);
		param_id = SRS_TRUMEDIA_PARAMS_HPF;
		adm_params->params.param_size =
				sizeof(struct srs_trumedia_params_HPF);
		hpf_params = (struct srs_trumedia_params_HPF *)
			((u8 *)adm_params +
			sizeof(struct adm_cmd_set_pp_params_inband_v5));
		memcpy(hpf_params, srs_params,
			sizeof(struct srs_trumedia_params_HPF));
		ad_logd("SRS - %s: HPF params - 1 = %x\n", __func__,
				(int)hpf_params->v1);
		break;
	}
	case SRS_ID_PEQ: {
		struct srs_trumedia_params_PEQ *peq_params = NULL;
		sz = sizeof(struct adm_cmd_set_pp_params_inband_v5) +
			sizeof(struct srs_trumedia_params_PEQ);
		adm_params = kzalloc(sz, GFP_KERNEL);
		if (!adm_params) {
			ad_loge("%s, adm params memory alloc failed\n",
				__func__);
			return -ENOMEM;
		}
		adm_params->payload_size =
				sizeof(struct srs_trumedia_params_PEQ) +
				sizeof(struct adm_param_data_v5);
		adm_params->params.param_id = SRS_TRUMEDIA_PARAMS_PEQ;
		adm_params->params.param_size =
				sizeof(struct srs_trumedia_params_PEQ);
		peq_params = (struct srs_trumedia_params_PEQ *)
			((u8 *)adm_params +
			sizeof(struct adm_cmd_set_pp_params_inband_v5));
		memcpy(peq_params, srs_params,
				sizeof(struct srs_trumedia_params_PEQ));
		ad_logd("SRS - %s: PEQ params - 1 = %x 2 = %x, 3 = %x, 4 = %x\n",
			__func__, (int)peq_params->v1,
			(int)peq_params->v2, (int)peq_params->v3,
			(int)peq_params->v4);
		break;
	}
	case SRS_ID_HL: {
		struct srs_trumedia_params_HL *hl_params = NULL;
		sz = sizeof(struct adm_cmd_set_pp_params_inband_v5) +
			sizeof(struct srs_trumedia_params_HL);
		adm_params = kzalloc(sz, GFP_KERNEL);
		if (!adm_params) {
			ad_loge("%s, adm params memory alloc failed\n",
				__func__);
			return -ENOMEM;
		}
		adm_params->payload_size =
			sizeof(struct srs_trumedia_params_HL) +
			sizeof(struct adm_param_data_v5);
		param_id = SRS_TRUMEDIA_PARAMS_HL;
		adm_params->params.param_size =
			sizeof(struct srs_trumedia_params_HL);
		hl_params = (struct srs_trumedia_params_HL *)
			((u8 *)adm_params +
			sizeof(struct adm_cmd_set_pp_params_inband_v5));
		memcpy(hl_params, srs_params,
				sizeof(struct srs_trumedia_params_HL));
		ad_logd("SRS - %s: HL params - 1 = %x, 2 = %x, 3 = %x, 4 = %x, 5 = %x, 6 = %x, 7 = %x\n",
				__func__, (int)hl_params->v1,
				(int)hl_params->v2, (int)hl_params->v3,
				(int)hl_params->v4, (int)hl_params->v5,
				(int)hl_params->v6, (int)hl_params->v7);
		break;
	}
	default:
		goto fail_cmd;
	}

	adm_params->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	adm_params->hdr.src_svc = APR_SVC_ADM;
	adm_params->hdr.src_domain = APR_DOMAIN_APPS;
	adm_params->hdr.src_port = port_id;
	adm_params->hdr.dest_svc = APR_SVC_ADM;
	adm_params->hdr.dest_domain = APR_DOMAIN_ADSP;
	index = afe_get_port_index(port_id);
	if (index < 0 || index >= AFE_MAX_PORTS) {
		ad_loge("%s: invalid port idx %d portid %#x\n",
				__func__, index, port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	adm_params->hdr.dest_port = atomic_read(&this_adm.copp_id[index]);
	adm_params->hdr.token = port_id;
	adm_params->hdr.opcode = ADM_CMD_SET_PP_PARAMS_V5;
	if (outband) {
		adm_params->hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE, sizeof(
					      struct adm_cmd_set_pp_params_v5));
		adm_params->payload_addr_lsw = lower_32_bits(
						this_adm.outband_memmap.paddr);
		adm_params->payload_addr_msw = upper_32_bits(
						this_adm.outband_memmap.paddr);
		adm_params->mem_map_handle = atomic_read(&this_adm.
					mem_map_handles[ADM_SRS_TRUMEDIA]);
	} else {
		adm_params->hdr.pkt_size = sz;
		adm_params->payload_addr_lsw = 0;
		adm_params->payload_addr_msw = 0;
		adm_params->mem_map_handle = 0;

		adm_params->params.module_id = module_id;
		adm_params->params.param_id = param_id;
		adm_params->params.reserved = 0;
	}

	ad_logd("SRS - %s: Command was sent now check Q6 - port id = %d, size %d, module id %x, param id %x.\n",
			__func__, adm_params->hdr.dest_port,
			adm_params->payload_size, module_id, param_id);

	atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *)adm_params);
	if (ret < 0) {
		ad_loge("SRS - %s: ADM enable for port %d failed\n", __func__,
			port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	/* Wait for the callback with copp id */
	ret = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
			atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		ad_loge("%s: SRS set params timed out port = %d\n",
			__func__, port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}

fail_cmd:
	kfree(adm_params);
	return ret;
}

int adm_set_stereo_to_custom_stereo(int port_id, int copp_idx,
				    unsigned int session_id, char *params,
				    uint32_t params_length)
{
	struct adm_cmd_set_pspd_mtmx_strtr_params_v5 *adm_params = NULL;
	int sz, rc = 0, port_idx;

	ad_logd("%s\n", __func__);
	if (index < 0 || index >= AFE_MAX_PORTS) {
		ad_loge("%s: invalid port idx %d port_id %#x\n", __func__, index,
			port_id);
		return -EINVAL;
	}

	sz = sizeof(struct adm_cmd_set_pspd_mtmx_strtr_params_v5) +
		params_length;
	adm_params = kzalloc(sz, GFP_KERNEL);
	if (!adm_params) {
		ad_loge("%s, adm params memory alloc failed\n", __func__);
		return -ENOMEM;
	}

	memcpy(((u8 *)adm_params +
		sizeof(struct adm_cmd_set_pspd_mtmx_strtr_params_v5)),
		params, params_length);
	adm_params->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	adm_params->hdr.pkt_size = sz;
	adm_params->hdr.src_svc = APR_SVC_ADM;
	adm_params->hdr.src_domain = APR_DOMAIN_APPS;
	adm_params->hdr.src_port = port_id;
	adm_params->hdr.dest_svc = APR_SVC_ADM;
	adm_params->hdr.dest_domain = APR_DOMAIN_ADSP;
	adm_params->hdr.dest_port = 0; /* Ignored */;
	adm_params->hdr.token = 0;
	adm_params->hdr.opcode = ADM_CMD_SET_PSPD_MTMX_STRTR_PARAMS_V5;
	adm_params->payload_addr_lsw = 0;
	adm_params->payload_addr_msw = 0;
	adm_params->mem_map_handle = 0;
	adm_params->payload_size = params_length;
	/* direction RX as 0 */
	adm_params->direction = ADM_PATH_PLAYBACK;
	/* session id for this cmd to be applied on */
	adm_params->sessionid = session_id;
	adm_params->deviceid =
			atomic_read(&this_adm.copp.id[port_idx][copp_idx]);
	adm_params->reserved = 0;
	ad_logd("%s: deviceid %d, session_id %d, src_port %d, dest_port %d\n",
		__func__, adm_params->deviceid, adm_params->sessionid,
		adm_params->hdr.src_port, adm_params->hdr.dest_port);
	atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 0);
	rc = apr_send_pkt(this_adm.apr, (uint32_t *)adm_params);
	if (rc < 0) {
		ad_loge("%s: Set params failed port = %#x\n",
			__func__, port_id);
		rc = -EINVAL;
		goto set_stereo_to_custom_stereo_return;
	}
	/* Wait for the callback */
	rc = wait_event_timeout(this_adm.matrix_map_wait,
				atomic_read(&this_adm.matrix_map_stat),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		ad_loge("%s: Set params timed out port = %#x\n", __func__,
			port_id);
		rc = -EINVAL;
		goto set_stereo_to_custom_stereo_return;
	}
	rc = 0;
set_stereo_to_custom_stereo_return:
	kfree(adm_params);
	return rc;
}

int adm_dolby_dap_send_params(int port_id, int copp_idx, char *params,
			      uint32_t params_length)
{
	struct adm_cmd_set_pp_params_v5	*adm_params = NULL;
	int sz, rc = 0;
	int port_idx;

	ad_logd("%s\n", __func__);
	if (index < 0 || index >= AFE_MAX_PORTS) {
		ad_loge("%s: invalid port idx %d portid %#x\n",
			__func__, index, port_id);
		return -EINVAL;
	}

	sz = sizeof(struct adm_cmd_set_pp_params_v5) + params_length;
	adm_params = kzalloc(sz, GFP_KERNEL);
	if (!adm_params) {
		ad_loge("%s, adm params memory alloc failed", __func__);
		return -ENOMEM;
	}

	memcpy(((u8 *)adm_params + sizeof(struct adm_cmd_set_pp_params_v5)),
			params, params_length);
	adm_params->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	adm_params->hdr.pkt_size = sz;
	adm_params->hdr.src_svc = APR_SVC_ADM;
	adm_params->hdr.src_domain = APR_DOMAIN_APPS;
	adm_params->hdr.src_port = port_id;
	adm_params->hdr.dest_svc = APR_SVC_ADM;
	adm_params->hdr.dest_domain = APR_DOMAIN_ADSP;
	adm_params->hdr.dest_port =
			atomic_read(&this_adm.copp.id[port_idx][copp_idx]);
	adm_params->hdr.token = port_idx << 16 | copp_idx;
	adm_params->hdr.opcode = ADM_CMD_SET_PP_PARAMS_V5;
	adm_params->payload_addr_lsw = 0;
	adm_params->payload_addr_msw = 0;
	adm_params->mem_map_handle = 0;
	adm_params->payload_size = params_length;

	atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 0);
	rc = apr_send_pkt(this_adm.apr, (uint32_t *)adm_params);
	if (rc < 0) {
		ad_loge("%s: Set params failed port = %#x\n",
			__func__, port_id);
		rc = -EINVAL;
		goto dolby_dap_send_param_return;
	}
	/* Wait for the callback */
	rc = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
		atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		ad_loge("%s: Set params timed out port = %#x\n",
			 __func__, port_id);
		rc = -EINVAL;
		goto dolby_dap_send_param_return;
	}
	rc = 0;
dolby_dap_send_param_return:
	kfree(adm_params);
	return rc;
}

int adm_send_params_v5(int port_id, int copp_idx, char *params,
			      uint32_t params_length)
{
	struct adm_cmd_set_pp_params_v5	*adm_params = NULL;
	int rc = 0;
	int sz, port_idx;

	pr_debug("%s:\n", __func__);
	port_id = afe_convert_virtual_to_portid(port_id);
	port_idx = adm_validate_and_get_port_index(port_id);
	if (port_idx < 0) {
		pr_err("%s: Invalid port_id 0x%x\n", __func__, port_id);
		return -EINVAL;
	}

	sz = sizeof(struct adm_cmd_set_pp_params_v5) + params_length;
	adm_params = kzalloc(sz, GFP_KERNEL);
	if (!adm_params) {
		pr_err("%s, adm params memory alloc failed", __func__);
		return -ENOMEM;
	}

	memcpy(((u8 *)adm_params + sizeof(struct adm_cmd_set_pp_params_v5)),
			params, params_length);
	adm_params->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	adm_params->hdr.pkt_size = sz;
	adm_params->hdr.src_svc = APR_SVC_ADM;
	adm_params->hdr.src_domain = APR_DOMAIN_APPS;
	adm_params->hdr.src_port = port_id;
	adm_params->hdr.dest_svc = APR_SVC_ADM;
	adm_params->hdr.dest_domain = APR_DOMAIN_ADSP;
	adm_params->hdr.dest_port =
			atomic_read(&this_adm.copp.id[port_idx][copp_idx]);
	adm_params->hdr.token = port_idx << 16 | copp_idx;
	adm_params->hdr.opcode = ADM_CMD_SET_PP_PARAMS_V5;
	adm_params->payload_addr_lsw = 0;
	adm_params->payload_addr_msw = 0;
	adm_params->mem_map_handle = 0;
	adm_params->payload_size = params_length;

	atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 0);
	rc = apr_send_pkt(this_adm.apr, (uint32_t *)adm_params);
	if (rc < 0) {
		pr_err("%s: Set params failed port = 0x%x rc %d\n",
			__func__, port_id, rc);
		rc = -EINVAL;
		goto send_param_return;
	}
	/* Wait for the callback */
	rc = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
		atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: Set params timed out port = 0x%x\n",
			 __func__, port_id);
		rc = -EINVAL;
		goto send_param_return;
	}
	rc = 0;
send_param_return:
	kfree(adm_params);
	return rc;
}

int adm_get_params_v2(int port_id, int copp_idx, uint32_t module_id,
		      uint32_t param_id, uint32_t params_length,
		      char *params, uint32_t client_id)
{
	struct adm_cmd_get_pp_params_v5 *adm_params = NULL;
	int sz, rc = 0, i = 0;
	int port_idx, idx;
	int *params_data = (int *)params;

	if (index < 0 || index >= AFE_MAX_PORTS) {
		ad_loge("%s: invalid port idx %d portid %#x\n",
			__func__, index, port_id);
		return -EINVAL;
	}

	sz = sizeof(struct adm_cmd_get_pp_params_v5) + params_length;
	adm_params = kzalloc(sz, GFP_KERNEL);
	if (!adm_params) {
		ad_loge("%s, adm params memory alloc failed", __func__);
		return -ENOMEM;
	}

	memcpy(((u8 *)adm_params + sizeof(struct adm_cmd_get_pp_params_v5)),
		params, params_length);
	adm_params->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
	APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	adm_params->hdr.pkt_size = sz;
	adm_params->hdr.src_svc = APR_SVC_ADM;
	adm_params->hdr.src_domain = APR_DOMAIN_APPS;
	adm_params->hdr.src_port = port_id;
	adm_params->hdr.dest_svc = APR_SVC_ADM;
	adm_params->hdr.dest_domain = APR_DOMAIN_ADSP;
	adm_params->hdr.dest_port =
			atomic_read(&this_adm.copp.id[port_idx][copp_idx]);
	adm_params->hdr.token = port_idx << 16 | client_id << 8 | copp_idx;
	adm_params->hdr.opcode = ADM_CMD_GET_PP_PARAMS_V5;
	adm_params->data_payload_addr_lsw = 0;
	adm_params->data_payload_addr_msw = 0;
	adm_params->mem_map_handle = 0;
	adm_params->module_id = module_id;
	adm_params->param_id = param_id;
	adm_params->param_max_size = params_length;
	adm_params->reserved = 0;

	atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 0);
	rc = apr_send_pkt(this_adm.apr, (uint32_t *)adm_params);
	if (rc < 0) {
		ad_loge("%s: Failed to Get Params on port %d\n", __func__,
			port_id);
		rc = -EINVAL;
		goto adm_get_param_return;
	}
	/* Wait for the callback with copp id */
	rc = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
	atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		ad_loge("%s: get params timed out port = %d\n", __func__,
			port_id);
		rc = -EINVAL;
		goto adm_get_param_return;
	}

	idx = ADM_GET_PARAMETER_LENGTH * copp_idx;

	if (adm_get_parameters[idx] < 0) {
		pr_err("%s: Size is invalid %d\n", __func__,
			adm_get_parameters[idx]);
		rc = -EINVAL;
		goto adm_get_param_return;
	}
	if ((params_data) &&
		(ARRAY_SIZE(adm_get_parameters) >
		idx) &&
		(ARRAY_SIZE(adm_get_parameters) >=
		1+adm_get_parameters[idx]+idx) &&
		(params_length/sizeof(uint32_t) >=
		adm_get_parameters[idx])) {
		for (i = 0; i < adm_get_parameters[idx]; i++)
			params_data[i] = adm_get_parameters[1+i+idx];

	} else {
		pr_err("%s: Get param data not copied! get_param array size %zd, index %d, params array size %zd, index %d\n",
		__func__, ARRAY_SIZE(adm_get_parameters),
		(1+adm_get_parameters[idx]+idx),
		params_length/sizeof(int),
		adm_get_parameters[idx]);
	}
	rc = 0;
adm_get_param_return:
	kfree(adm_params);

	return rc;
}

int adm_get_params(int port_id, int copp_idx, uint32_t module_id,
		   uint32_t param_id, uint32_t params_length, char *params)
{
	return adm_get_params_v2(port_id, copp_idx, module_id, param_id,
				 params_length, params, 0);
}

int adm_get_pp_topo_module_list(int port_id, int copp_idx, int32_t param_length,
				char *params)
{
	struct adm_cmd_get_pp_topo_module_list_t *adm_pp_module_list = NULL;
	int sz, rc = 0, i = 0;
	int port_idx, idx;
	int32_t *params_data = (int32_t *)params;
	int *topo_list;

	if (data->payload_size >= 8)
		ad_logd("%s: code = 0x%x PL#0[%x], PL#1[%x], size = %d\n",
			__func__, data->opcode, payload[0], payload[1],
			data->payload_size);
	else if (data->payload_size >= 4)
		ad_logd("%s: code = 0x%x PL#0[%x], size = %d\n",
			__func__, data->opcode, payload[0],
			data->payload_size);
	else
		ad_logd("%s: code = 0x%x, size = %d\n",
			__func__, data->opcode, data->payload_size);
}

void adm_set_multi_ch_map(char *channel_map)
{
	memcpy(multi_ch_map.channel_mapping, channel_map,
		PCM_FORMAT_MAX_NUM_CHANNEL);
	multi_ch_map.set_channel_map = true;
}

void adm_get_multi_ch_map(char *channel_map)
{
	if (multi_ch_map.set_channel_map) {
		memcpy(channel_map, multi_ch_map.channel_mapping,
			PCM_FORMAT_MAX_NUM_CHANNEL);
	}
}

static int32_t adm_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *payload;
	int i, j, port_idx, copp_idx, idx, client_id;

	if (data == NULL) {
		ad_loge("%s: data paramter is null\n", __func__);
		return -EINVAL;
	}

	payload = data->payload;

	if (data->opcode == RESET_EVENTS) {
		ad_logd("adm_callback: Reset event is received: %d %d apr[%p]\n",
				data->reset_event, data->reset_proc,
				this_adm.apr);
		if (this_adm.apr) {
			apr_reset(this_adm.apr);
			for (i = 0; i < AFE_MAX_PORTS; i++) {
				for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
					atomic_set(&this_adm.copp.id[i][j],
						   RESET_COPP_ID);
					atomic_set(&this_adm.copp.cnt[i][j], 0);
					atomic_set(
					   &this_adm.copp.topology[i][j], 0);
					atomic_set(&this_adm.copp.mode[i][j],
						   0);
					atomic_set(&this_adm.copp.stat[i][j],
						   0);
					atomic_set(&this_adm.copp.rate[i][j],
						   0);
					atomic_set(
					    &this_adm.copp.bit_width[i][j], 0);
					atomic_set(
					    &this_adm.copp.app_type[i][j], 0);
					atomic_set(
					   &this_adm.copp.acdb_id[i][j], 0);
				}
			}
			this_adm.apr = NULL;
			cal_utils_clear_cal_block_q6maps(ADM_MAX_CAL_TYPES,
				this_adm.cal_data);
			mutex_lock(&this_adm.cal_data
				[ADM_CUSTOM_TOP_CAL]->lock);
			this_adm.set_custom_topology = 1;
			mutex_unlock(&this_adm.cal_data[
				ADM_CUSTOM_TOP_CAL]->lock);
			rtac_clear_mapping(ADM_RTAC_CAL);
		}
		ad_logd("Resetting calibration blocks");
		for (i = 0; i < MAX_AUDPROC_TYPES; i++) {
			/* Device calibration */
			this_adm.mem_addr_audproc[i].cal_size = 0;
			this_adm.mem_addr_audproc[i].cal_kvaddr = 0;
			this_adm.mem_addr_audproc[i].cal_paddr = 0;

			/* Volume calibration */
			this_adm.mem_addr_audvol[i].cal_size = 0;
			this_adm.mem_addr_audvol[i].cal_kvaddr = 0;
			this_adm.mem_addr_audvol[i].cal_paddr = 0;
		}
		return 0;
	}

	adm_callback_debug_print(data);
	if (data->payload_size) {
		index = q6audio_get_port_index(data->token);
		if (index < 0 || index >= AFE_MAX_PORTS) {
			ad_loge("%s: invalid port idx %d token %d\n",
					__func__, index, data->token);
			return 0;
		}
		if (data->opcode == APR_BASIC_RSP_RESULT) {
			ad_logd("APR_BASIC_RSP_RESULT id %x\n", payload[0]);
			if (payload[1] != 0) {
				ad_loge("%s: cmd = 0x%x returned error = 0x%x\n",
					__func__, payload[0], payload[1]);
			}
			switch (payload[0]) {
			case ADM_CMD_SET_PP_PARAMS_V5:
				ad_logd("%s: ADM_CMD_SET_PP_PARAMS_V5\n",
					__func__);
				if (client_id == ADM_CLIENT_ID_SOURCE_TRACKING)
					this_adm.sourceTrackingData.
						apr_cmd_status = payload[1];
				else if (rtac_make_adm_callback(payload,
							data->payload_size))
					break;
				/*
				 * if soft volume is called and already
				 * interrupted break out of the sequence here
				 */
			case ADM_CMD_DEVICE_CLOSE_V5:
				pr_debug("%s: Basic callback received, wake up.\n",
					__func__);
				atomic_set(&this_adm.copp.stat[port_idx]
							      [copp_idx], 1);
				wake_up(
				&this_adm.copp.wait[port_idx][copp_idx]);
				break;
			case ADM_CMD_ADD_TOPOLOGIES:
				ad_logd("%s: Basic callback received, wake up.\n",
					__func__);
				atomic_set(&this_adm.matrix_map_stat, 1);
				wake_up(&this_adm.matrix_map_wait);
				break;
			case ADM_CMD_SHARED_MEM_UNMAP_REGIONS:
				pr_debug("%s: ADM_CMD_SHARED_MEM_UNMAP_REGIONS\n",
					__func__);
				atomic_set(&this_adm.adm_stat, 1);
				wake_up(&this_adm.adm_wait);
				break;
			case ADM_CMD_SHARED_MEM_MAP_REGIONS:
				ad_logd("%s: ADM_CMD_SHARED_MEM_MAP_REGIONS\n",
					__func__);
				/* Should only come here if there is an APR */
				/* error or malformed APR packet. Otherwise */
				/* response will be returned as */
				if (payload[1] != 0) {
					ad_loge("%s: ADM map error, resuming\n",
						__func__);
					atomic_set(&this_adm.adm_stat, 1);
					wake_up(&this_adm.adm_wait);
				}
				break;
			case ADM_CMD_GET_PP_PARAMS_V5:
				ad_logd("%s: ADM_CMD_GET_PP_PARAMS_V5\n",
					__func__);
				/* Should only come here if there is an APR */
				/* error or malformed APR packet. Otherwise */
				/* response will be returned as */
				/* ADM_CMDRSP_GET_PP_PARAMS_V5 */
				if (payload[1] != 0) {
					ad_loge("%s: ADM get param error = %d, resuming\n",
						__func__, payload[1]);
					rtac_make_adm_callback(payload,
						data->payload_size);
				}
				break;
			case ADM_CMD_SET_PSPD_MTMX_STRTR_PARAMS_V5:
				ad_logd("%s:ADM_CMD_SET_PSPD_MTMX_STRTR_PARAMS_V5\n",
					__func__);
				atomic_set(&this_adm.copp.stat[port_idx]
							      [copp_idx], 1);
				wake_up(
				&this_adm.copp.wait[port_idx][copp_idx]);
				break;
			case ADM_CMD_GET_PP_TOPO_MODULE_LIST:
				pr_debug("%s:ADM_CMD_GET_PP_TOPO_MODULE_LIST\n",
					 __func__);
				if (payload[1] != 0)
					pr_err("%s: ADM get topo list error = %d,\n",
						__func__, payload[1]);
				break;
			default:
				ad_loge("%s: Unknown Cmd: 0x%x\n", __func__,
								payload[0]);
				break;
			}
			return 0;
		}

		switch (data->opcode) {
		case ADM_CMDRSP_DEVICE_OPEN_V5: {
			struct adm_cmd_rsp_device_open_v5 *open =
			(struct adm_cmd_rsp_device_open_v5 *)data->payload;

			if (open->copp_id == INVALID_COPP_ID) {
				ad_loge("%s: invalid coppid rxed %d\n",
					__func__, open->copp_id);
				atomic_set(&this_adm.copp.stat[port_idx]
							      [copp_idx], 1);
				wake_up(
				&this_adm.copp.wait[port_idx][copp_idx]);
				break;
			}
			if (atomic_read(&this_adm.copp_perf_mode[index])) {
				atomic_set(&this_adm.copp_low_latency_id[index],
						open->copp_id);
			} else {
				atomic_set(&this_adm.copp_id[index],
					open->copp_id);
			}
			atomic_set(&this_adm.copp_stat[index], 1);
			ad_logd("%s: coppid rxed=%d\n", __func__,
							open->copp_id);
			wake_up(&this_adm.wait[index]);
			}
			break;
		case ADM_CMDRSP_GET_PP_PARAMS_V5:
			ad_logd("%s: ADM_CMDRSP_GET_PP_PARAMS_V5\n", __func__);
			if (payload[0] != 0)
				ad_loge("%s: ADM_CMDRSP_GET_PP_PARAMS_V5 returned error = 0x%x\n",
					__func__, payload[0]);
			if (client_id == ADM_CLIENT_ID_SOURCE_TRACKING)
				this_adm.sourceTrackingData.apr_cmd_status =
								payload[0];
			else if (rtac_make_adm_callback(payload,
					data->payload_size))
				break;

			if (data->payload_size > (4 * sizeof(uint32_t))) {
				adm_get_parameters[0] = payload[3];
				ad_logd("GET_PP PARAM:received parameter length: %x\n",
						adm_get_parameters[0]);
				/* storing param size then params */
				for (i = 0; i < payload[3] /
						sizeof(uint32_t); i++)
					adm_get_parameters[idx+1+i] =
							payload[4+i];
			} else if (payload[0] == 0) {
				adm_get_parameters[idx] = -1;
				pr_err("%s: Out of band case, setting size to %d\n",
					__func__, adm_get_parameters[idx]);
			} else {
				adm_get_parameters[idx] = -1;
				pr_err("%s: GET_PP_PARAMS failed, setting size to %d\n",
					__func__, adm_get_parameters[idx]);
			}
			atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 1);
			wake_up(&this_adm.copp.wait[port_idx][copp_idx]);
			break;
		case ADM_CMDRSP_GET_PP_TOPO_MODULE_LIST:
			pr_debug("%s: ADM_CMDRSP_GET_PP_TOPO_MODULE_LIST\n",
				 __func__);
			if (payload[0] != 0) {
				pr_err("%s: ADM_CMDRSP_GET_PP_TOPO_MODULE_LIST",
					 __func__);
				pr_err(":err = 0x%x\n", payload[0]);
			} else if (payload[1] >
				   ((ADM_GET_TOPO_MODULE_LIST_LENGTH /
				   sizeof(uint32_t)) - 1)) {
				pr_err("%s: ADM_CMDRSP_GET_PP_TOPO_MODULE_LIST",
					 __func__);
				pr_err(":size = %d\n", payload[1]);
			} else {
				idx = ADM_GET_TOPO_MODULE_LIST_LENGTH *
					copp_idx;
				pr_debug("%s:Num modules payload[1] %d\n",
					 __func__, payload[1]);
				adm_module_topo_list[idx] = payload[1];
				for (i = 1; i <= payload[1]; i++) {
					adm_module_topo_list[idx+i] =
						payload[1+i];
					pr_debug("%s:payload[%d] = %x\n",
						 __func__, (i+1), payload[1+i]);
				}
			}
			atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 1);
			wake_up(&this_adm.copp.wait[port_idx][copp_idx]);
			break;
		case ADM_CMDRSP_SHARED_MEM_MAP_REGIONS:
			ad_logd("%s: ADM_CMDRSP_SHARED_MEM_MAP_REGIONS\n",
				__func__);
			atomic_set(&this_adm.mem_map_handles[
				   atomic_read(&this_adm.mem_map_index)],
				   *payload);
			atomic_set(&this_adm.adm_stat, 1);
			wake_up(&this_adm.adm_wait);
			break;
		default:
			ad_loge("%s: Unknown cmd:0x%x\n", __func__,
							data->opcode);
			break;
		}
	}
	return 0;
}

static int adm_memory_map_regions(phys_addr_t *buf_add, uint32_t mempool_id,
			   uint32_t *bufsz, uint32_t bufcnt)
{
	struct  avs_cmd_shared_mem_map_regions *mmap_regions = NULL;
	struct  avs_shared_map_region_payload *mregions = NULL;
	void    *mmap_region_cmd = NULL;
	void    *payload = NULL;
	int     ret = 0;
	int     i = 0;
	int     cmd_size = 0;
	static DEFINE_RATELIMIT_STATE(rl, HZ/2, 1);

	get_adm_custom_topology(&cal_block);
	if (cal_block.cal_size == 0) {
		ad_logd("%s: no cal to send addr= 0x%pa\n",
				__func__, &cal_block.cal_paddr);
		goto done;
	}

	index = afe_get_port_index(port_id);
	if (index < 0 || index >= AFE_MAX_PORTS) {
		ad_loge("%s: invalid port idx %d portid %#x\n",
				__func__, index, port_id);
		goto done;
	}

	ret = wait_event_timeout(this_adm.adm_wait,
				 atomic_read(&this_adm.adm_stat),
				 5 * HZ);
	if (!ret) {
		pr_err("%s: timeout. waited for memory_unmap\n",
		       __func__);
		ret = -EINVAL;
		goto fail_cmd;
	} else {
		pr_debug("%s: Unmap handle 0x%x succeeded\n", __func__,
			 unmap_regions.mem_map_handle);
	}
fail_cmd:
	return ret;
}

static void remap_cal_data(struct cal_block_data *cal_block, int cal_index)
{
	int ret = 0;

		result = adm_memory_map_regions(port_id,
				&cal_block.cal_paddr, 0, &size, 1);
		if (result < 0) {
			ad_loge("%s: mmap did not work! addr = 0x%pa, size = %zd\n",
				__func__, &cal_block.cal_paddr,
			       cal_block.cal_size);
			goto done;
		}
		cal_block->map_data.q6map_handle = atomic_read(&this_adm.
			mem_map_handles[cal_index]);
	}
done:
	return;
}

static void send_adm_custom_topology(void)
{
	struct cal_block_data		*cal_block = NULL;
	struct cmd_set_topologies	adm_top;
	int				cal_index = ADM_CUSTOM_TOP_CAL;
	int				result;

	if (this_adm.cal_data[cal_index] == NULL)
		goto done;

	mutex_lock(&this_adm.cal_data[cal_index]->lock);
	if (!this_adm.set_custom_topology)
		goto unlock;
	this_adm.set_custom_topology = 0;

	cal_block = cal_utils_get_only_cal_block(this_adm.cal_data[cal_index]);
	if (cal_block == NULL)
		goto unlock;

	pr_debug("%s: Sending cal_index %d\n", __func__, cal_index);

	remap_cal_data(cal_block, cal_index);
	atomic_set(&this_adm.mem_map_index, cal_index);
	atomic_set(&this_adm.mem_map_handles[cal_index],
		cal_block->map_data.q6map_handle);

	if (cal_block->cal_data.size == 0) {
		pr_debug("%s: No ADM cal to send\n", __func__);
		goto unlock;
	}

	adm_top.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(20), APR_PKT_VER);
	adm_top.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(adm_top));
	adm_top.hdr.src_svc = APR_SVC_ADM;
	adm_top.hdr.src_domain = APR_DOMAIN_APPS;
	adm_top.hdr.src_port = 0;
	adm_top.hdr.dest_svc = APR_SVC_ADM;
	adm_top.hdr.dest_domain = APR_DOMAIN_ADSP;
	adm_top.hdr.dest_port = 0;
	adm_top.hdr.token = 0;
	adm_top.hdr.opcode = ADM_CMD_ADD_TOPOLOGIES;
	adm_top.payload_addr_lsw = lower_32_bits(cal_block.cal_paddr);
	adm_top.payload_addr_msw = upper_32_bits(cal_block.cal_paddr);
	adm_top.mem_map_handle =
		atomic_read(&this_adm.mem_map_cal_handles[ADM_CUSTOM_TOP_CAL]);
	adm_top.payload_size = cal_block.cal_size;

	atomic_set(&this_adm.copp_stat[index], 0);
	ad_logd("%s: Sending ADM_CMD_ADD_TOPOLOGIES payload = 0x%x, size = %d\n",
		__func__, adm_top.payload_addr_lsw,
		adm_top.payload_size);
	result = apr_send_pkt(this_adm.apr, (uint32_t *)&adm_top);
	if (result < 0) {
		ad_loge("%s: Set topologies failed port = 0x%x payload = 0x%pa\n",
			__func__, port_id, &cal_block.cal_paddr);
		goto done;
	}
	/* Wait for the callback */
	result = wait_event_timeout(this_adm.adm_wait,
				    atomic_read(&this_adm.adm_stat),
				    msecs_to_jiffies(TIMEOUT_MS));
	if (!result) {
		ad_loge("%s: Set topologies timed out port = 0x%x, payload = 0x%pa\n",
			__func__, port_id, &cal_block.cal_paddr);
		goto done;
	}
unlock:
	mutex_unlock(&this_adm.cal_data[cal_index]->lock);
done:
	return;
}

static int send_adm_cal_block(int port_id, int copp_idx,
			      struct cal_block_data *cal_block, int perf_mode,
			      int app_type, int acdb_id, int sample_rate)
{
	s32				result = 0;
	struct adm_cmd_set_pp_params_v5	adm_params;
	int index = afe_get_port_index(port_id);
	if (index < 0 || index >= AFE_MAX_PORTS) {
		ad_loge("%s: invalid port idx %d portid %#x\n",
				__func__, index, port_id);
		return 0;
	}

	ad_logd("%s: Port id %#x, index %d\n", __func__, port_id, index);

	if (!aud_cal || aud_cal->cal_size == 0) {
		ad_logd("%s: No ADM cal to send for port_id = %#x!\n",
			__func__, port_id);
		result = -EINVAL;
		goto done;
	}

	if (perf_mode == LEGACY_PCM_MODE &&
		((atomic_read(&this_adm.copp.topology[port_idx][copp_idx])) ==
			DS2_ADM_COPP_TOPOLOGY_ID)) {
		pr_err("%s: perf_mode %d, topology 0x%x\n", __func__, perf_mode,
			atomic_read(
				&this_adm.copp.topology[port_idx][copp_idx]));
		goto done;
	}

	adm_params.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(20), APR_PKT_VER);
	adm_params.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(adm_params));
	adm_params.hdr.src_svc = APR_SVC_ADM;
	adm_params.hdr.src_domain = APR_DOMAIN_APPS;
	adm_params.hdr.src_port = port_id;
	adm_params.hdr.dest_svc = APR_SVC_ADM;
	adm_params.hdr.dest_domain = APR_DOMAIN_ADSP;

	adm_params.hdr.token = port_idx << 16 | copp_idx;
	adm_params.hdr.dest_port =
			atomic_read(&this_adm.copp.id[port_idx][copp_idx]);
	adm_params.hdr.opcode = ADM_CMD_SET_PP_PARAMS_V5;
	adm_params.payload_addr_lsw = lower_32_bits(aud_cal->cal_paddr);
	adm_params.payload_addr_msw = upper_32_bits(aud_cal->cal_paddr);
	adm_params.mem_map_handle = atomic_read(&this_adm.mem_map_cal_handles[
				atomic_read(&this_adm.mem_map_cal_index)]);
	adm_params.payload_size = aud_cal->cal_size;

	atomic_set(&this_adm.copp_stat[index], 0);
	ad_logd("%s: Sending SET_PARAMS payload = 0x%x, size = %d\n",
		__func__, adm_params.payload_addr_lsw,
		adm_params.payload_size);
	result = apr_send_pkt(this_adm.apr, (uint32_t *)&adm_params);
	if (result < 0) {
		ad_loge("%s: Set params failed port = %#x payload = 0x%pa\n",
			__func__, port_id, &aud_cal->cal_paddr);
		result = -EINVAL;
		goto done;
	}
	/* Wait for the callback */
	result = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
		atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!result) {
		ad_loge("%s: Set params timed out port = %#x, payload = 0x%pa\n",
			__func__, port_id, &aud_cal->cal_paddr);
		result = -EINVAL;
		goto done;
	}

done:
	return result;
}

static struct cal_block_data *adm_find_cal_by_path(int cal_index, int path)
{
	struct list_head		*ptr, *next;
	struct cal_block_data		*cal_block = NULL;
	struct audio_cal_info_audproc	*audproc_cal_info = NULL;
	struct audio_cal_info_audvol	*audvol_cal_info = NULL;
	pr_debug("%s:\n", __func__);

	list_for_each_safe(ptr, next,
		&this_adm.cal_data[cal_index]->cal_blocks) {

		cal_block = list_entry(ptr,
			struct cal_block_data, list);

		if (cal_index == ADM_AUDPROC_CAL) {
			audproc_cal_info = cal_block->cal_info;
			if (audproc_cal_info->path == path)
				return cal_block;
		} else if (cal_index == ADM_AUDVOL_CAL) {
			audvol_cal_info = cal_block->cal_info;
			if (audvol_cal_info->path == path)
				return cal_block;
		}
	}
	pr_debug("%s: Can't find ADM cal for cal_index %d, path %d\n",
		__func__, cal_index, path);
	return NULL;
}

static struct cal_block_data *adm_find_cal_by_app_type(int cal_index, int path,
								int app_type)
{
	int			result = 0;
	s32			acdb_path;
	struct acdb_cal_block	aud_cal;
	int			size;
	ad_logd("%s\n", __func__);

	/* Maps audio_dev_ctrl path definition to ACDB definition */
	acdb_path = path - 1;
	if (acdb_path == TX_CAL)
		size = 4096 * 4;
	else
		size = 4096;

	ad_logd("%s: Sending audproc cal\n", __func__);
	get_audproc_cal(acdb_path, &aud_cal);

	/* map & cache buffers used */
	atomic_set(&this_adm.mem_map_cal_index, acdb_path);
	if (((this_adm.mem_addr_audproc[acdb_path].cal_paddr !=
		aud_cal.cal_paddr)  && (aud_cal.cal_size > 0)) ||
		(aud_cal.cal_size >
		this_adm.mem_addr_audproc[acdb_path].cal_size)) {

		if (this_adm.mem_addr_audproc[acdb_path].cal_paddr != 0)
			adm_memory_unmap_regions(port_id);

		result = adm_memory_map_regions(port_id, &aud_cal.cal_paddr,
						0, &size, 1);
		if (result < 0) {
			ad_loge("ADM audproc mmap did not work! path = %d, addr = 0x%pa, size = %zd\n",
				acdb_path, &aud_cal.cal_paddr,
				aud_cal.cal_size);
		} else {
			this_adm.mem_addr_audproc[acdb_path].cal_paddr =
							aud_cal.cal_paddr;
			this_adm.mem_addr_audproc[acdb_path].cal_size = size;
		}
	}
	pr_debug("%s: Can't find ADM cali for cal_index %d, path %d, app %d, defaulting to search by path\n",
		__func__, cal_index, path, app_type);
	return adm_find_cal_by_path(cal_index, path);
}

	if (!send_adm_cal_block(port_id, &aud_cal, perf_mode))
		ad_logd("%s: Audproc cal sent for port id: %#x, path %d\n",
			__func__, port_id, acdb_path);
	else
		ad_logd("%s: Audproc cal not sent for port id: %#x, path %d\n",
			__func__, port_id, acdb_path);

	ad_logd("%s: Sending audvol cal\n", __func__);
	get_audvol_cal(acdb_path, &aud_cal);

	/* map & cache buffers used */
	atomic_set(&this_adm.mem_map_cal_index,
		(acdb_path + MAX_AUDPROC_TYPES));
	if (((this_adm.mem_addr_audvol[acdb_path].cal_paddr !=
		aud_cal.cal_paddr)  && (aud_cal.cal_size > 0))  ||
		(aud_cal.cal_size >
		this_adm.mem_addr_audvol[acdb_path].cal_size)) {

		if (this_adm.mem_addr_audvol[acdb_path].cal_paddr != 0)
			adm_memory_unmap_regions(port_id);

		result = adm_memory_map_regions(port_id, &aud_cal.cal_paddr,
						0, &size, 1);
		if (result < 0) {
			ad_loge("ADM audvol mmap did not work! path = %d, addr = 0x%pa, size = %zd\n",
				acdb_path, &aud_cal.cal_paddr,
				aud_cal.cal_size);
		} else {
			this_adm.mem_addr_audvol[acdb_path].cal_paddr =
							aud_cal.cal_paddr;
			this_adm.mem_addr_audvol[acdb_path].cal_size = size;
		}
	}

	if (!send_adm_cal_block(port_id, &aud_cal, perf_mode))
		ad_logd("%s: Audvol cal sent for port id: %#x, path %d\n",
			__func__, port_id, acdb_path);
	else
		ad_logd("%s: Audvol cal not sent for port id: %#x, path %d\n",
			__func__, port_id, acdb_path);
}

static void send_adm_cal_type(int cal_index, int path, int port_id,
			      int copp_idx, int perf_mode, int app_type,
			      int acdb_id, int sample_rate)
{
	int	result = 0;
	ad_logd("%s\n", __func__);

	if (cal_block == NULL) {
		ad_loge("%s: cal_block is NULL!\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	if (cal_block->cal_data.paddr == 0) {
		ad_logd("%s: No address to map!\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	if (cal_block->map_data.map_size == 0) {
		ad_logd("%s: map size is 0!\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	/* valid port ID needed for callback use primary I2S */
	atomic_set(&this_adm.mem_map_cal_index, ADM_RTAC);
	result = adm_memory_map_regions(PRIMARY_I2S_RX,
			&cal_block->cal_data.paddr, 0,
			&cal_block->map_data.map_size, 1);
	if (result < 0) {
		ad_loge("%s: RTAC mmap did not work! addr = 0x%pa, size = %d\n",
			__func__, &cal_block->cal_data.paddr,
			cal_block->map_data.map_size);
		goto done;
	}

	cal_block->map_data.map_handle = atomic_read(
		&this_adm.mem_map_cal_handles[ADM_RTAC]);
done:
	return result;
}

int adm_unmap_rtac_block(uint32_t *mem_map_handle)
{
	int	result = 0;
	ad_logd("%s\n", __func__);

	if (mem_map_handle == NULL) {
		ad_logd("%s: Map handle is NULL, nothing to unmap\n",
			__func__);
		goto done;
	}

	if (*mem_map_handle == 0) {
		ad_logd("%s: Map handle is 0, nothing to unmap\n",
			__func__);
		goto done;
	}

	if (*mem_map_handle != atomic_read(
			&this_adm.mem_map_cal_handles[ADM_RTAC])) {
		ad_loge("%s: Map handles do not match! Unmapping RTAC, RTAC map 0x%x, ADM map 0x%x\n",
			__func__, *mem_map_handle, atomic_read(
			&this_adm.mem_map_cal_handles[ADM_RTAC]));

		/* if mismatch use handle passed in to unmap */
		atomic_set(&this_adm.mem_map_cal_handles[ADM_RTAC],
			   *mem_map_handle);
	}

	/* valid port ID needed for callback use primary I2S */
	atomic_set(&this_adm.mem_map_cal_index, ADM_RTAC);
	result = adm_memory_unmap_regions(PRIMARY_I2S_RX);
	if (result < 0) {
		ad_logd("%s: adm_memory_unmap_regions failed, error %d\n",
			__func__, result);
	} else {
		atomic_set(&this_adm.mem_map_cal_handles[ADM_RTAC], 0);
		*mem_map_handle = 0;
	}
done:
	return result;
}

static void send_adm_cal(int port_id, int copp_idx, int path, int perf_mode,
			 int app_type, int acdb_id, int sample_rate)
{
	pr_debug("%s:\n", __func__);

			/* valid port ID needed for callback use primary I2S */
			atomic_set(&this_adm.mem_map_cal_index, i);
			result2 = adm_memory_unmap_regions(PRIMARY_I2S_RX);
			if (result2 < 0) {
				ad_loge("%s: adm_memory_unmap_regions failed, err %d\n",
						__func__, result2);
				result = result2;
			} else {
				atomic_set(&this_adm.mem_map_cal_handles[i],
					0);
			}
		}
	}
	return result;
}

int adm_connect_afe_port(int mode, int session_id, int port_id)
{
	struct adm_cmd_connect_afe_port_v5	cmd;
	int ret = 0;
	int port_idx, copp_idx = 0;

	ad_logd("%s: port %d session id:%d mode:%d\n", __func__,
				port_id, session_id, mode);

	port_id = afe_convert_virtual_to_portid(port_id);

	if (afe_validate_port(port_id) < 0) {
		ad_loge("%s port idi[%d] is invalid\n", __func__, port_id);
		return -ENODEV;
	}

	if (this_adm.apr == NULL) {
		this_adm.apr = apr_register("ADSP", "ADM", adm_callback,
						0xFFFFFFFF, &this_adm);
		if (this_adm.apr == NULL) {
			ad_loge("%s: Unable to register ADM\n", __func__);
			ret = -ENODEV;
			return ret;
		}
		rtac_set_adm_handle(this_adm.apr);
	}
	index = afe_get_port_index(port_id);
	ad_logd("%s: Port ID %#x, index %d\n", __func__, port_id, index);

	cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.src_svc = APR_SVC_ADM;
	cmd.hdr.src_domain = APR_DOMAIN_APPS;
	cmd.hdr.src_port = port_id;
	cmd.hdr.dest_svc = APR_SVC_ADM;
	cmd.hdr.dest_domain = APR_DOMAIN_ADSP;
	cmd.hdr.dest_port = 0; /* Ignored */
	cmd.hdr.token = port_idx << 16 | copp_idx;
	cmd.hdr.opcode = ADM_CMD_CONNECT_AFE_PORT_V5;

	cmd.mode = mode;
	cmd.session_id = session_id;
	cmd.afe_port_id = port_id;

	atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *)&cmd);
	if (ret < 0) {
		ad_loge("%s:ADM enable for port %#x failed\n",
					__func__, port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	/* Wait for the callback with copp id */
	ret = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
		atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		ad_loge("%s ADM connect AFE failed for port %#x\n", __func__,
							port_id);
		ret = -EINVAL;
		goto fail_cmd;
	}
	atomic_inc(&this_adm.copp.cnt[port_idx][copp_idx]);
	return 0;

fail_cmd:

	return ret;
}

int adm_open(int port_id, int path, int rate, int channel_mode, int topology,
	     int perf_mode, uint16_t bit_width, int app_type, int acdb_id)
{
	struct adm_cmd_device_open_v5	open;
	int ret = 0;
	int port_idx, copp_idx, flags;
	int tmp_port = q6audio_get_port_id(port_id);

	ad_logd("%s: port %#x path:%d rate:%d mode:%d perf_mode:%d\n",
		 __func__, port_id, path, rate, channel_mode, perf_mode);

	port_id = q6audio_convert_virtual_to_portid(port_id);

	if (q6audio_validate_port(port_id) < 0) {
		ad_loge("%s port idi[%#x] is invalid\n", __func__, port_id);
		return -ENODEV;
	}

	index = q6audio_get_port_index(port_id);
	ad_logd("%s: Port ID %#x, index %d\n", __func__, port_id, index);

	if (this_adm.apr == NULL) {
		this_adm.apr = apr_register("ADSP", "ADM", adm_callback,
						0xFFFFFFFF, &this_adm);
		if (this_adm.apr == NULL) {
			ad_loge("%s: Unable to register ADM\n", __func__);
			ret = -ENODEV;
			return ret;
		}
		rtac_set_adm_handle(this_adm.apr);
	}

	if (perf_mode == ULTRA_LOW_LATENCY_PCM_MODE) {
		flags = ADM_ULTRA_LOW_LATENCY_DEVICE_SESSION;
		topology = NULL_COPP_TOPOLOGY;
		rate = ULL_SUPPORTED_SAMPLE_RATE;
		bit_width = ULL_SUPPORTED_BITS_PER_SAMPLE;
	} else if (perf_mode == LOW_LATENCY_PCM_MODE) {
		flags = ADM_LOW_LATENCY_DEVICE_SESSION;
		if ((topology == DOLBY_ADM_COPP_TOPOLOGY_ID) ||
		    (topology == DS2_ADM_COPP_TOPOLOGY_ID) ||
		    (topology == SRS_TRUMEDIA_TOPOLOGY_ID))
			topology = DEFAULT_COPP_TOPOLOGY;
	} else {
		if (path == ADM_PATH_COMPRESSED_RX)
			flags = 0;
		else
			flags = ADM_LEGACY_DEVICE_SESSION;
	}

	if ((topology == VPM_TX_SM_ECNS_COPP_TOPOLOGY) ||
	    (topology == VPM_TX_DM_FLUENCE_COPP_TOPOLOGY) ||
	    (topology == VPM_TX_DM_RFECNS_COPP_TOPOLOGY))
		rate = 16000;

	copp_idx = adm_get_idx_if_copp_exists(port_idx, topology, perf_mode,
						rate, bit_width, app_type);
	if (copp_idx < 0) {
		copp_idx = adm_get_next_available_copp(port_idx);
		if (copp_idx >= MAX_COPPS_PER_PORT) {
			pr_err("%s: exceeded copp id %d\n",
				 __func__, copp_idx);
			return -EINVAL;
		} else {
			atomic_set(&this_adm.copp.cnt[port_idx][copp_idx], 0);
			atomic_set(&this_adm.copp.topology[port_idx][copp_idx],
				   topology);
			atomic_set(&this_adm.copp.mode[port_idx][copp_idx],
				   perf_mode);
			atomic_set(&this_adm.copp.rate[port_idx][copp_idx],
				   rate);
			atomic_set(&this_adm.copp.bit_width[port_idx][copp_idx],
				   bit_width);
			atomic_set(&this_adm.copp.app_type[port_idx][copp_idx],
				   app_type);
			atomic_set(&this_adm.copp.acdb_id[port_idx][copp_idx],
				   acdb_id);
			if (path != ADM_PATH_COMPRESSED_RX)
				send_adm_custom_topology();
		}
	}

	if ((topology == ADM_CMD_COPP_OPEN_TOPOLOGY_ID_DTS_HPX_0 ||
	     topology == ADM_CMD_COPP_OPEN_TOPOLOGY_ID_DTS_HPX_1) &&
	    !perf_mode) {
		int res;
		atomic_set(&this_adm.mem_map_index, ADM_DTS_EAGLE);
		msm_dts_ion_memmap(&this_adm.outband_memmap);
		res = adm_memory_map_regions(&this_adm.outband_memmap.paddr, 0,
				(uint32_t *)&this_adm.outband_memmap.size, 1);
		if (res < 0)
			pr_err("%s: DTS_EAGLE mmap did not work!", __func__);
	}
	if (this_adm.copp.adm_delay[port_idx][copp_idx] &&
		perf_mode == LEGACY_PCM_MODE) {
		atomic_set(&this_adm.copp.adm_delay_stat[port_idx][copp_idx],
			   1);
		this_adm.copp.adm_delay[port_idx][copp_idx] = 0;
		wake_up(&this_adm.copp.adm_delay_wait[port_idx][copp_idx]);
	}

	/* Create a COPP if port id are not enabled */
	if ((perf_mode == LEGACY_PCM_MODE &&
		(atomic_read(&this_adm.copp_cnt[index]) == 0)) ||
		(perf_mode != LEGACY_PCM_MODE &&
		(atomic_read(&this_adm.copp_low_latency_cnt[index]) == 0))) {
		ad_logd("%s:opening ADM: perf_mode: %d\n", __func__,
			perf_mode);
		open.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						   APR_HDR_LEN(APR_HDR_SIZE),
						   APR_PKT_VER);
		open.hdr.pkt_size = sizeof(open);
		open.hdr.src_svc = APR_SVC_ADM;
		open.hdr.src_domain = APR_DOMAIN_APPS;
		open.hdr.src_port = tmp_port;
		open.hdr.dest_svc = APR_SVC_ADM;
		open.hdr.dest_domain = APR_DOMAIN_ADSP;
		open.hdr.dest_port = tmp_port;
		open.hdr.token = port_idx << 16 | copp_idx;
		open.hdr.opcode = ADM_CMD_DEVICE_OPEN_V5;
		open.flags = flags;
		open.mode_of_operation = path;
		open.endpoint_id_1 = tmp_port;

		if (this_adm.ec_ref_rx == -1) {
			open.endpoint_id_2 = 0xFFFF;
		} else if (this_adm.ec_ref_rx && (path != 1)) {
			open.endpoint_id_2 = this_adm.ec_ref_rx;
			this_adm.ec_ref_rx = -1;
		}

		open.topology_id = topology;

		open.dev_num_channel = channel_mode & 0x00FF;
		open.bit_width = bit_width;
		WARN_ON((perf_mode == ULTRA_LOW_LATENCY_PCM_MODE) &&
			(rate != ULL_SUPPORTED_SAMPLE_RATE));
		open.sample_rate  = rate;
		memset(open.dev_channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

		if (channel_mode == 1)	{
			open.dev_channel_mapping[0] = PCM_CHANNEL_FC;
		} else if (channel_mode == 2) {
			open.dev_channel_mapping[0] = PCM_CHANNEL_FL;
			open.dev_channel_mapping[1] = PCM_CHANNEL_FR;
		} else if (channel_mode == 3)	{
			open.dev_channel_mapping[0] = PCM_CHANNEL_FL;
			open.dev_channel_mapping[1] = PCM_CHANNEL_FR;
			open.dev_channel_mapping[2] = PCM_CHANNEL_FC;
		} else if (channel_mode == 4) {
			open.dev_channel_mapping[0] = PCM_CHANNEL_FL;
			open.dev_channel_mapping[1] = PCM_CHANNEL_FR;
			open.dev_channel_mapping[2] = PCM_CHANNEL_RB;
			open.dev_channel_mapping[3] = PCM_CHANNEL_LB;
		} else if (channel_mode == 5) {
			open.dev_channel_mapping[0] = PCM_CHANNEL_FL;
			open.dev_channel_mapping[1] = PCM_CHANNEL_FR;
			open.dev_channel_mapping[2] = PCM_CHANNEL_FC;
			open.dev_channel_mapping[3] = PCM_CHANNEL_LB;
			open.dev_channel_mapping[4] = PCM_CHANNEL_RB;
		} else if (channel_mode == 6) {
			open.dev_channel_mapping[0] = PCM_CHANNEL_FL;
			open.dev_channel_mapping[1] = PCM_CHANNEL_FR;
			open.dev_channel_mapping[2] = PCM_CHANNEL_LFE;
			open.dev_channel_mapping[3] = PCM_CHANNEL_FC;
			open.dev_channel_mapping[4] = PCM_CHANNEL_LS;
			open.dev_channel_mapping[5] = PCM_CHANNEL_RS;
		} else if (channel_mode == 8) {
			open.dev_channel_mapping[0] = PCM_CHANNEL_FL;
			open.dev_channel_mapping[1] = PCM_CHANNEL_FR;
			open.dev_channel_mapping[2] = PCM_CHANNEL_LFE;
			open.dev_channel_mapping[3] = PCM_CHANNEL_FC;
			open.dev_channel_mapping[4] = PCM_CHANNEL_LB;
			open.dev_channel_mapping[5] = PCM_CHANNEL_RB;
			open.dev_channel_mapping[6] = PCM_CHANNEL_FLC;
			open.dev_channel_mapping[7] = PCM_CHANNEL_FRC;
		} else {
			ad_loge("%s invalid num_chan %d\n", __func__,
					channel_mode);
			return -EINVAL;
		}
		if ((open.dev_num_channel > 2) && multi_ch_map.set_channel_map)
			memcpy(open.dev_channel_mapping,
			       multi_ch_map.channel_mapping,
			       PCM_FORMAT_MAX_NUM_CHANNEL);

		ad_logd("%s: port_id=%#x rate=%d topology_id=0x%X\n",
			__func__, open.endpoint_id_1, open.sample_rate,
			open.topology_id);

		atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 0);

		ret = apr_send_pkt(this_adm.apr, (uint32_t *)&open);
		if (ret < 0) {
			ad_loge("%s:ADM enable for port %#x for[%d] failed\n",
						__func__, tmp_port, port_id);
			ret = -EINVAL;
			goto fail_cmd;
		}
		/* Wait for the callback with copp id */
		ret = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
			atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
			msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			ad_loge("%s ADM open failed for port %#x for [%d]\n",
						__func__, tmp_port, port_id);
			return -EINVAL;
		}
	}
	if (perf_mode == ULTRA_LOW_LATENCY_PCM_MODE ||
			perf_mode == LOW_LATENCY_PCM_MODE) {
		atomic_inc(&this_adm.copp_low_latency_cnt[index]);
		ad_logd("%s: index: %d coppid: %d", __func__, index,
			atomic_read(&this_adm.copp_low_latency_id[index]));
	} else {
		atomic_inc(&this_adm.copp_cnt[index]);
		ad_logd("%s: index: %d coppid: %d", __func__, index,
			atomic_read(&this_adm.copp_id[index]));
	}
	return 0;

fail_cmd:

	return ret;
}

int adm_matrix_map(int path, struct route_payload payload_map, int perf_mode)
{
	struct adm_cmd_matrix_map_routings_v5	*route;
	struct adm_session_map_node_v5 *node;
	uint16_t *copps_list;
	int cmd_size = 0;
	int ret = 0, i = 0;
	void *payload = NULL;
	void *matrix_map = NULL;
	int port_idx, copp_idx;

	/* Assumes port_ids have already been validated during adm_open */
	int index = q6audio_get_port_index(copp_id);
	if (index < 0 || index >= AFE_MAX_PORTS) {
		ad_loge("%s: invalid port idx %d token %d\n",
					__func__, index, copp_id);
		return 0;
	}
	cmd_size = (sizeof(struct adm_cmd_matrix_map_routings_v5) +
			sizeof(struct adm_session_map_node_v5) +
			(sizeof(uint32_t) * payload_map.num_copps));
	matrix_map = kzalloc(cmd_size, GFP_KERNEL);
	if (matrix_map == NULL) {
		ad_loge("%s: Mem alloc failed\n", __func__);
		ret = -EINVAL;
		return ret;
	}
	route = (struct adm_cmd_matrix_map_routings_v5 *)matrix_map;

	ad_logd("%s: session 0x%x path:%d num_copps:%d port_id[0]:%#x coppid[%d]\n",
		 __func__, session_id, path, num_copps, port_id[0], copp_id);

	route->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	route->hdr.pkt_size = cmd_size;
	route->hdr.src_svc = 0;
	route->hdr.src_domain = APR_DOMAIN_APPS;
	route->hdr.src_port = 0; /* Ignored */;
	route->hdr.dest_svc = APR_SVC_ADM;
	route->hdr.dest_domain = APR_DOMAIN_ADSP;
	route->hdr.dest_port = 0; /* Ignored */;
	route->hdr.token = 0;
	if (path == ADM_PATH_COMPRESSED_RX) {
		pr_debug("%s: ADM_CMD_STREAM_DEVICE_MAP_ROUTINGS_V5 0x%x\n",
			 __func__, ADM_CMD_STREAM_DEVICE_MAP_ROUTINGS_V5);
		route->hdr.opcode = ADM_CMD_STREAM_DEVICE_MAP_ROUTINGS_V5;
	} else {
		pr_debug("%s: DM_CMD_MATRIX_MAP_ROUTINGS_V5 0x%x\n",
			 __func__, ADM_CMD_MATRIX_MAP_ROUTINGS_V5);
		route->hdr.opcode = ADM_CMD_MATRIX_MAP_ROUTINGS_V5;
	}
	route->num_sessions = 1;

	switch (path) {
	case ADM_PATH_PLAYBACK:
		route->matrix_id = ADM_MATRIX_ID_AUDIO_RX;
		break;
	case ADM_PATH_LIVE_REC:
	case ADM_PATH_NONLIVE_REC:
		route->matrix_id = ADM_MATRIX_ID_AUDIO_TX;
		break;
	case ADM_PATH_COMPRESSED_RX:
		route->matrix_id = ADM_MATRIX_ID_COMPRESSED_AUDIO_RX;
		break;
	default:
		ad_loge("%s: Wrong path set[%d]\n", __func__, path);
		break;
	}
	payload = ((u8 *)matrix_map +
			sizeof(struct adm_cmd_matrix_map_routings_v5));
	node = (struct adm_session_map_node_v5 *)payload;

	node->session_id = payload_map.session_id;
	node->num_copps = payload_map.num_copps;
	payload = (u8 *)node + sizeof(struct adm_session_map_node_v5);
	copps_list = (uint16_t *)payload;
	for (i = 0; i < payload_map.num_copps; i++) {
		port_idx =
		adm_validate_and_get_port_index(payload_map.port_id[i]);
		if (port_idx < 0) {
			pr_err("%s: Invalid port_id 0x%x\n", __func__,
				payload_map.port_id[i]);
			return -EINVAL;
		}
		else
			continue;
		ad_logd("%s: port_id[%#x]: %d, index: %d act coppid[0x%x]\n",
			__func__, i, port_id[i], tmp, copps_list[i]);
	}
	atomic_set(&this_adm.matrix_map_stat, 0);

	ret = apr_send_pkt(this_adm.apr, (uint32_t *)matrix_map);
	if (ret < 0) {
		ad_loge("%s: ADM routing for port %#x failed\n",
					__func__, port_id[0]);
		ret = -EINVAL;
		goto fail_cmd;
	}
	ret = wait_event_timeout(this_adm.matrix_map_wait,
				atomic_read(&this_adm.matrix_map_stat),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		ad_loge("%s: ADM cmd Route failed for port %#x\n",
					__func__, port_id[0]);
		ret = -EINVAL;
		goto fail_cmd;
	}

	if (perf_mode != ULTRA_LOW_LATENCY_PCM_MODE) {
		for (i = 0; i < num_copps; i++)
			send_adm_cal(port_id[i], path, perf_mode);

		for (i = 0; i < num_copps; i++) {
			int tmp, copp_id;
			tmp = afe_get_port_index(port_id[i]);
			if (tmp >= 0 && tmp < AFE_MAX_PORTS) {
				if (perf_mode == LEGACY_PCM_MODE)
					copp_id = atomic_read(
					&this_adm.copp_id[tmp]);
				else
					copp_id = atomic_read(
					&this_adm.copp_low_latency_id[tmp]);
				rtac_add_adm_device(port_id[i],
						copp_id, path, session_id);
				ad_logd("%s, copp_id: %d\n",
							__func__, copp_id);
			} else
				ad_logd("%s: Invalid port index %d",
							__func__, tmp);
		}
	}

fail_cmd:
	kfree(matrix_map);
	return ret;
}

void adm_ec_ref_rx_id(int port_id)
{
	this_adm.ec_ref_rx = port_id;
	pr_debug("%s: ec_ref_rx:%d", __func__, this_adm.ec_ref_rx);
}

	ad_logd("%s\n", __func__);
	if (this_adm.apr == NULL) {
		this_adm.apr = apr_register("ADSP", "ADM", adm_callback,
						0xFFFFFFFF, &this_adm);
		if (this_adm.apr == NULL) {
			ad_loge("%s: Unable to register ADM\n", __func__);
			ret = -ENODEV;
			return ret;
		}
		rtac_set_adm_handle(this_adm.apr);
	}

	port_id = q6audio_convert_virtual_to_portid(port_id);
	port_idx = adm_validate_and_get_port_index(port_id);
	if (port_idx < 0) {
		pr_err("%s: Invalid port_id 0x%x\n",
			__func__, port_id);
		return -EINVAL;
	}

	if (q6audio_validate_port(port_id) < 0) {
		ad_loge("%s port id[%#x] is invalid\n", __func__, port_id);
		return -ENODEV;
	}

	if (this_adm.copp.adm_delay[port_idx][copp_idx] && perf_mode
		== LEGACY_PCM_MODE) {
		atomic_set(&this_adm.copp.adm_delay_stat[port_idx][copp_idx],
			   1);
		this_adm.copp.adm_delay[port_idx][copp_idx] = 0;
		wake_up(&this_adm.copp.adm_delay_wait[port_idx][copp_idx]);
	}

	atomic_dec(&this_adm.copp.cnt[port_idx][copp_idx]);
	if (!(atomic_read(&this_adm.copp.cnt[port_idx][copp_idx]))) {
		copp_id = adm_get_copp_id(port_idx, copp_idx);
		pr_debug("%s: Closing ADM port_idx:%d copp_idx:%d copp_id:0x%x\n",
			 __func__, port_idx, copp_idx, copp_id);
		if ((!perf_mode) && (this_adm.outband_memmap.paddr != 0) &&
			(atomic_read(&this_adm.copp.topology[port_idx][copp_idx]) ==
			SRS_TRUMEDIA_TOPOLOGY_ID)) {
			atomic_set(&this_adm.mem_map_index,
				ADM_SRS_TRUMEDIA);
			ret = adm_memory_unmap_regions();
			if (ret < 0) {
				pr_err("%s: adm mem unmmap err %d",
					__func__, ret);
			} else {
				atomic_set(&this_adm.mem_map_handles
					   [ADM_SRS_TRUMEDIA], 0);
			}
		}

		if ((!perf_mode) && (this_adm.outband_memmap.paddr != 0)) {
			atomic_set(&this_adm.mem_map_index, ADM_DTS_EAGLE);
			ret = adm_memory_unmap_regions();
			if (ret < 0) {
				pr_err("%s: adm mem unmmap err %d",
					__func__, ret);
			} else {
				atomic_set(&this_adm.mem_map_handles
					   [ADM_DTS_EAGLE], 0);
			}
		}

	mmap_region_cmd = kzalloc(cmd_size, GFP_KERNEL);
	if (!mmap_region_cmd) {
		ad_loge("%s: allocate mmap_region_cmd failed\n", __func__);
		return -ENOMEM;
	}
	mmap_regions = (struct avs_cmd_shared_mem_map_regions *)mmap_region_cmd;
	mmap_regions->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
		close.pkt_size = sizeof(close);
		close.src_svc = APR_SVC_ADM;
		close.src_domain = APR_DOMAIN_APPS;
		close.src_port = port_id;
		close.dest_svc = APR_SVC_ADM;
		close.dest_domain = APR_DOMAIN_ADSP;
		close.dest_port = copp_id;
		close.token = port_idx << 16 | copp_idx;
		close.opcode = ADM_CMD_DEVICE_CLOSE_V5;

	ad_logd("%s: map_regions->num_regions = %d\n", __func__,
				mmap_regions->num_regions);
	payload = ((u8 *) mmap_region_cmd +
				sizeof(struct avs_cmd_shared_mem_map_regions));
	mregions = (struct avs_shared_map_region_payload *)payload;

		ret = apr_send_pkt(this_adm.apr, (uint32_t *)&close);
		if (ret < 0) {
			pr_err("%s: ADM close failed %d\n", __func__, ret);
			return -EINVAL;
		}

	atomic_set(&this_adm.copp_stat[index], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *) mmap_region_cmd);
	if (ret < 0) {
		ad_loge("%s: mmap_regions op[0x%x]rc[%d]\n", __func__,
					mmap_regions->hdr.opcode, ret);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_adm.wait[index],
			atomic_read(&this_adm.copp_stat[index]), 5 * HZ);
	if (!ret) {
		ad_loge("%s: timeout. waited for memory_map\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}
	return 0;
}

int adm_map_rtac_block(struct rtac_cal_block_data *cal_block)
{
	struct  avs_cmd_shared_mem_unmap_regions unmap_regions;
	int     ret = 0;
	int     index = 0;

	ad_logd("%s\n", __func__);

	if (this_adm.apr == NULL) {
		ad_loge("%s APR handle NULL\n", __func__);
		return -EINVAL;
	}

	if (q6audio_validate_port(port_id) < 0) {
		ad_loge("%s port idi[%d] is invalid\n", __func__, port_id);
		return -ENODEV;
	}

	index = q6audio_get_port_index(port_id);

	unmap_regions.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
							APR_PKT_VER);
	unmap_regions.hdr.pkt_size = sizeof(unmap_regions);
	unmap_regions.hdr.src_port = 0;
	unmap_regions.hdr.dest_port = atomic_read(&this_adm.copp_id[index]);
	unmap_regions.hdr.token = port_id;
	unmap_regions.hdr.opcode = ADM_CMD_SHARED_MEM_UNMAP_REGIONS;
	unmap_regions.mem_map_handle = atomic_read(&this_adm.
		mem_map_cal_handles[atomic_read(&this_adm.mem_map_cal_index)]);
	atomic_set(&this_adm.copp_stat[index], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *) &unmap_regions);
	if (ret < 0) {
		ad_loge("%s: mmap_regions op[0x%x]rc[%d]\n", __func__,
				unmap_regions.hdr.opcode, ret);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(this_adm.wait[index],
				 atomic_read(&this_adm.copp_stat[index]),
				 5 * HZ);
	if (!ret) {
		ad_loge("%s: timeout. waited for memory_unmap index %d\n",
		       __func__, index);
		ret = -EINVAL;
		goto fail_cmd;
	} else {
		ad_logd("%s: Unmap handle 0x%x succeeded\n", __func__,
			 unmap_regions.mem_map_handle);
	}
	/* Wait for the callback */
	rc = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
		atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: Set params timed out port = %#x\n",
			 __func__, port_id);
		rc = -EINVAL;
		goto end;
	}
	rc = 0;

end:
	kfree(adm_params);
	return rc;
}

/*
 * adm_update_wait_parameters must be called with routing driver locks.
 * adm_reset_wait_parameters must be called with routing driver locks.
 * set and reset parmeters are seperated to make sure it is always called
 * under routing driver lock.
 * adm_wait_timeout is to block until timeout or interrupted. Timeout is
 * not a an error.
 */
int adm_set_wait_parameters(int port_id, int copp_idx)
{

	int ret = 0, port_idx;
	pr_debug("%s: port_id 0x%x, copp_idx %d\n", __func__, port_id,
		 copp_idx);
	port_id = afe_convert_virtual_to_portid(port_id);
	port_idx = adm_validate_and_get_port_index(port_id);
	if (port_idx < 0) {
		pr_err("%s: Invalid port_id %#x\n", __func__, port_id);
		ret = -EINVAL;
		goto end;
	}

	if (copp_idx < 0 || copp_idx >= MAX_COPPS_PER_PORT) {
		pr_err("%s: Invalid copp_num: %d\n", __func__, copp_idx);
		return -EINVAL;
	}

	this_adm.copp.adm_delay[port_idx][copp_idx] = 1;
	atomic_set(&this_adm.copp.adm_delay_stat[port_idx][copp_idx], 0);

end:
	return ret;

}

int adm_reset_wait_parameters(int port_id, int copp_idx)
{
	int copp_id;
	ad_logd("%s\n", __func__);

	if (port_index < 0) {
		ad_loge("%s: invalid port_id = %d\n", __func__, port_index);
		return -EINVAL;
	}

	atomic_set(&this_adm.copp.adm_delay_stat[port_idx][copp_idx], 1);
	this_adm.copp.adm_delay[port_idx][copp_idx] = 0;

end:
	return ret;
}

int adm_wait_timeout(int port_id, int copp_idx, int wait_time)
{
	ad_logd("%s\n", __func__);

	if (port_index < 0) {
		ad_loge("%s: invalid port_id = %d\n", __func__, port_index);
		return -EINVAL;
	}

	ret = wait_event_timeout(
		this_adm.copp.adm_delay_wait[port_idx][copp_idx],
		atomic_read(&this_adm.copp.adm_delay_stat[port_idx][copp_idx]),
		msecs_to_jiffies(wait_time));
	pr_debug("%s: return %d\n", __func__, ret);
	if (ret != 0)
		ret = -EINTR;
end:
	pr_debug("%s: return %d--\n", __func__, ret);
	return ret;
}

int adm_store_cal_data(int port_id, int copp_idx, int path, int perf_mode,
		       int cal_index, char *params, int *size)
{
	int rc = 0;
	struct cal_block_data		*cal_block = NULL;
	int app_type, acdb_id, port_idx, sample_rate;

	if (this_adm.cal_data[cal_index] == NULL) {
		pr_debug("%s: cal_index %d not allocated!\n",
			__func__, cal_index);
		goto end;
	}

	if (get_cal_path(path) != RX_DEVICE) {
		pr_debug("%s: Invalid path to store calibration %d\n",
			 __func__, path);
		rc = -EINVAL;
		goto end;
	}

	port_id = afe_convert_virtual_to_portid(port_id);
	port_idx = adm_validate_and_get_port_index(port_id);
	if (port_idx < 0) {
		pr_err("%s: Invalid port_id 0x%x\n", __func__, port_id);
		rc = -EINVAL;
		goto end;
	}

	if (copp_idx < 0 || copp_idx >= MAX_COPPS_PER_PORT) {
		pr_err("%s: Invalid copp_num: %d\n", __func__, copp_idx);
		return -EINVAL;
	}

	acdb_id = atomic_read(&this_adm.copp.acdb_id[port_idx][copp_idx]);
	app_type = atomic_read(&this_adm.copp.app_type[port_idx][copp_idx]);
	sample_rate = atomic_read(&this_adm.copp.rate[port_idx][copp_idx]);

	mutex_lock(&this_adm.cal_data[cal_index]->lock);
	cal_block = adm_find_cal(cal_index, path, app_type,
				acdb_id, sample_rate);
	if (cal_block == NULL)
		goto unlock;

	if (cal_block->cal_data.size <= 0) {
		pr_debug("%s: No ADM cal send for port_id = 0x%x!\n",
			__func__, port_id);
		rc = -EINVAL;
		goto unlock;
	}

	pr_debug("%s:port_id %d, copp_idx %d, path %d",
		 __func__, port_id, copp_idx, path);
	pr_debug("perf_mode %d, cal_type %d, size %d\n",
		 perf_mode, cal_index, *size);

	if (cal_index == ADM_AUDPROC_CAL) {
		if (cal_block->cal_data.size > AUD_PROC_BLOCK_SIZE) {
			pr_err("%s:audproc:invalid size exp/actual[%zd, %d]\n",
				__func__, cal_block->cal_data.size, *size);
			rc = -ENOMEM;
			goto unlock;
		}
	} else if (cal_index == ADM_AUDVOL_CAL) {
		if (cal_block->cal_data.size > AUD_VOL_BLOCK_SIZE) {
			pr_err("%s:aud_vol:invalid size exp/actual[%zd, %d]\n",
				__func__, cal_block->cal_data.size, *size);
			rc = -ENOMEM;
			goto unlock;
		}
	} else {
		pr_debug("%s: Not valid calibration for dolby topolgy\n",
			 __func__);
		rc = -EINVAL;
		goto unlock;
	}
	memcpy(params, cal_block->cal_data.kvaddr, cal_block->cal_data.size);
	*size = cal_block->cal_data.size;

unlock:
	mutex_unlock(&this_adm.cal_data[cal_index]->lock);
end:
	return rc;
}

int adm_send_compressed_device_mute(int port_id, int copp_idx, bool mute_on)
{
	struct adm_set_compressed_device_mute mute_params;
	int ret = 0;
	int port_idx;

	pr_debug("%s port_id: 0x%x, copp_idx %d, mute_on: %d\n",
		 __func__, port_id, copp_idx, mute_on);
	port_id = afe_convert_virtual_to_portid(port_id);
	port_idx = adm_validate_and_get_port_index(port_id);
	if (port_idx < 0 || port_idx >= AFE_MAX_PORTS) {
		pr_err("%s: Invalid port_id %#x copp_idx %d\n",
			__func__, port_id, copp_idx);
		ret = -EINVAL;
		goto end;
	}

	mute_params.command.hdr.hdr_field =
			APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mute_params.command.hdr.pkt_size =
			sizeof(struct adm_set_compressed_device_mute);
	mute_params.command.hdr.src_svc = APR_SVC_ADM;
	mute_params.command.hdr.src_domain = APR_DOMAIN_APPS;
	mute_params.command.hdr.src_port = port_id;
	mute_params.command.hdr.dest_svc = APR_SVC_ADM;
	mute_params.command.hdr.dest_domain = APR_DOMAIN_ADSP;
	mute_params.command.hdr.dest_port =
			atomic_read(&this_adm.copp.id[port_idx][copp_idx]);
	mute_params.command.hdr.token = port_idx << 16 | copp_idx;
	mute_params.command.hdr.opcode = ADM_CMD_SET_PP_PARAMS_V5;
	mute_params.command.payload_addr_lsw = 0;
	mute_params.command.payload_addr_msw = 0;
	mute_params.command.mem_map_handle = 0;
	mute_params.command.payload_size = sizeof(mute_params) -
						sizeof(mute_params.command);
	mute_params.params.module_id = AUDPROC_MODULE_ID_COMPRESSED_MUTE;
	mute_params.params.param_id = AUDPROC_PARAM_ID_COMPRESSED_MUTE;
	mute_params.params.param_size = mute_params.command.payload_size -
					sizeof(mute_params.params);
	mute_params.params.reserved = 0;
	mute_params.mute_on = mute_on;

	atomic_set(&this_adm.copp.stat[port_idx][copp_idx], 0);
	ret = apr_send_pkt(this_adm.apr, (uint32_t *)&mute_params);
	if (ret < 0) {
		pr_err("%s: device mute for port %d copp %d failed, ret %d\n",
			__func__, port_id, copp_idx, ret);
		ret = -EINVAL;
		goto end;
	}

	/* Wait for the callback */
	ret = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
		atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: send device mute for port %d copp %d failed\n",
			__func__, port_id, copp_idx);
		ret = -EINVAL;
		goto end;
	}
	ret = 0;
end:
	return ret;
}

int adm_send_compressed_device_latency(int port_id, int copp_idx, int latency)
{
	this_adm.ec_ref_rx = port_id;
	ad_logd("%s ec_ref_rx:%d", __func__, this_adm.ec_ref_rx);
}

int adm_set_sound_focus(int port_id, int copp_idx,
			struct sound_focus_param soundFocusData)
{
	struct adm_set_fluence_soundfocus_param soundfocus_params;
	int sz = 0;
	int ret  = 0;
	int port_idx;
	int i;

	pr_debug("%s: Enter, port_id %d, copp_idx %d\n",
		  __func__, port_id, copp_idx);

	port_id = afe_convert_virtual_to_portid(port_id);
	port_idx = adm_validate_and_get_port_index(port_id);
	if (port_idx < 0) {
		pr_err("%s: Invalid port_id %#x\n", __func__, port_id);

		ret = -EINVAL;
		goto done;
	}

	ad_logd("%s port_id=%#x index %d perf_mode: %d\n", __func__, port_id,
		index, perf_mode);

	if (perf_mode == ULTRA_LOW_LATENCY_PCM_MODE ||
				perf_mode == LOW_LATENCY_PCM_MODE) {
		if (!(atomic_read(&this_adm.copp_low_latency_cnt[index]))) {
			ad_loge("%s: copp count for port[%#x]is 0\n", __func__,
				port_id);
			goto fail_cmd;
		}
		atomic_dec(&this_adm.copp_low_latency_cnt[index]);
	} else {
		if (!(atomic_read(&this_adm.copp_cnt[index]))) {
			ad_loge("%s: copp count for port[%#x]is 0\n", __func__,
				port_id);
			goto fail_cmd;
		}
		atomic_dec(&this_adm.copp_cnt[index]);
	}

		ad_logd("%s:Closing ADM: perf_mode: %d\n", __func__,
				perf_mode);
		close.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		close.pkt_size = sizeof(close);
		close.src_svc = APR_SVC_ADM;
		close.src_domain = APR_DOMAIN_APPS;
		close.src_port = port_id;
		close.dest_svc = APR_SVC_ADM;
		close.dest_domain = APR_DOMAIN_ADSP;
		if (perf_mode == ULTRA_LOW_LATENCY_PCM_MODE ||
				perf_mode == LOW_LATENCY_PCM_MODE)
			close.dest_port =
			     atomic_read(&this_adm.copp_low_latency_id[index]);
		else
			close.dest_port = atomic_read(&this_adm.copp_id[index]);
		close.token = port_id;
		close.opcode = ADM_CMD_DEVICE_CLOSE_V5;

		ret = -EINVAL;
		goto done;
	}
	/* Wait for the callback */
	ret = wait_event_timeout(this_adm.copp.wait[port_idx][copp_idx],
		atomic_read(&this_adm.copp.stat[port_idx][copp_idx]),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Set params timed out\n", __func__);

		if (perf_mode == ULTRA_LOW_LATENCY_PCM_MODE ||
				perf_mode == LOW_LATENCY_PCM_MODE) {
			copp_id = atomic_read(
				&this_adm.copp_low_latency_id[index]);
			ad_logd("%s:coppid %d portid=%#x index=%d coppcnt=%d\n",
				__func__,
				copp_id,
				port_id, index,
				atomic_read(
					&this_adm.copp_low_latency_cnt[index]));
			atomic_set(&this_adm.copp_low_latency_id[index],
				RESET_COPP_ID);
		} else {
			copp_id = atomic_read(&this_adm.copp_id[index]);
			ad_logd("%s:coppid %d portid=%#x index=%d coppcnt=%d\n",
				__func__,
				copp_id,
				port_id, index,
				atomic_read(&this_adm.copp_cnt[index]));
			atomic_set(&this_adm.copp_id[index],
				RESET_COPP_ID);
		}

		ret = apr_send_pkt(this_adm.apr, (uint32_t *)&close);
		if (ret < 0) {
			ad_loge("%s ADM close failed\n", __func__);
			ret = -EINVAL;
			goto fail_cmd;
		}

		ret = wait_event_timeout(this_adm.wait[index],
				atomic_read(&this_adm.copp_stat[index]),
				msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			ad_loge("%s: ADM cmd Route failed for port %#x\n",
						__func__, port_id);
			ret = -EINVAL;
			goto done;
		}
	}

	if (perf_mode != ULTRA_LOW_LATENCY_PCM_MODE) {
		ad_logd("%s: remove adm device from rtac\n", __func__);
		rtac_remove_adm_device(port_id, copp_id);
	}

	admp.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	admp.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE, sizeof(admp));
	admp.hdr.src_svc = APR_SVC_ADM;
	admp.hdr.src_domain = APR_DOMAIN_APPS;
	admp.hdr.src_port = port_id;
	admp.hdr.dest_svc = APR_SVC_ADM;
	admp.hdr.dest_domain = APR_DOMAIN_ADSP;
	admp.hdr.dest_port = atomic_read(&this_adm.copp.id[p_idx][copp_idx]);
	admp.hdr.token = p_idx << 16 | ADM_CLIENT_ID_SOURCE_TRACKING << 8 |
			 copp_idx;
	admp.hdr.opcode = ADM_CMD_GET_PP_PARAMS_V5;
	admp.data_payload_addr_lsw =
		lower_32_bits(this_adm.sourceTrackingData.memmap.paddr);
	admp.data_payload_addr_msw =
		upper_32_bits(this_adm.sourceTrackingData.memmap.paddr);
	admp.mem_map_handle = atomic_read(&this_adm.mem_map_handles[
					  ADM_MEM_MAP_INDEX_SOURCE_TRACKING]);
	admp.module_id = VOICEPROC_MODULE_ID_GENERIC_TX;
	admp.param_id = VOICEPROC_PARAM_ID_FLUENCE_SOURCETRACKING;
	admp.param_max_size = sizeof(struct adm_param_fluence_sourcetracking_t)
				+ sizeof(struct adm_param_data_v5);
	admp.reserved = 0;

	atomic_set(&this_adm.copp.stat[p_idx][copp_idx], 0);

	ret = apr_send_pkt(this_adm.apr, (uint32_t *)&admp);
	if (ret < 0) {
		pr_err("%s - failed to get Source Tracking Params\n",
			__func__);

		ret = -EINVAL;
		goto done;
	}
	ret = wait_event_timeout(this_adm.copp.wait[p_idx][copp_idx],
			atomic_read(&this_adm.copp.stat[p_idx][copp_idx]),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s - get params timed out\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (this_adm.sourceTrackingData.apr_cmd_status != 0) {
		pr_err("%s - get params returned error %d\n",
			__func__, this_adm.sourceTrackingData.apr_cmd_status);

		ret = -EINVAL;
		goto done;
	}

	source_tracking_params = (struct adm_param_fluence_sourcetracking_t *)
			(this_adm.sourceTrackingData.memmap.kvaddr +
			 sizeof(struct adm_param_data_v5));
	for (i = 0; i < MAX_SECTORS; i++) {
		sourceTrackingData->vad[i] = source_tracking_params->vad[i];
		pr_debug("%s: vad[%d] = %d\n",
			  __func__, i, sourceTrackingData->vad[i]);
	}
	sourceTrackingData->doa_speech = source_tracking_params->doa_speech;
	pr_debug("%s: doa_speech = %d\n",
		  __func__, sourceTrackingData->doa_speech);

	for (i = 0; i < MAX_NOISE_SOURCE_INDICATORS; i++) {
		sourceTrackingData->doa_noise[i] =
					source_tracking_params->doa_noise[i];
		pr_debug("%s: doa_noise[%d] = %d\n",
			  __func__, i, sourceTrackingData->doa_noise[i]);
	}
	for (i = 0; i < MAX_POLAR_ACTIVITY_INDICATORS; i++) {
		sourceTrackingData->polar_activity[i] =
				source_tracking_params->polar_activity[i];
		pr_debug("%s: polar_activity[%d] = %d\n",
			  __func__, i, sourceTrackingData->polar_activity[i]);
	}

	ret = 0;

done:
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}

static int __init adm_init(void)
{
	int i = 0, j;
	this_adm.apr = NULL;
	this_adm.ec_ref_rx = -1;
	atomic_set(&this_adm.matrix_map_stat, 0);
	init_waitqueue_head(&this_adm.matrix_map_wait);
	atomic_set(&this_adm.adm_stat, 0);
	init_waitqueue_head(&this_adm.adm_wait);

	for (i = 0; i < AFE_MAX_PORTS; i++) {
		for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
			atomic_set(&this_adm.copp.id[i][j], RESET_COPP_ID);
			atomic_set(&this_adm.copp.cnt[i][j], 0);
			atomic_set(&this_adm.copp.topology[i][j], 0);
			atomic_set(&this_adm.copp.mode[i][j], 0);
			atomic_set(&this_adm.copp.stat[i][j], 0);
			atomic_set(&this_adm.copp.rate[i][j], 0);
			atomic_set(&this_adm.copp.bit_width[i][j], 0);
			atomic_set(&this_adm.copp.app_type[i][j], 0);
			atomic_set(&this_adm.copp.acdb_id[i][j], 0);
			init_waitqueue_head(&this_adm.copp.wait[i][j]);
			atomic_set(&this_adm.copp.adm_delay_stat[i][j], 0);
			init_waitqueue_head(
				&this_adm.copp.adm_delay_wait[i][j]);
			atomic_set(&this_adm.copp.topology[i][j], 0);
			this_adm.copp.adm_delay[i][j] = 0;
		}
	}

	if (adm_init_cal_data())
		pr_err("%s: could not init cal data!\n", __func__);

	this_adm.sourceTrackingData.ion_client = NULL;
	this_adm.sourceTrackingData.ion_handle = NULL;
	this_adm.sourceTrackingData.memmap.size = 0;
	this_adm.sourceTrackingData.memmap.kvaddr = NULL;
	this_adm.sourceTrackingData.memmap.paddr = 0;
	this_adm.sourceTrackingData.apr_cmd_status = -1;
	atomic_set(&this_adm.mem_map_handles[ADM_MEM_MAP_INDEX_SOURCE_TRACKING],
		   0);

	return 0;
}

static void __exit adm_exit(void)
{
	adm_delete_cal_data();
}

device_initcall(adm_init);

