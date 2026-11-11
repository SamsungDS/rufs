// SPDX-License-Identifier: GPL-2.0-or-later
/* Analysis-only direct blk-mq frontend. Included by ufshcd-standalone.c. */

#define UFS_SA_MAX_LUS UFS_UPIU_MAX_UNIT_NUM_ID
#define UFS_SA_INLINE_SG 4
#define UFS_SA_MAX_REQUEST_RETRIES 5

struct ufs_sa_lu {
	struct ufs_hba *hba;
	struct request_queue *queue;
	struct gendisk *disk;
	struct scsi_device sdev;
	u64 block_count;
	u32 block_size;
	u8 lun;
	bool published;
};

struct ufs_sa_host {
	struct ufs_hba *hba;
	struct ufs_sa_lu lus[UFS_SA_MAX_LUS];
	unsigned int nr_lus;
	bool runtime_pm_held;
	struct mutex tm_lock;
	struct mutex recovery_lock;
	struct work_struct recovery_work;
	enum {
		UFS_SA_HOST_ONLINE,
		UFS_SA_HOST_RECOVERING,
		UFS_SA_HOST_FAILED,
	} state;
};

static const struct block_device_operations ufs_sa_fops = {
	.owner = THIS_MODULE,
};

enum ufs_sa_request_state {
	UFS_SA_REQ_IDLE,
	UFS_SA_REQ_ACTIVE,
	UFS_SA_REQ_TIMED_OUT,
	UFS_SA_REQ_ABORT_COMMITTED,
	UFS_SA_REQ_LU_RESET_COMMITTED,
	UFS_SA_REQ_CONTROLLER_RESET_COMMITTED,
	UFS_SA_REQ_RETIRED,
};

enum ufs_sa_tm_op {
	UFS_SA_TM_QUERY_TASK,
	UFS_SA_TM_ABORT_TASK,
	UFS_SA_TM_LOGICAL_RESET,
};

enum ufs_sa_disposition {
	UFS_SA_DISPOSITION_REQUEUE,
	UFS_SA_DISPOSITION_END_TIMEOUT,
	UFS_SA_DISPOSITION_END_IOERR,
};

struct ufs_sa_tm_result {
	int error;
	u8 response;
};

static struct ufs_sa_tm_result ufs_sa_exec_tm(struct ufs_sa_host *host,
					       enum ufs_sa_tm_op op, u8 lun,
					       u8 task_tag)
{
	struct ufs_sa_tm_result result = { .response = 0xff };
	u8 function;

	switch (op) {
	case UFS_SA_TM_QUERY_TASK:
		function = UFS_QUERY_TASK;
		break;
	case UFS_SA_TM_ABORT_TASK:
		function = UFS_ABORT_TASK;
		break;
	case UFS_SA_TM_LOGICAL_RESET:
		function = UFS_LOGICAL_RESET;
		task_tag = 0;
		break;
	default:
		result.error = -EINVAL;
		return result;
	}

	mutex_lock(&host->tm_lock);
	result.error = ufshcd_issue_tm_cmd(host->hba, lun, task_tag, function,
					   &result.response);
	mutex_unlock(&host->tm_lock);
	return result;
}

struct ufs_sa_pdu {
	struct scsi_cmnd cmd;
	struct ufshcd_lrb lrb;
	struct scatterlist inline_sg[UFS_SA_INLINE_SG];
	u8 sense[SCSI_SENSE_BUFFERSIZE];
	unsigned int dma_nents;
	spinlock_t state_lock;
	enum ufs_sa_request_state state;
	bool hardware_released;
	bool timed_out_origin;
	u8 retries;
};

static struct ufs_sa_host *ufs_sa_host(struct ufs_hba *hba)
{
	return hba->standalone_host;
}

static struct ufs_sa_pdu *ufs_sa_pdu(struct request *rq)
{
	return blk_mq_rq_to_pdu(rq);
}

static void ufs_sa_unmap(struct ufs_hba *hba, struct ufs_sa_pdu *pdu)
{
	struct scsi_cmnd *cmd = &pdu->cmd;

	if (!cmd->sdb.table.nents)
		return;
	dma_unmap_sg(hba->dev, cmd->sdb.table.sgl, pdu->dma_nents,
		     cmd->sc_data_direction);
	sg_free_table_chained(&cmd->sdb.table, UFS_SA_INLINE_SG);
	cmd->sdb.table.nents = 0;
	pdu->dma_nents = 0;
}

static blk_status_t ufs_sa_map(struct ufs_hba *hba, struct request *rq,
			       struct scsi_cmnd *cmd)
{
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	struct scatterlist *last = NULL;
	unsigned int nr_segs = blk_rq_nr_phys_segments(rq);
	int mapped;

	if (!nr_segs)
		return BLK_STS_OK;
	cmd->sdb.table.sgl = pdu->inline_sg;
	if (sg_alloc_table_chained(&cmd->sdb.table, nr_segs, pdu->inline_sg,
				   UFS_SA_INLINE_SG))
		return BLK_STS_RESOURCE;
	cmd->sdb.table.nents = __blk_rq_map_sg(rq, cmd->sdb.table.sgl, &last);
	cmd->sdb.length = blk_rq_payload_bytes(rq);
	pdu->dma_nents = cmd->sdb.table.nents;
	mapped = dma_map_sg(hba->dev, cmd->sdb.table.sgl,
			    pdu->dma_nents, cmd->sc_data_direction);
	if (!mapped) {
		sg_free_table_chained(&cmd->sdb.table, UFS_SA_INLINE_SG);
		cmd->sdb.table.nents = 0;
		pdu->dma_nents = 0;
		return BLK_STS_RESOURCE;
	}
	cmd->sdb.table.nents = mapped;
	return BLK_STS_OK;
}

static blk_status_t ufs_sa_build_cmd(struct request *rq, struct scsi_cmnd *cmd,
				     const struct ufs_sa_lu *lu)
{
	u64 lba;
	u32 blocks;

	memset(cmd->cmnd, 0, sizeof(cmd->cmnd));
	cmd->sc_data_direction = DMA_NONE;
	if (req_op(rq) == REQ_OP_FLUSH) {
		cmd->cmd_len = 10;
		cmd->cmnd[0] = SYNCHRONIZE_CACHE;
		return BLK_STS_OK;
	}
	if (req_op(rq) != REQ_OP_READ && req_op(rq) != REQ_OP_WRITE)
		return BLK_STS_NOTSUPP;
	if (!blk_rq_bytes(rq) || blk_rq_bytes(rq) % lu->block_size)
		return BLK_STS_IOERR;
	lba = div_u64(blk_rq_pos(rq) << SECTOR_SHIFT, lu->block_size);
	blocks = blk_rq_bytes(rq) / lu->block_size;
	cmd->cmd_len = 16;
	cmd->cmnd[0] = req_op(rq) == REQ_OP_WRITE ? WRITE_16 : READ_16;
	cmd->cmnd[1] = rq->cmd_flags & REQ_FUA ? 0x8 : 0;
	put_unaligned_be64(lba, &cmd->cmnd[2]);
	put_unaligned_be32(blocks, &cmd->cmnd[10]);
	cmd->sc_data_direction = req_op(rq) == REQ_OP_WRITE ?
		DMA_TO_DEVICE : DMA_FROM_DEVICE;
	return BLK_STS_OK;
}

static blk_status_t ufs_sa_queue_rq(struct blk_mq_hw_ctx *hctx,
				    const struct blk_mq_queue_data *bd)
{
	struct request *rq = bd->rq;
	struct ufs_sa_lu *lu = rq->q->queuedata;
	struct ufs_hba *hba = lu->hba;
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	struct scsi_cmnd *cmd = &pdu->cmd;
	struct ufshcd_lrb *lrb = &pdu->lrb;
	struct ufs_hw_queue *hwq = NULL;
	blk_status_t status;

	if (READ_ONCE(ufs_sa_host(hba)->state) != UFS_SA_HOST_ONLINE ||
	    READ_ONCE(hba->ufshcd_state) != UFSHCD_STATE_OPERATIONAL)
		return BLK_STS_RESOURCE;
	memset(cmd, 0, sizeof(*cmd));
	memset(lrb, 0, sizeof(*lrb));
	cmd->device = &lu->sdev;
	cmd->sense_buffer = pdu->sense;
	cmd->submitter = SUBMITTED_BY_BLOCK_LAYER;
	status = ufs_sa_build_cmd(rq, cmd, lu);
	if (status != BLK_STS_OK)
		return status;
	status = ufs_sa_map(hba, rq, cmd);
	if (status != BLK_STS_OK)
		return status;

	ufshcd_hold(hba);
	lrb->standalone_req = true;
	ufshcd_setup_scsi_cmd(hba, cmd, lu->lun, rq->tag);
	ufshcd_sgl_to_prdt(hba, lrb, cmd->sdb.table.nents,
			    cmd->sdb.table.sgl);
	if (hba->mcq_enabled)
		hwq = ufshcd_mcq_req_to_hwq(hba, rq);
	spin_lock_irq(&pdu->state_lock);
	pdu->state = UFS_SA_REQ_ACTIVE;
	pdu->hardware_released = false;
	pdu->timed_out_origin = false;
	spin_unlock_irq(&pdu->state_lock);
	blk_mq_start_request(rq);
	ufshcd_send_command(hba, cmd, hwq);
	return BLK_STS_OK;
}

static bool ufs_sa_retryable_completion(struct scsi_cmnd *cmd)
{
	struct scsi_sense_hdr sense;
	u8 status = status_byte(cmd->result);

	if (host_byte(cmd->result) == DID_REQUEUE)
		return true;
	if (status == SAM_STAT_BUSY || status == SAM_STAT_TASK_SET_FULL)
		return true;
	return status == SAM_STAT_CHECK_CONDITION &&
		scsi_command_normalize_sense(cmd, &sense) &&
		sense.sense_key == UNIT_ATTENTION;
}

static bool ufs_sa_consume_retry(struct request *rq)
{
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	unsigned long flags;
	bool retry = false;

	if (rq->cmd_flags & REQ_FAILFAST_MASK)
		return false;
	spin_lock_irqsave(&pdu->state_lock, flags);
	if (pdu->retries < UFS_SA_MAX_REQUEST_RETRIES) {
		pdu->retries++;
		retry = true;
	}
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	return retry;
}

static void ufs_sa_reset_retries(struct ufs_sa_pdu *pdu)
{
	unsigned long flags;

	spin_lock_irqsave(&pdu->state_lock, flags);
	pdu->retries = 0;
	spin_unlock_irqrestore(&pdu->state_lock, flags);
}

static void ufs_sa_complete(struct request *rq)
{
	struct ufs_sa_lu *lu = rq->q->queuedata;
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	struct scsi_cmnd *cmd = &pdu->cmd;
	bool error = cmd->result || scsi_get_resid(cmd);
	bool retry = error && ufs_sa_retryable_completion(cmd) &&
		ufs_sa_consume_retry(rq);
	blk_status_t status = error ? BLK_STS_IOERR : BLK_STS_OK;
	unsigned long flags;

	ufs_sa_unmap(lu->hba, pdu);
	spin_lock_irqsave(&pdu->state_lock, flags);
	pdu->state = UFS_SA_REQ_IDLE;
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	if (retry) {
		blk_mq_requeue_request(rq, true);
		return;
	}
	ufs_sa_reset_retries(pdu);
	blk_mq_end_request(rq, status);
}

static enum blk_eh_timer_return ufs_sa_timeout(struct request *rq)
{
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	struct ufs_sa_lu *lu = rq->q->queuedata;
	struct ufs_sa_host *host = ufs_sa_host(lu->hba);
	unsigned long flags;
	bool schedule = false;

	spin_lock_irqsave(&pdu->state_lock, flags);
	if (pdu->state == UFS_SA_REQ_ACTIVE) {
		pdu->state = UFS_SA_REQ_TIMED_OUT;
		pdu->timed_out_origin = true;
		schedule = true;
	}
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	if (schedule)
		schedule_work(&host->recovery_work);
	return BLK_EH_RESET_TIMER;
}

static int ufs_sa_init_request(struct blk_mq_tag_set *set, struct request *rq,
			       unsigned int hctx_idx, int numa_node)
{
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);

	memset(pdu, 0, sizeof(*pdu));
	spin_lock_init(&pdu->state_lock);
	pdu->state = UFS_SA_REQ_IDLE;
	return 0;
}

static void ufs_sa_map_queues(struct blk_mq_tag_set *set)
{
	struct ufs_hba *hba = set->driver_data;
	unsigned int offset = 0;
	int type;

	for (type = 0; type < set->nr_maps; type++) {
		struct blk_mq_queue_map *map = &set->map[type];

		map->nr_queues = hba->nr_queues[type];
		if (!map->nr_queues)
			continue;
		map->queue_offset = offset;
		if (type == HCTX_TYPE_POLL && !hba->mcq_enabled)
			map->queue_offset = 0;
		blk_mq_map_queues(map);
		offset += map->nr_queues;
	}
}

static int ufs_sa_poll(struct blk_mq_hw_ctx *hctx, struct io_comp_batch *iob)
{
	struct ufs_sa_lu *lu = hctx->queue->queuedata;

	return ufshcd_poll(lu->hba->host, hctx->queue_num);
}

static const struct blk_mq_ops ufs_sa_mq_ops = {
	.queue_rq = ufs_sa_queue_rq,
	.complete = ufs_sa_complete,
	.timeout = ufs_sa_timeout,
	.init_request = ufs_sa_init_request,
	.map_queues = ufs_sa_map_queues,
	.poll = ufs_sa_poll,
};

static int ufs_sa_read_lu_geometry(struct ufs_hba *hba, u8 lun,
				   u64 *block_count, u32 *block_size)
{
	__be64 raw_block_count;
	u8 enabled, block_shift;
	int ret;

	ret = ufshcd_read_unit_desc_param(hba, lun, UNIT_DESC_PARAM_LU_ENABLE,
					  &enabled, sizeof(enabled));
	if (ret || !enabled)
		return ret ? ret : -ENODEV;
	ret = ufshcd_read_unit_desc_param(hba, lun,
					  UNIT_DESC_PARAM_LOGICAL_BLK_SIZE,
					  &block_shift, sizeof(block_shift));
	if (ret)
		return ret;
	ret = ufshcd_read_unit_desc_param(hba, lun,
					  UNIT_DESC_PARAM_LOGICAL_BLK_COUNT,
					  (u8 *)&raw_block_count,
					  sizeof(raw_block_count));
	if (ret)
		return ret;
	*block_size = 1U << block_shift;
	*block_count = be64_to_cpu(raw_block_count);
	return *block_size < SECTOR_SIZE ? -EINVAL : 0;
}

static int ufs_sa_publish_lu(struct ufs_sa_host *host, u8 lun)
{
	struct ufs_sa_lu *lu = &host->lus[lun];
	struct queue_limits lim = {
		.logical_block_size = SECTOR_SIZE,
		.max_hw_sectors = SZ_1M / SECTOR_SIZE,
		.max_segments = SG_ALL,
		.max_segment_size = PRDT_DATA_BYTE_COUNT_MAX,
	};
	int ret;

	lu->hba = host->hba;
	lu->lun = lun;
	ret = ufs_sa_read_lu_geometry(host->hba, lun, &lu->block_count,
				      &lu->block_size);
	if (ret)
		return ret;
	lu->sdev.host = host->hba->host;
	lu->sdev.lun = lun;
	lu->sdev.sector_size = lu->block_size;
	lim.logical_block_size = lu->block_size;
	lu->disk = blk_mq_alloc_disk(&host->hba->standalone_tag_set, &lim, lu);
	if (IS_ERR(lu->disk))
		return PTR_ERR(lu->disk);
	lu->queue = lu->disk->queue;
	lu->queue->queuedata = lu;
	blk_queue_rq_timeout(lu->queue, 30 * HZ);
	lu->disk->fops = &ufs_sa_fops;
	lu->disk->private_data = lu;
	snprintf(lu->disk->disk_name, DISK_NAME_LEN, "ufs%dlun%d",
		 host->hba->host->host_no, lun);
	set_capacity(lu->disk, lu->block_count *
		     (lu->block_size >> SECTOR_SHIFT));
	ret = add_disk(lu->disk);
	if (ret) {
		put_disk(lu->disk);
		lu->disk = NULL;
		return ret;
	}
	lu->published = true;
	host->nr_lus++;
	return 0;
}

static int ufs_sa_register_lus(struct ufs_hba *hba)
{
	struct ufs_sa_host *host = ufs_sa_host(hba);
	u8 nr_lus;
	unsigned int lun;
	int ret;

	ret = ufshcd_read_desc_param(hba, QUERY_DESC_IDN_DEVICE, 0,
				     DEVICE_DESC_PARAM_NUM_LU, &nr_lus,
				     sizeof(nr_lus));
	if (ret)
		return ret;

	for (lun = 0; lun < min_t(unsigned int, hba->dev_info.max_lu_supported,
					 UFS_SA_MAX_LUS) && host->nr_lus < nr_lus;
	     lun++) {
		int err = ufs_sa_publish_lu(host, lun);

		if (err == -ENODEV || err == -EOPNOTSUPP)
			continue;
		if (err)
			return err;
	}
	if (host->nr_lus != nr_lus)
		return -ENODEV;

	/*
	 * The SCSI device hierarchy normally keeps the controller runtime
	 * active while requests may be dispatched.  This direct frontend has
	 * no such hierarchy, and queue_rq() cannot synchronously resume a PCI
	 * device.  Keep one host reference for the lifetime of the published
	 * disks; PM policy is outside this analysis frontend's scope.
	 */
	pm_runtime_get_noresume(hba->dev);
	host->runtime_pm_held = true;
	return 0;
}

struct ufs_sa_timeout_set {
	unsigned long tags;
};

struct ufs_sa_lu_request_set {
	u8 lun;
	unsigned long tags;
};

static bool ufs_sa_collect_timeout(struct request *rq, void *data)
{
	struct ufs_sa_timeout_set *set = data;
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	unsigned long flags;

	spin_lock_irqsave(&pdu->state_lock, flags);
	if (pdu->state == UFS_SA_REQ_TIMED_OUT)
		__set_bit(rq->tag, &set->tags);
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	return true;
}

static struct request *ufs_sa_tag_to_rq(struct ufs_hba *hba, unsigned int tag)
{
	return blk_mq_tag_to_rq(hba->standalone_tag_set.shared_tags, tag);
}

static bool ufs_sa_commit_lu_request(struct request *rq, void *data)
{
	struct ufs_sa_lu_request_set *set = data;
	struct ufs_sa_lu *lu = rq->q->queuedata;
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	unsigned long flags;

	if (lu->lun != set->lun)
		return true;

	spin_lock_irqsave(&pdu->state_lock, flags);
	switch (pdu->state) {
	case UFS_SA_REQ_ACTIVE:
	case UFS_SA_REQ_TIMED_OUT:
	case UFS_SA_REQ_ABORT_COMMITTED:
		pdu->state = UFS_SA_REQ_LU_RESET_COMMITTED;
		__set_bit(rq->tag, &set->tags);
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	return true;
}

static bool ufs_sa_commit_controller_request(struct request *rq, void *data)
{
	struct ufs_sa_timeout_set *set = data;
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	unsigned long flags;

	spin_lock_irqsave(&pdu->state_lock, flags);
	switch (pdu->state) {
	case UFS_SA_REQ_ACTIVE:
	case UFS_SA_REQ_TIMED_OUT:
	case UFS_SA_REQ_ABORT_COMMITTED:
	case UFS_SA_REQ_LU_RESET_COMMITTED:
		pdu->state = UFS_SA_REQ_CONTROLLER_RESET_COMMITTED;
		__set_bit(rq->tag, &set->tags);
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	return true;
}

static void ufs_sa_release_hardware(struct ufs_hba *hba,
				    struct ufs_sa_pdu *pdu)
{
	unsigned long flags;
	bool release = false;

	spin_lock_irqsave(&pdu->state_lock, flags);
	if (!pdu->hardware_released) {
		pdu->hardware_released = true;
		release = true;
	}
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	if (release) {
		ufshcd_release(hba);
		ufshcd_clk_scaling_update_busy(hba);
	}
}

static int ufs_sa_clear_request(struct ufs_hba *hba, unsigned int tag)
{
	unsigned long flags;
	int error;

	error = ufshcd_clear_cmd(hba, tag);
	if (error || hba->mcq_enabled)
		return error;

	spin_lock_irqsave(&hba->outstanding_lock, flags);
	__clear_bit(tag, &hba->outstanding_reqs);
	spin_unlock_irqrestore(&hba->outstanding_lock, flags);
	return 0;
}

static enum ufs_sa_disposition ufs_sa_recovery_disposition(struct request *rq)
{
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	unsigned long flags;
	bool timed_out;

	if (ufs_sa_consume_retry(rq))
		return UFS_SA_DISPOSITION_REQUEUE;
	spin_lock_irqsave(&pdu->state_lock, flags);
	timed_out = pdu->timed_out_origin;
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	return timed_out ? UFS_SA_DISPOSITION_END_TIMEOUT :
		UFS_SA_DISPOSITION_END_IOERR;
}

static void ufs_sa_return_recovered(struct request *rq,
				    enum ufs_sa_disposition disposition)
{
	struct ufs_sa_lu *lu = rq->q->queuedata;
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	unsigned long flags;

	ufs_sa_release_hardware(lu->hba, pdu);
	ufs_sa_unmap(lu->hba, pdu);
	spin_lock_irqsave(&pdu->state_lock, flags);
	pdu->state = UFS_SA_REQ_IDLE;
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	if (disposition == UFS_SA_DISPOSITION_REQUEUE) {
		blk_mq_requeue_request(rq, true);
		return;
	}
	ufs_sa_reset_retries(pdu);
	blk_mq_end_request(rq,
		disposition == UFS_SA_DISPOSITION_END_TIMEOUT ?
		BLK_STS_TIMEOUT : BLK_STS_IOERR);
}

static int ufs_sa_abort_request(struct ufs_sa_host *host, struct request *rq)
{
	struct ufs_sa_lu *lu = rq->q->queuedata;
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	struct ufs_sa_tm_result result;
	unsigned long flags;

	result = ufs_sa_exec_tm(host, UFS_SA_TM_QUERY_TASK, lu->lun, rq->tag);
	if (result.error)
		return result.error;
	if (result.response != UPIU_TASK_MANAGEMENT_FUNC_SUCCEEDED)
		return -EAGAIN;

	spin_lock_irqsave(&pdu->state_lock, flags);
	if (pdu->state != UFS_SA_REQ_TIMED_OUT) {
		spin_unlock_irqrestore(&pdu->state_lock, flags);
		return 0;
	}
	/* Publishing this state is the logical ABORT commitment boundary. */
	pdu->state = UFS_SA_REQ_ABORT_COMMITTED;
	spin_unlock_irqrestore(&pdu->state_lock, flags);

	result = ufs_sa_exec_tm(host, UFS_SA_TM_ABORT_TASK, lu->lun, rq->tag);
	if (result.error ||
	    result.response != UPIU_TASK_MANAGEMENT_FUNC_COMPL)
		return result.error ? result.error : -EIO;

	spin_lock_irqsave(&pdu->state_lock, flags);
	if (pdu->state == UFS_SA_REQ_ABORT_COMMITTED) {
		int clear_error;

		spin_unlock_irqrestore(&pdu->state_lock, flags);
		clear_error = ufs_sa_clear_request(host->hba, rq->tag);
		spin_lock_irqsave(&pdu->state_lock, flags);
		if (clear_error && pdu->state != UFS_SA_REQ_RETIRED) {
			spin_unlock_irqrestore(&pdu->state_lock, flags);
			return -EIO;
		}
		pdu->state = UFS_SA_REQ_RETIRED;
	}
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	ufs_sa_return_recovered(rq, ufs_sa_recovery_disposition(rq));
	return 0;
}

static void ufs_sa_reconcile_completions(struct ufs_hba *hba)
{
	unsigned int queue;

	if (!hba->mcq_enabled) {
		ufshcd_poll(hba->host, 0);
		return;
	}
	for (queue = 0; queue < hba->nr_hw_queues; queue++)
		ufshcd_mcq_poll_cqe_lock(hba, &hba->uhq[queue]);
}

static int ufs_sa_retire_reset_request(struct ufs_sa_host *host,
				       struct request *rq)
{
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	unsigned long flags;
	int clear_error;

	clear_error = ufs_sa_clear_request(host->hba, rq->tag);
	spin_lock_irqsave(&pdu->state_lock, flags);
	if (clear_error && pdu->state != UFS_SA_REQ_RETIRED) {
		spin_unlock_irqrestore(&pdu->state_lock, flags);
		return clear_error;
	}
	pdu->state = UFS_SA_REQ_RETIRED;
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	ufs_sa_return_recovered(rq, ufs_sa_recovery_disposition(rq));
	return 0;
}

static int ufs_sa_recover_lu(struct ufs_sa_host *host, u8 lun)
{
	struct ufs_hba *hba = host->hba;
	struct ufs_sa_lu_request_set requests = { .lun = lun };
	struct ufs_sa_tm_result result;
	unsigned int tag;
	int error = 0;

	blk_mq_tagset_busy_iter(&hba->standalone_tag_set,
				ufs_sa_commit_lu_request, &requests);
	if (!requests.tags)
		return 0;

	result = ufs_sa_exec_tm(host, UFS_SA_TM_LOGICAL_RESET, lun, 0);
	if (result.error)
		return result.error;
	if (result.response != UPIU_TASK_MANAGEMENT_FUNC_COMPL)
		return -EIO;

	for_each_set_bit(tag, &requests.tags, hba->nutrs) {
		struct request *rq = ufs_sa_tag_to_rq(hba, tag);

		if (rq && ufs_sa_retire_reset_request(host, rq) && !error)
			error = -EIO;
	}
	return error;
}

static int ufs_sa_validate_lus(struct ufs_sa_host *host)
{
	unsigned int lun;

	for (lun = 0; lun < UFS_SA_MAX_LUS; lun++) {
		struct ufs_sa_lu *lu = &host->lus[lun];
		u64 block_count;
		u32 block_size;
		int error;

		if (!lu->published)
			continue;
		error = ufs_sa_read_lu_geometry(host->hba, lun, &block_count,
					       &block_size);
		if (error)
			return error;
		if (block_count != lu->block_count || block_size != lu->block_size)
			return -ESTALE;
	}
	return 0;
}

static void ufs_sa_finish_controller_request(struct ufs_sa_host *host,
					     struct request *rq,
					     bool restored)
{
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	enum ufs_sa_disposition disposition;
	unsigned long flags;

	spin_lock_irqsave(&pdu->state_lock, flags);
	pdu->state = UFS_SA_REQ_RETIRED;
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	ufs_sa_release_hardware(host->hba, pdu);
	ufs_sa_unmap(host->hba, pdu);
	spin_lock_irqsave(&pdu->state_lock, flags);
	pdu->state = UFS_SA_REQ_IDLE;
	spin_unlock_irqrestore(&pdu->state_lock, flags);
	if (!restored) {
		ufs_sa_reset_retries(pdu);
		blk_mq_end_request(rq, BLK_STS_IOERR);
		return;
	}
	disposition = ufs_sa_recovery_disposition(rq);
	if (disposition == UFS_SA_DISPOSITION_REQUEUE) {
		blk_mq_requeue_request(rq, true);
		return;
	}
	ufs_sa_reset_retries(pdu);
	blk_mq_end_request(rq,
		disposition == UFS_SA_DISPOSITION_END_TIMEOUT ?
		BLK_STS_TIMEOUT : BLK_STS_IOERR);
}

static int ufs_sa_restore_controller(struct ufs_sa_host *host)
{
	struct ufs_hba *hba = host->hba;
	struct ufs_sa_timeout_set requests = { };
	ktime_t probe_start;
	unsigned int tag;
	int error;

	blk_mq_tagset_busy_iter(&hba->standalone_tag_set,
				ufs_sa_commit_controller_request, &requests);

	ufshcd_device_reset(hba);
	ufshcd_hba_stop(hba);
	if (ufshcd_is_hba_active(hba))
		return -EBUSY;

	error = ufshcd_hba_enable(hba);
	if (!error)
		error = ufshcd_device_init(hba, false);
	probe_start = ktime_get();
	if (!error)
		error = ufshcd_probe_hba(hba, false);
	ufshcd_process_probe_result(hba, probe_start, error);
	if (!error)
		error = ufs_sa_validate_lus(host);

	for_each_set_bit(tag, &requests.tags, hba->nutrs) {
		struct request *rq = ufs_sa_tag_to_rq(hba, tag);

		if (rq)
			ufs_sa_finish_controller_request(host, rq, !error);
	}
	return error;
}

static void ufs_sa_recovery_work(struct work_struct *work)
{
	struct ufs_sa_host *host = container_of(work, struct ufs_sa_host,
						 recovery_work);
	struct ufs_hba *hba = host->hba;
	struct ufs_sa_timeout_set timed_out = { };
	unsigned long failed_lus = 0;
	unsigned int tag;
	bool failed = false;
	bool controller_recovery = false;

	mutex_lock(&host->recovery_lock);
	WRITE_ONCE(host->state, UFS_SA_HOST_RECOVERING);
	blk_mq_quiesce_tagset(&hba->standalone_tag_set);
	ufs_sa_reconcile_completions(hba);
	blk_mq_tagset_busy_iter(&hba->standalone_tag_set,
				ufs_sa_collect_timeout, &timed_out);
	for_each_set_bit(tag, &timed_out.tags, hba->nutrs) {
		struct request *rq = ufs_sa_tag_to_rq(hba, tag);

		if (rq && ufs_sa_abort_request(host, rq)) {
			dev_err(hba->dev, "task recovery failed for tag %u\n", tag);
			__set_bit(((struct ufs_sa_lu *)rq->q->queuedata)->lun,
				  &failed_lus);
		}
	}
	for_each_set_bit(tag, &failed_lus, UFS_SA_MAX_LUS) {
		if (ufs_sa_recover_lu(host, tag)) {
			dev_err(hba->dev, "LU recovery failed for LUN %u\n", tag);
			controller_recovery = true;
		}
	}
	if (controller_recovery && ufs_sa_restore_controller(host)) {
		dev_err(hba->dev, "controller restoration failed\n");
		failed = true;
	}
	/* Keep dispatch stopped when no safe retirement proof was obtained. */
	if (!failed) {
		ufs_sa_reconcile_completions(hba);
		WRITE_ONCE(host->state, UFS_SA_HOST_ONLINE);
		blk_mq_unquiesce_tagset(&hba->standalone_tag_set);
	} else {
		WRITE_ONCE(host->state, UFS_SA_HOST_FAILED);
	}
	mutex_unlock(&host->recovery_lock);
}

static int ufs_sa_init(struct ufs_hba *hba)
{
	struct ufs_sa_host *host;
	struct blk_mq_tag_set *set = &hba->standalone_tag_set;
	int ret;

	host = devm_kzalloc(hba->dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	host->hba = hba;
	mutex_init(&host->tm_lock);
	mutex_init(&host->recovery_lock);
	INIT_WORK(&host->recovery_work, ufs_sa_recovery_work);
	host->state = UFS_SA_HOST_ONLINE;
	hba->standalone_host = host;
	memset(set, 0, sizeof(*set));
	set->ops = &ufs_sa_mq_ops;
	set->nr_hw_queues = max_t(unsigned int, 1, hba->nr_hw_queues);
	set->nr_maps = HCTX_MAX_TYPES;
	set->queue_depth = hba->nutrs;
	set->reserved_tags = UFSHCD_NUM_RESERVED;
	set->cmd_size = sizeof(struct ufs_sa_pdu);
	set->numa_node = dev_to_node(hba->dev);
	set->flags = BLK_MQ_F_TAG_HCTX_SHARED;
	set->driver_data = hba;
	ret = blk_mq_alloc_tag_set(set);
	if (ret)
		return ret;
	hba->standalone_tag_set_allocated = true;
	return 0;
}

static void ufs_sa_exit(struct ufs_hba *hba)
{
	struct ufs_sa_host *host = ufs_sa_host(hba);
	unsigned int lun;

	if (!host)
		return;
	cancel_work_sync(&host->recovery_work);
	for (lun = 0; lun < UFS_SA_MAX_LUS; lun++) {
		struct ufs_sa_lu *lu = &host->lus[lun];

		if (!lu->published)
			continue;
		del_gendisk(lu->disk);
		put_disk(lu->disk);
		lu->published = false;
	}
	if (hba->standalone_tag_set_allocated) {
		hba->standalone_tag_set_allocated = false;
		blk_mq_free_tag_set(&hba->standalone_tag_set);
	}
	if (host->runtime_pm_held) {
		host->runtime_pm_held = false;
		pm_runtime_put_noidle(hba->dev);
	}
}

static bool ufs_sa_is_request(const struct scsi_cmnd *cmd)
{
	const struct ufshcd_lrb *lrb = scsi_cmd_priv((struct scsi_cmnd *)cmd);

	return lrb->standalone_req;
}

static void ufs_sa_complete_from_hardware(struct ufs_hba *hba,
					  struct scsi_cmnd *cmd,
					  struct cq_entry *cqe)
{
	struct request *rq = scsi_cmd_to_rq(cmd);
	struct ufs_sa_pdu *pdu = ufs_sa_pdu(rq);
	unsigned long flags;
	bool complete = false;

	spin_lock_irqsave(&pdu->state_lock, flags);
	switch (pdu->state) {
	case UFS_SA_REQ_ACTIVE:
	case UFS_SA_REQ_TIMED_OUT:
		/* A completion observed before ABORT commitment still wins. */
		pdu->state = UFS_SA_REQ_RETIRED;
		complete = true;
		break;
	case UFS_SA_REQ_ABORT_COMMITTED:
	case UFS_SA_REQ_LU_RESET_COMMITTED:
	case UFS_SA_REQ_CONTROLLER_RESET_COMMITTED:
		/* Recovery owns disposition after either commitment boundary. */
		pdu->state = UFS_SA_REQ_RETIRED;
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&pdu->state_lock, flags);

	ufs_sa_release_hardware(hba, pdu);
	if (complete) {
		cmd->result = ufshcd_transfer_rsp_status(hba, cmd, cqe);
		blk_mq_complete_request(rq);
	}
}
