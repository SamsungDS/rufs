// SPDX-License-Identifier: GPL-2.0-or-later
/* Analysis-only direct blk-mq frontend. Included by ufshcd-standalone.c. */

#define UFS_SA_MAX_LUS UFS_UPIU_MAX_UNIT_NUM_ID
#define UFS_SA_INLINE_SG 4

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
};

struct ufs_sa_pdu {
	struct scsi_cmnd cmd;
	struct ufshcd_lrb lrb;
	struct scatterlist inline_sg[UFS_SA_INLINE_SG];
	u8 sense[SCSI_SENSE_BUFFERSIZE];
};

static struct ufs_sa_host *ufs_sa_host(struct ufs_hba *hba)
{
	return hba->standalone_host;
}

static struct ufs_sa_pdu *ufs_sa_pdu(struct request *rq)
{
	return blk_mq_rq_to_pdu(rq);
}

static void ufs_sa_unmap(struct ufs_hba *hba, struct scsi_cmnd *cmd)
{
	if (!cmd->sdb.table.nents)
		return;
	dma_unmap_sg(hba->dev, cmd->sdb.table.sgl, cmd->sdb.table.orig_nents,
		     cmd->sc_data_direction);
	sg_free_table_chained(&cmd->sdb.table, UFS_SA_INLINE_SG);
	cmd->sdb.table.nents = 0;
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
	cmd->sdb.table.orig_nents = cmd->sdb.table.nents;
	cmd->sdb.length = blk_rq_payload_bytes(rq);
	mapped = dma_map_sg(hba->dev, cmd->sdb.table.sgl,
			    cmd->sdb.table.orig_nents, cmd->sc_data_direction);
	if (!mapped) {
		sg_free_table_chained(&cmd->sdb.table, UFS_SA_INLINE_SG);
		cmd->sdb.table.nents = 0;
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

	if (READ_ONCE(hba->ufshcd_state) != UFSHCD_STATE_OPERATIONAL)
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
	ufshcd_setup_scsi_cmd(hba, cmd, lu->lun, rq->tag);
	lrb->standalone_req = true;
	ufshcd_sgl_to_prdt(hba, lrb, cmd->sdb.table.nents,
			    cmd->sdb.table.sgl);
	if (hba->mcq_enabled)
		hwq = ufshcd_mcq_req_to_hwq(hba, rq);
	blk_mq_start_request(rq);
	ufshcd_send_command(hba, cmd, hwq);
	return BLK_STS_OK;
}

static void ufs_sa_complete(struct request *rq)
{
	struct ufs_sa_lu *lu = rq->q->queuedata;
	struct scsi_cmnd *cmd = blk_mq_rq_to_pdu(rq);
	blk_status_t status = cmd->result ? BLK_STS_IOERR : BLK_STS_OK;

	ufs_sa_unmap(lu->hba, cmd);
	blk_mq_end_request(rq, status);
}

static enum blk_eh_timer_return ufs_sa_timeout(struct request *rq)
{
	return BLK_EH_RESET_TIMER;
}

static int ufs_sa_init_request(struct blk_mq_tag_set *set, struct request *rq,
			       unsigned int hctx_idx, int numa_node)
{
	memset(ufs_sa_pdu(rq), 0, sizeof(struct ufs_sa_pdu));
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
				   struct ufs_sa_lu *lu)
{
	__be64 block_count;
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
					  (u8 *)&block_count, sizeof(block_count));
	if (ret)
		return ret;
	lu->block_size = 1U << block_shift;
	lu->block_count = be64_to_cpu(block_count);
	return lu->block_size < SECTOR_SIZE ? -EINVAL : 0;
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
	ret = ufs_sa_read_lu_geometry(host->hba, lun, lu);
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
	lu->disk->major = 0;
	lu->disk->first_minor = 0;
	lu->disk->minors = 1;
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
	unsigned int lun;
	int ret = 0;

	for (lun = 0; lun < min_t(unsigned int, hba->dev_info.max_lu_supported,
					 UFS_SA_MAX_LUS); lun++) {
		int err = ufs_sa_publish_lu(host, lun);

		if (err == -ENODEV || err == -EOPNOTSUPP)
			continue;
		if (err && !ret)
			ret = err;
	}
	return host->nr_lus ? 0 : ret ? ret : -ENODEV;
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
}

static bool ufs_sa_is_request(const struct scsi_cmnd *cmd)
{
	const struct ufshcd_lrb *lrb = scsi_cmd_priv((struct scsi_cmnd *)cmd);

	return lrb->standalone_req;
}

static void ufs_sa_complete_from_hardware(struct ufs_hba *hba,
					  struct scsi_cmnd *cmd)
{
	ufshcd_release(hba);
	ufshcd_clk_scaling_update_busy(hba);
	blk_mq_complete_request(scsi_cmd_to_rq(cmd));
}
