/*
* Copyright (C) 2012-2013 CERN (www.cern.ch)
* Author: Juan David Gonzalez Cobas <dcobas@cern.ch>
* Author: Luis Fernando Ruiz Gago <lfruiz@cern.ch>
*
* Released according to the GNU GPL, version 2 or any later version
*
* Driver for SVEC (Simple VME FMC carrier) board.
*/
#ifndef __SVEC_H__
#define __SVEC_H__

#include <linux/firmware.h>
#include <linux/fmc.h>
#include <vmebus.h>

#define SVEC_MAX_DEVICES        32
#define SVEC_DEFAULT_IDX { [0 ... (SVEC_MAX_DEVICES-1)] = -1 }
#define SVEC_IRQ_LEVEL	2
#define SVEC_N_SLOTS	2
#define SVEC_BASE_LOADER	0x70000
#define SVEC_VENDOR_ID		0x80030

#define VME_VENDOR_ID_OFFSET	0x24

/* The eeprom is at address 0x50 */
/* FIXME ? Copied from spec.h */
#define SVEC_I2C_EEPROM_SIZE (8 * 1024)

enum svec_map_win {
	MAP_CR_CSR = 0,	/* CR/CSR */
	MAP_REG		/* A32 space */
};

/* Our device structure */
struct svec_dev {
	int			lun;
	int			slot;
	uint32_t		vmebase;
	int			vector;
	int			level;

	char			*fw_name;
	struct device		*dev;
	char			driver[16];
	char			description[80];
	uint32_t		fw_hash;	
	struct vme_mapping	*map[2];

	/* struct work_struct	work; */
	unsigned long		irqcount;
	struct fmc_device	*fmcs[SVEC_N_SLOTS];
						/* FMC devices */
	int			fmcs_n;		/* Number of FMC devices */
	int			irq_count;	/* for mezzanine use too */
};

/* Functions and data in svec-vme.c */
extern int svec_is_bootloader_active(struct svec_dev *svec);
extern int svec_bootloader_unlock (struct svec_dev *svec);
extern int svec_load_fpga(struct svec_dev *svec, const void *data, int size);
extern int svec_load_fpga_file(struct svec_dev *svec, const char *name);
extern void svec_setup_csr_fa0(void *base, u32 vme, unsigned vector,
			       unsigned level);
extern int svec_unmap_window(struct svec_dev *svec, enum svec_map_win map_type);
extern int svec_map_window( struct svec_dev *svec, enum svec_map_win map_type);

extern char *svec_fw_name;
extern int spec_use_msi;

/* Functions in svec-fmc.c, used by svec-vme.c */
extern int svec_fmc_create(struct svec_dev *svec);
extern void svec_fmc_destroy(struct svec_dev *svec);

/* Functions in svec-i2c.c, used by svec-fmc.c */
extern int svec_i2c_init(struct fmc_device *fmc);
extern void svec_i2c_exit(struct fmc_device *fmc);
extern int svec_eeprom_read(struct fmc_device *fmc, uint32_t offset,
			    void *buf, size_t size);
extern int svec_eeprom_write(struct fmc_device *fmc, uint32_t offset,
			     const void *buf, size_t size);

/* SVEC CSR offsets */
#define FUN0ADER	0x7FF63
#define INT_LEVEL	0x7ff5b
#define INTVECTOR	0x7ff5f
#define WB_32_64	0x7ff33
#define BIT_SET_REG	0x7FFFB
#define BIT_CLR_REG	0x7FFF7
#define WB32		1
#define WB64		0
#define RESET_CORE	0x80
#define ENABLE_CORE	0x10

/* Functions in svec-sysfs.c */
extern int svec_create_sysfs_files(struct svec_dev *card);
extern void svec_remove_sysfs_files(struct svec_dev *card);

#endif /* __SVEC_H__ */
