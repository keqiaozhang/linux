// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Authors: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//	    Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
//	    Rander Wang <rander.wang@intel.com>
//          Keyon Jie <yang.jie@linux.intel.com>
//

/*
 * Hardware interface for generic Intel audio DSP HDA IP
 */

#include <sound/hdaudio_ext.h>
#include <sound/hda_register.h>
#include "../ops.h"
#include "hda.h"

/*
 * HDA Operations.
 */

int hda_dsp_ctrl_link_reset(struct snd_sof_dev *sdev, bool reset)
{
	unsigned long timeout;
	u32 gctl = 0;
	u32 val;

	/* 0 to enter reset and 1 to exit reset */
	val = reset ? 0 : SOF_HDA_GCTL_RESET;

	/* enter/exit HDA controller reset */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_HDA_BAR, SOF_HDA_GCTL,
				SOF_HDA_GCTL_RESET, val);

	/* wait to enter/exit reset */
	timeout = jiffies + msecs_to_jiffies(HDA_DSP_CTRL_RESET_TIMEOUT);
	while (time_before(jiffies, timeout)) {
		gctl = snd_sof_dsp_read(sdev, HDA_DSP_HDA_BAR, SOF_HDA_GCTL);
		if ((gctl & SOF_HDA_GCTL_RESET) == val)
			return 0;
		usleep_range(500, 1000);
	}

	/* enter/exit reset failed */
	dev_err(sdev->dev, "error: failed to %s HDA controller gctl 0x%x\n",
		reset ? "reset" : "ready", gctl);
	return -EIO;
}

int hda_dsp_ctrl_get_caps(struct snd_sof_dev *sdev)
{
	struct hdac_bus *bus = sof_to_bus(sdev);
	u32 cap, offset, feature;
	int count = 0;

	offset = snd_sof_dsp_read(sdev, HDA_DSP_HDA_BAR, SOF_HDA_LLCH);

	do {
		cap = snd_sof_dsp_read(sdev, HDA_DSP_HDA_BAR, offset);

		dev_dbg(sdev->dev, "checking for capabilities at offset 0x%x\n",
			offset & SOF_HDA_CAP_NEXT_MASK);

		feature = (cap & SOF_HDA_CAP_ID_MASK) >> SOF_HDA_CAP_ID_OFF;

		switch (feature) {
		case SOF_HDA_PP_CAP_ID:
			dev_dbg(sdev->dev, "found DSP capability at 0x%x\n",
				offset);
			bus->ppcap = bus->remap_addr + offset;
			sdev->bar[HDA_DSP_PP_BAR] = bus->ppcap;
			break;
		case SOF_HDA_SPIB_CAP_ID:
			dev_dbg(sdev->dev, "found SPIB capability at 0x%x\n",
				offset);
			bus->spbcap = bus->remap_addr + offset;
			sdev->bar[HDA_DSP_SPIB_BAR] = bus->spbcap;
			break;
		case SOF_HDA_DRSM_CAP_ID:
			dev_dbg(sdev->dev, "found DRSM capability at 0x%x\n",
				offset);
			bus->drsmcap = bus->remap_addr + offset;
			sdev->bar[HDA_DSP_DRSM_BAR] = bus->drsmcap;
			break;
		case SOF_HDA_GTS_CAP_ID:
			dev_dbg(sdev->dev, "found GTS capability at 0x%x\n",
				offset);
			bus->gtscap = bus->remap_addr + offset;
			break;
		case SOF_HDA_ML_CAP_ID:
			dev_dbg(sdev->dev, "found ML capability at 0x%x\n",
				offset);
			bus->mlcap = bus->remap_addr + offset;
			break;
		default:
			dev_vdbg(sdev->dev, "found capability %d at 0x%x\n",
				 feature, offset);
			break;
		}

		offset = cap & SOF_HDA_CAP_NEXT_MASK;
	} while (count++ <= SOF_HDA_MAX_CAPS && offset);

	return 0;
}

void hda_dsp_ctrl_misc_clock_gating(struct snd_sof_dev *sdev, bool enable)
{
	u32 val = enable ? PCI_CGCTL_MISCBDCGE_MASK : 0;

	snd_sof_pci_update_bits(sdev, PCI_CGCTL, PCI_CGCTL_MISCBDCGE_MASK, val);
}

/*
 * enable/disable audio dsp clock gating and power gating bits.
 * This allows the HW to opportunistically power and clock gate
 * the audio dsp when it is idle
 */
int hda_dsp_ctrl_clock_power_gating(struct snd_sof_dev *sdev, bool enable)
{
#if IS_ENABLED(CONFIG_SND_SOC_SOF_HDA)
	struct hdac_bus *bus = sof_to_bus(sdev);
#endif
	u32 val;

	/* enable/disable audio dsp clock gating */
	val = enable ? PCI_CGCTL_ADSPDCGE : 0;
	snd_sof_pci_update_bits(sdev, PCI_CGCTL, PCI_CGCTL_ADSPDCGE, val);

#if IS_ENABLED(CONFIG_SND_SOC_SOF_HDA)
	/* enable/disable L1 support */
	val = enable ? SOF_HDA_VS_EM2_L1SEN : 0;
	snd_hdac_chip_updatel(bus, VS_EM2, SOF_HDA_VS_EM2_L1SEN, val);
#endif

	/* enable/disable audio dsp power gating */
	val = enable ? 0 : PCI_PGCTL_ADSPPGD;
	snd_sof_pci_update_bits(sdev, PCI_PGCTL, PCI_PGCTL_ADSPPGD, val);

	return 0;
}

#if IS_ENABLED(CONFIG_SND_SOC_SOF_HDA)
/*
 * While performing reset, controller may not come back properly and causing
 * issues, so recommendation is to set CGCTL.MISCBDCGE to 0 then do reset
 * (init chip) and then again set CGCTL.MISCBDCGE to 1
 */
static int show_stream = 7;
static void dump_hda_registers(struct snd_sof_dev *sdev)
{
        int i, end;
       u32 val, gcap;
       int bar = HDA_DSP_HDA_BAR;
       int stream, hl_idx;
       int num_playback, num_capture, num_total;
       struct hdac_bus *bus = sof_to_bus(sdev);

       stream = show_stream;
       hl_idx = 0; //on apl it is 0

       val = snd_hdac_chip_readl(bus, GCAP);
       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", 0x0, val);
       val = snd_hdac_chip_readl(bus, GCTL);
       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", 0x08, val);
       val = snd_hdac_chip_readl(bus, WAKEEN);
       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", 0x0c, val);
       /* dump Global Registers */
       for (i = 0; i < 0x20; i=i+4) {
	 val = snd_sof_dsp_read(sdev, bar, i);
	 dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
       }

       /* dump Interrupt Registers */
       for (i = 0x20; i < 0x30; i=i+4) {
               val = snd_sof_dsp_read(sdev, bar, i);
	       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
       }

       /* dump Controller Registers */
       i = 0x38;
       val = snd_sof_dsp_read(sdev, bar, i);
       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
       for (i = 0x40; i < 0x78; i=i+4) {
               val = snd_sof_dsp_read(sdev, bar, i);
	       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
       }

       /* dump sdx Registers */
       i = 0x80 + 0x20 * stream;
       end = i + 0x20;
       for (; i < end; i=i+4) {
               val = snd_sof_dsp_read(sdev, bar, i);
	       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
       }

       /* dump PP (Processing Pipe) Capability */
       i = 0x804;
       val = snd_sof_dsp_read(sdev, bar, i);
       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);

       i = 0x808;
       val = snd_sof_dsp_read(sdev, bar, i);
       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);

       gcap = snd_sof_dsp_read(sdev, HDA_DSP_HDA_BAR, SOF_HDA_GCAP);
       num_capture = (gcap >> 8) & 0x0f;
       num_playback = (gcap >> 12) & 0x0f;
       num_total = num_playback + num_capture;
       i = 0x810 + 0x10 * stream;
       end = i + 0x10;
       for (; i < end; i=i+4) {
               val = snd_sof_dsp_read(sdev, bar, i);
	       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
       }
       i = 0x810 + 0x10 * num_total + 0x10 * stream;
       end = i + 0x10;
       for (; i < end; i=i+4) {
               val = snd_sof_dsp_read(sdev, bar, i);
	       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
       }

       /* dump ML Registers */
       i = 0xc40 + 0x40*hl_idx;
       end = i + 0x1c;
       for (; i < end; i=i+4) {
               val = snd_sof_dsp_read(sdev, bar, i);
	       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
       }

       /* dump Vendor Specific Registers */
       i = 0x104a;
       val = snd_sof_dsp_read(sdev, bar, i);
       dev_err(sdev->dev, "ylb, 0x%4x: 0x%8x\n", i, val);
}
int hda_dsp_ctrl_init_chip(struct snd_sof_dev *sdev, bool full_reset)
{
	struct hdac_bus *bus = sof_to_bus(sdev);
	int ret;

	hda_dsp_ctrl_misc_clock_gating(sdev, false);

	dev_err(sdev->dev, "in %s %d ylb\n", __func__, __LINE__);
	dump_hda_registers(sdev);
	ret = snd_hdac_bus_init_chip(bus, full_reset);
	dump_hda_registers(sdev);
	hda_dsp_ctrl_misc_clock_gating(sdev, true);

	return ret;
}
#endif
