// SPDX-License-Identifier: GPL-2.0-only
/*
 * OF helpers for IOMMU
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/export.h>
#include <linux/iommu.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_iommu.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/fsl/mc.h>

static int of_iommu_xlate(struct iommu_fwspec *fwspec, struct device *dev,
			  struct of_phandle_args *iommu_spec)
{
	if (!of_device_is_available(iommu_spec->np))
		return -ENODEV;

	return iommu_fwspec_of_xlate(fwspec, dev, &iommu_spec->np->fwnode,
				     iommu_spec);
}

static int of_iommu_configure_dev_id(struct iommu_fwspec *fwspec,
				     struct device_node *master_np,
				     struct device *dev, const u32 *id)
{
	struct of_phandle_args iommu_spec = { .args_count = 1 };
	int err;

	err = of_map_id(master_np, *id, "iommu-map",
			 "iommu-map-mask", &iommu_spec.np,
			 iommu_spec.args);
	if (err)
		return err;

	err = of_iommu_xlate(fwspec, dev, &iommu_spec);
	of_node_put(iommu_spec.np);
	return err;
}

static int of_iommu_configure_dev(struct iommu_fwspec *fwspec,
				  struct device_node *master_np,
				  struct device *dev)
{
	struct of_phandle_args iommu_spec;
	int err = -ENODEV, idx = 0;

	while (!of_parse_phandle_with_args(master_np, "iommus",
					   "#iommu-cells",
					   idx, &iommu_spec)) {
		err = of_iommu_xlate(fwspec, dev, &iommu_spec);
		of_node_put(iommu_spec.np);
		idx++;
		if (err)
			break;
	}

	return err;
}

struct of_pci_iommu_alias_info {
	struct device *dev;
	struct device_node *np;
	struct iommu_fwspec *fwspec;
};

static int of_pci_iommu_init(struct pci_dev *pdev, u16 alias, void *data)
{
	struct of_pci_iommu_alias_info *info = data;
	u32 input_id = alias;

	return of_iommu_configure_dev_id(info->fwspec, info->np, info->dev,
					 &input_id);
}

static int of_iommu_configure_device(struct iommu_fwspec *fwspec,
				     struct device_node *master_np,
				     struct device *dev, const u32 *id)
{
	return (id) ? of_iommu_configure_dev_id(fwspec, master_np, dev, id) :
		      of_iommu_configure_dev(fwspec, master_np, dev);
}

/*
 * Returns:
 *  0 on success, an iommu was configured
 *  -ENODEV if the device does not have any IOMMU
 *  -EPROBEDEFER if probing should be tried again
 *  -errno fatal errors
 */
int of_iommu_configure(struct device *dev, struct device_node *master_np,
		       const u32 *id)
{
	struct iommu_fwspec *fwspec;
	int err;

	if (!master_np)
		return -ENODEV;

	fwspec = iommu_fwspec_alloc();
	if (IS_ERR(fwspec))
		return PTR_ERR(fwspec);

	/*
	 * We don't currently walk up the tree looking for a parent IOMMU.
	 * See the `Notes:' section of
	 * Documentation/devicetree/bindings/iommu/iommu.txt
	 */
	if (dev_is_pci(dev)) {
		struct of_pci_iommu_alias_info info = {
			.dev = dev,
			.np = master_np,
			.fwspec = fwspec,
		};

		pci_request_acs();
		err = pci_for_each_dma_alias(to_pci_dev(dev),
					     of_pci_iommu_init, &info);
	} else {
		err = of_iommu_configure_device(fwspec, master_np, dev, id);
	}

	if (err == -ENODEV || err == -EPROBE_DEFER)
		goto err_free;
	if (err)
		goto err_log;

	err = iommu_probe_device_fwspec(dev, fwspec);
	if (err) {
		/*
		 * Ownership for fwspec always passes into
		 * iommu_probe_device_fwspec()
		 */
		fwspec = NULL;
		goto err_log;
	}
	return 0;

err_log:
	dev_dbg(dev, "Adding to IOMMU failed: %pe\n", ERR_PTR(err));
err_free:
	iommu_fwspec_dealloc(fwspec);
	return err;
}

static enum iommu_resv_type __maybe_unused
iommu_resv_region_get_type(struct device *dev,
			   struct resource *phys,
			   phys_addr_t start, size_t length)
{
	phys_addr_t end = start + length - 1;

	/*
	 * IOMMU regions without an associated physical region cannot be
	 * mapped and are simply reservations.
	 */
	if (phys->start >= phys->end)
		return IOMMU_RESV_RESERVED;

	/* may be IOMMU_RESV_DIRECT_RELAXABLE for certain cases */
	if (start == phys->start && end == phys->end)
		return IOMMU_RESV_DIRECT;

	return IOMMU_RESV_TRANSLATED;
}

/**
 * of_iommu_get_resv_regions - reserved region driver helper for device tree
 * @dev: device for which to get reserved regions
 * @list: reserved region list
 *
 * IOMMU drivers can use this to implement their .get_resv_regions() callback
 * for memory regions attached to a device tree node. See the reserved-memory
 * device tree bindings on how to use these:
 *
 *   Documentation/devicetree/bindings/reserved-memory/reserved-memory.txt
 */
void of_iommu_get_resv_regions(struct device *dev, struct list_head *list)
{
#if IS_ENABLED(CONFIG_OF_ADDRESS)
	struct of_phandle_iterator it;
	int err;

	of_for_each_phandle(&it, err, dev->of_node, "memory-region", NULL, 0) {
		const __be32 *maps, *end;
		struct resource phys;
		int size;

		memset(&phys, 0, sizeof(phys));

		/*
		 * The "reg" property is optional and can be omitted by reserved-memory regions
		 * that represent reservations in the IOVA space, which are regions that should
		 * not be mapped.
		 */
		if (of_find_property(it.node, "reg", NULL)) {
			err = of_address_to_resource(it.node, 0, &phys);
			if (err < 0) {
				dev_err(dev, "failed to parse memory region %pOF: %d\n",
					it.node, err);
				continue;
			}
		}

		maps = of_get_property(it.node, "iommu-addresses", &size);
		if (!maps)
			continue;

		end = maps + size / sizeof(__be32);

		while (maps < end) {
			struct device_node *np;
			u32 phandle;

			phandle = be32_to_cpup(maps++);
			np = of_find_node_by_phandle(phandle);

			if (np == dev->of_node) {
				int prot = IOMMU_READ | IOMMU_WRITE;
				struct iommu_resv_region *region;
				enum iommu_resv_type type;
				phys_addr_t iova;
				size_t length;

				maps = of_translate_dma_region(np, maps, &iova, &length);
				type = iommu_resv_region_get_type(dev, &phys, iova, length);

				if (type == IOMMU_RESV_TRANSLATED)
					region = iommu_alloc_resv_region_tr(phys.start, iova, length, prot, type,
								    GFP_KERNEL);
				else
					region = iommu_alloc_resv_region(iova, length, prot, type,
								 GFP_KERNEL);

				if (region)
					list_add_tail(&region->list, list);
			}
		}
	}
#endif
}
EXPORT_SYMBOL(of_iommu_get_resv_regions);
